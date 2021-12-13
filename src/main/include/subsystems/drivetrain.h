#pragma once

#include "subsystems/Subsystem.h"
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <frc/controller/RamseteController.h>
#include <rclcpp/rclcpp.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

#include "frc/kinematics/DifferentialDriveOdometry.h"

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>

namespace robot
{

    /**
     * Possible control states for the drivetrain to be in
     **/
    enum ControlState
    {
        OPEN_LOOP_STICK,
        VELOCITY_TWIST,
        RAMSETE,
        PURSUIT
    };

    /**
     * Possible control states for the transmissions to be in
     **/
    enum ShifterState
    {
        DISABLED,
        VELOCITY_THRESH,
        FIXED_HIGH,
        FIXED_LOW,
        USER_INPUT
    };

    class Drivetrain : public Subsystem
    {
        public:

        Drivetrain();

        /**
         * Override this function in order to create pulbishers or subscribers against the parent node.
         * NOTE: This function is automatically called by the subsystem manager on registration
         **/
        void createRosBindings(rclcpp::Node *node) override;

        /**
         * Override this function with all the nessecary code needed to reset a subsystem
         **/
        void reset() override;

        /**
         * Overrride this function with any code needed to be called only once on the first onloop iteration
         **/
        void onStart() override;

        /**
         * Override this function for any code that must be called periodically by the subsystem
         **/
        void onLoop() override;

        /**
         * Override this function with code needed to publish all data out to the ros network
         **/
        void publishData() override;

        /**
         * Callbacks for ROS Subscribers 
         **/

        /**
         * Callback for streaming generated trajectories into the trajectory follower
         **/
        void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

        /**
         * Calbback for streaming the current desired velocity twist of the drivetrain
         **/
        void twistCallback(const geometry_msgs::msg::Twist msg);

        /**
         * Callback for streaming joystick input into the drivetrain
         **/ 
        void stickCallback(const sensor_msgs::msg::Joy msg);

        /**
         * Callback for setting transmission mode between the control modes.
         * See ShifterState for availiable modes.
         **/
        void transModeCallback(const std_msgs::msg::Int16 msg);

        /**
         * Callback for setting drivetrain modes. Selects between the control modes
         * enumerated in the ControlState enum.
         **/
        void driveModeCallback(const std_msgs::msg::Int16 msg);

    private:
        
    /**
     * Converts a roll pitch and yaw to a quaternion as specified in this paper
     * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles 
     * @param roll the roll value to be converted
     * @param pitch the pitch value to be converted
     * @param yaw the yaw value to be converted
     * @return the quaternion to be converted from rpy format
     **/
    geometry_msgs::msg::Quaternion toQuaternion(double roll, double pitch, double yaw) // yaw (Z), pitch (Y), roll (X)
    {
        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        geometry_msgs::msg::Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    /**
     * Converts a quaterion q into a roll pitch and yaw stored at the locations specified
     * @param q the quaternion to be converted into rpy format
     * @param roll the address to save the roll value to
     * @param pitch the address to save the pitch value to
     * @param yaw the address to save the yaw value to
     **/
    void toRollPitchYaw(geometry_msgs::msg::Quaternion q, double* roll, double* pitch, double* yaw) {
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        *roll = std::atan2(sinr_cosp, cosr_cosp);
        // pitch (y-axis rotation)
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            *pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            *pitch = std::asin(sinp);
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        *yaw = std::atan2(siny_cosp, cosy_cosp);
        return;
    }
    /**
     *  quaternion multiplication ORDER MATTERS!!
     **/
    geometry_msgs::msg::Quaternion hamiltonProduct(geometry_msgs::msg::Quaternion q1, geometry_msgs::msg::Quaternion q2){
        geometry_msgs::msg::Quaternion r;
        r.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
        r.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
        r.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
        r.z = q1.w * q2.z - q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
        return r;
        
    }

        /**
         * Configure the associated motor controllers with their settings as specified in constants
         **/ 
        void configMotors();

        void updateSensorData();

        //IO devices
        std::shared_ptr<TalonFX> leftMaster, rightMaster, leftFollower, rightFollower;
        std::shared_ptr<frc::DoubleSolenoid> shifter;
        std::shared_ptr<PigeonIMU> imu;

        // ROS Publishers
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheelStatePub;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr poseStatePub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yawPub;

        // ROS Messages for publishing
        sensor_msgs::msg::Imu imuMsg;
        sensor_msgs::msg::JointState wheelState;
        geometry_msgs::msg::Pose poseState;
        std_msgs::msg::Float32 yawMsg;

        // ROS Subscibers
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectorySub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr stickSub;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr transModeSub, DriveModeSub;

        // ROS Messages for storing subscription data
        geometry_msgs::msg::Twist lastTwist;
        sensor_msgs::msg::Joy lastStick;

        // underlying controllers
        std::shared_ptr<frc::DifferentialDriveOdometry> driveOdom;
        frc::RamseteController controller;

        // Control states for the DT
        ControlState driveState;
        ShifterState shiftState;

        // last update time for safety critical topics
        double lastTwistTime, lastStickTime;

        // Demand variables
        double leftDemand, rightDemand;
        frc::DoubleSolenoid::Value shifterDemand;

        // Offset vars
        geometry_msgs::msg::Quaternion offsetQuaternion;

        //gyro bits
        double roll = 0;
        double pitch = 0;
        double yaw =  0;

    };

} // namespace robot
