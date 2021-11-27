#pragma once

#include "subsystems/Subsystem.h"
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <frc/controller/RamseteController.h>
#include <rclcpp/rclcpp.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16.hpp>

namespace robot
{

    /**
     * Possible control states for the drivetrain to be in
     **/
    enum ControlState
    {
        OPEN_LOOP_TWIST,
        VELOCITY,
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
        FIXED_LOW
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
         * Calbback for streaming the current desired twist of the drivetrain
         **/
        void twistCallback(const geometry_msgs::msg::Twist msg);

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

        // ROS Messages for publishing
        sensor_msgs::msg::Imu imuMsg;
        sensor_msgs::msg::JointState wheelState;

        // ROS Subscibers
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectorySub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr transModeSub, DriveModeSub;

        // ROS Messages for storing subscription data
        geometry_msgs::msg::Twist lastTwist;

        // underlying controllers
        frc::RamseteController controller;

        // Control states for the DT
        ControlState driveState;
        ShifterState shiftState;

        double lastTwistTime;

        // Demand variables
        double leftDemand, rightDemand;
        frc::DoubleSolenoid::Value shifterDemand;
    };

} // namespace robot
