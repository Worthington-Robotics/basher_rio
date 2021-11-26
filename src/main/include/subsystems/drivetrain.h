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
    enum ControlState {
        OPEN_LOOP_TWIST,
        OPEN_LOOP_DIFF,
        VELOCITY,
        RAMSETE,
        PURSUIT
    };

    enum ShifterState {
        OPEN_LOOP,
        VELOCITY_THRESH,
        FIXED_HIGH,
        FIXED_LOW  
    };

    class Drivetrain : public Subsystem
    {

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


        void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

        void twistCallback(const geometry_msgs::msg::Twist msg);

        /**
         * Callback for setting transmission mode between the 4 availiable modes.
         * See ShifterState for availiable modes.
         **/ 
        void gearCallback(const std_msgs::msg::Int16 msg);

        /**
         * Callback for setting drivetrain modes. Selects between the control modes
         * enumerated in
         *  the ControlState enum.
         **/ 
        void controlCallback(const std_msgs::msg::Int16 msg);



    private:
        //IO devices
        std::shared_ptr<TalonFX> leftMaster, rightMaster, leftFollower, rightFollower;
        std::shared_ptr<frc::DoubleSolenoid> shifter;
        std::shared_ptr<PigeonIMU> imu;

        // ROS Publishers
        rclcpp::Publisher<sensor_msgs::msg::Imu> imuPub;
        rclcpp::Publisher<sensor_msgs::msg::JointState> leftWheel, rightWheel;

        // ROS Messages for publishing
        sensor_msgs::msg::Imu imuMsg;
        sensor_msgs::msg::JointState leftWheelState, rightWheelState; 

        // ROS Subscibers
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory> trajectorySub;
        rclcpp::Subscription<geometry_msgs::msg::Twist> twistSub;

        // mode control subscribers
        rclcpp::Subscription<std_msgs::msg::Int16> transModeSub, DriveModeSub;

        // underlying controllers
        frc::RamseteController controller;

        // Control states for the DT
        ControlState driveState;
        ShifterState shiftState;
    };

} // namespace robot
