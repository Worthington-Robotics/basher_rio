#pragma once
#include "subsystems/Subsystem.h"
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <vector>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robot
{

    class UserInput : public Subsystem
    {
    public:
        UserInput();

        void registerSticks(std::vector<int> stickIds);

        /**
         * Override this function in order to create pulbishers or subscribers against the parent node.
         * NOTE: This function is automatically called by the subsystem manager on registration
         **/
        void createRosBindings(rclcpp::Node *node) override;

        /**
         * Override this function with all the nessecary code needed to reset a subsystem.
         * For sticks, nothing needs to be reset (no accumulators)
         **/
        void reset() override;

        /**
         * Overrride this function with any code needed to be called only once on the first onloop iteration
         **/
        void onStart() override;

        /**
         * Override this function for any code that must be called periodically by the subsystem
         * For sticks, nothing is called during onloop
         **/
        void onLoop() override;

        /**
         * Override this function with code needed to publish all data out to the ros network
         **/
        void publishData() override;

    private:
        std::vector<frc::Joystick> sticks = {};
        std::vector<rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr> stickPubs = {};
    };

} // namespace robot
