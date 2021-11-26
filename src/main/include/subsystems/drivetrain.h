#pragma once

#include "subsystems/Subsystem.h"
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <frc/controller/RamseteController.h>

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

    enum ShiftingMode {
        OPEN_LOOP,
        VELOCITY_THRESH,
        
    }

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

    private:
        std::shared_ptr<TalonFX> leftMaster, rightMaster, leftFollower, rightFollower;
        std::shared_ptr<frc::DoubleSolenoid> shifter;
        frc::RamseteController controller;
    };

} // namespace robot
