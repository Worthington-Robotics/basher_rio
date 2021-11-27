#pragma once

#include <vector>

#include <frc/Notifier.h>

#include "rclcpp/rclcpp.hpp"
#include "subsystems/Subsystem.h"

namespace robot
{

    class SubsystemManager : public rclcpp::Node
    {
        public: 
        SubsystemManager();

        /**
         * Registers a list of subsystems into the manager for ticking and handling publishing data.
         * They should already have been constructed, but not yet registered. Double registration may cause memory leaks.
         **/ 
        void registerSubsystems(std::vector<std::shared_ptr<Subsystem>> subsystems);

        /**
         * Commands all susbsystems to undergo a reset.
         **/ 
        void reset();

        /**
         * starts the subsystem manager thread, and begins updating the subsystems in order. 
         * Each subsystem is attempted to be updated at a fixed rate of 100 hz
         **/ 
        void startLoop();

        /**
         * Stops the susbsystem manager thread. The manager enters a stopped state.
         **/ 
        void stopLoop();

    private:
        bool isFirstIteration = false;
        std::vector<std::shared_ptr<Subsystem>> subsystems;
        frc::Notifier notif;

        void onLoop();
    };

} // namespace robot
