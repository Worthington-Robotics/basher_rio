#include "SubsystemManager.h"
#include "units/time.h"
#include <exception>
#include <frc/DriverStation.h>
#include "rclcpp/rclcpp.hpp"

namespace robot
{

    SubsystemManager::SubsystemManager() : Node("roborio"),
                                           subsystems(),
                                           notif(std::bind(&SubsystemManager::onLoop, this))
    {
    }

    void SubsystemManager::registerSubsystems(std::vector<std::shared_ptr<Subsystem>> subsystems)
    {
        // Add all subsystems in new list to the master list
        this->subsystems.insert(this->subsystems.end(), subsystems.begin(), subsystems.end());

        // Create the ros bindings for each subsystem and reset its state
        for (std::shared_ptr<Subsystem> subsystem : subsystems)
        {
            subsystem->createRosBindings(this);
            subsystem->reset();
        }
    }

    void SubsystemManager::reset()
    {
        for (std::shared_ptr<Subsystem> subsystem : subsystems)
        {
            subsystem->reset();
        }
    }

    void SubsystemManager::startLoop()
    {
        isFirstIteration = true;
        notif.StartPeriodic(10_ms);
    }

    void SubsystemManager::stopLoop()
    {
        notif.Stop();
    }

    void SubsystemManager::onLoop()
    {
        try
        {
            // For the first iteration, run onstart
            if (isFirstIteration)
            {
                //frc::DriverStation::ReportWarning("Running first iteration");
                for (std::shared_ptr<Subsystem> subsystem : subsystems)
                {
                    subsystem->onStart();
                    subsystem->publishData();
                }
                isFirstIteration = false;
            }
            // for all others run onloop
            else
            {
                //frc::DriverStation::ReportWarning("Running onloop iteration");
                for (std::shared_ptr<Subsystem> subsystem : subsystems)
                {
                    subsystem->onLoop();
                    subsystem->publishData();
                }
            }

            rclcpp::spin_some(this->shared_from_this());
        } catch(const std::exception & e){
            frc::DriverStation::ReportError(e.what());
        } catch ( ... ){
            frc::DriverStation::ReportError("Looper Thread died with unknown exception");
        }
    }

} // namespace robot
