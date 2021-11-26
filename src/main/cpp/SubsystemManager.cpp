#include "SubsystemManager.h"
#include "units/time.h"

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
        // For the first iteration, run onstart
        if (isFirstIteration)
        {
            for (std::shared_ptr<Subsystem> subsystem : subsystems)
            {
                subsystem->onStart();
            }
            isFirstIteration = false;
        }
        // for all others run onloop
        else
        {
            for (std::shared_ptr<Subsystem> subsystem : subsystems)
            {
                subsystem->onLoop();
            }
        }
    }

} // namespace robot
