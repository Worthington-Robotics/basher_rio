// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include "subsystems/drivetrain.h"
#include "subsystems/userinput.h"
#include "Constants.h"
#include <frc/Filesystem.h>

void Robot::RobotInit()
{
    rclcpp::init(0, NULL);

    frc::DriverStation::GetInstance().ReportWarning("ROS Sucessfully Init!");

    // construct subsystems
    auto drive = std::make_shared<robot::Drivetrain>();
    auto sticks = std::make_shared<robot::UserInput>();
    sticks->registerSticks(USER_STICKS); //  register which joystick IDs to read

    // intialize all subsystems here
    manager = std::make_shared<robot::SubsystemManager>();
    manager->registerSubsystems(std::vector<std::shared_ptr<robot::Subsystem>>{
        drive,
        sticks});

    // grab the version string
    int versionNum = 1337;
    // wpi::SmallVectorImpl<char> filename = wpi::SmallVectorImpl<char>(0);
    // frc::filesystem::GetDeployDirectory(filename);
    // std::cout << filename.data() << std::endl;
    // while(filename.size() > 0){
    //     std::cout << filename.pop_back_val() << std::endl;
    // }
    // filename += "/version.dat";
    // FILE* version = std::fopen( filename., "r");
    // std::fscanf(version, "%*s%i", &versionNum);
    // std::fclose(version);
    frc::DriverStation::GetInstance().ReportWarning(std::string("Robot Code Initialized with version ") + std::to_string(versionNum));
}

void Robot::RobotPeriodic()
{
    //std::cout << "spinning" << std::endl;
}

void Robot::AutonomousInit()
{
    manager->stopDisabledLoop();
    manager->startEnabledLoop();
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    manager->stopDisabledLoop();
    manager->startEnabledLoop();
}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit()
{
    manager->stopEnabledLoop();
    manager->startDisabledLoop();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit()
{
    manager->stopDisabledLoop();
    manager->startEnabledLoop();
}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
