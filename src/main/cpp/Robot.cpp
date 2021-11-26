// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>

void Robot::RobotInit()
{
  rclcpp::init(0, NULL);

  frc::DriverStation::GetInstance().ReportWarning("ROS Sucessfully Init!");

  // intialize all subsystems here
  manager = std::make_shared<robot::SubsystemManager>();
  manager->registerSubsystems(std::vector<std::shared_ptr<robot::Subsystem>>{

  });

  frc::DriverStation::GetInstance().ReportWarning("Robot Code Initialized");
}

void Robot::RobotPeriodic()
{
  //std::cout << "spinning" << std::endl;
  rclcpp::spin_some(node);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  manager->startLoop();
}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
  manager->stopLoop();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
