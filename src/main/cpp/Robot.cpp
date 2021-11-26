// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>

void Robot::RobotInit()
{
  rclcpp::init(0, NULL);

  frc::DriverStation::GetInstance().ReportWarning("ROS Sucessfully Init!");

  node = std::make_shared<ros::RosNode>();

  leftMaster = std::make_shared<TalonFX>(1);
  rightMaster = std::make_shared<TalonFX>(2);

  leftFollower = std::make_shared<TalonFX>(3);
  rightFollower = std::make_shared<TalonFX>(4);

  rightFollower->Follow(*rightMaster);
  leftFollower->Follow(*leftMaster);

  shifter = std::make_shared<frc::DoubleSolenoid>(1,2);
  aux = std::make_shared<frc::Solenoid>(3);

  frc::DriverStation::GetInstance().ReportWarning("Robot Code Initialized");
}

void Robot::RobotPeriodic()
{
  //std::cout << "spinning" << std::endl;
  node->publish();
  rclcpp::spin_some(node);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
  double leftData, rightData;
  node->getNewData(leftData, rightData);
  leftMaster->Set(ControlMode::PercentOutput, leftData);
  rightMaster->Set(ControlMode::PercentOutput, rightData);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
