// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <hal/DriverStation.h>
#include <networktables/NetworkTable.h>

void leftCallback(const void * leftFloat) {
  leftMotor->Set(ControlMode::PercentOutput, ((std_msgs__msg__Float32*) leftFloat)->data);
}

void rightCallback(const void * rightFloat) {
  rightMotor->Set(ControlMode::PercentOutput, ((std_msgs__msg__Float32*) rightFloat)->data);
}

void Robot::RobotInit() {
  std::string hostIP = "10.41.45.84";
  std::string hostPort = "8888";

  helper = std::make_shared<ros::RosHelper>(hostIP, hostPort);
  leftMotor = new TalonSRX(1);
  rightMotor = new TalonSRX(2);

  frc::DriverStation::GetInstance().ReportWarning("Attempting connection to micro_ros agent...");
  if (helper->initAgentConnect()) {
    frc::DriverStation::GetInstance().ReportWarning("Connection established, starting ROS");

    std::string leftTopic = "/left";
    std::string rightTopic = "/right";
    std::string twistTopic = "/drive/twist";

    helper->registerSubscriber(leftSub, leftTopic, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), &leftCallback, left);
    helper->registerSubscriber(rightSub, rightTopic, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), &rightCallback, right);

    helper->registerPublisher(twistPub, twistTopic, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist));

    frc::DriverStation::GetInstance().ReportWarning("Code init complete");
    startOK = true;
  }
}

void Robot::Disabled() {}

void Robot::Autonomous() {


}

void Robot::Teleop() {
  velocity.linear.x = frc::DriverStation::GetInstance().GetStickAxis(0, 0);
  velocity.linear.y = frc::DriverStation::GetInstance().GetStickAxis(0, 1);
  velocity.linear.z = frc::DriverStation::GetInstance().GetStickAxis(0, 2);
}

void Robot::Test() {}

void Robot::StartCompetition() {
  auto& lw = *frc::LiveWindow::GetInstance();

  RobotInit();

  // Tell the DS that the robot is ready to be enabled
  HAL_ObserveUserProgramStarting();

  while (!m_exit) {
    if (IsDisabled()) {
      m_ds.InDisabled(true);
      Disabled();
      m_ds.InDisabled(false);
      while (IsDisabled()) {
        m_ds.WaitForData();
      }
    } else if (IsAutonomous()) {
      m_ds.InAutonomous(true);
      Autonomous();
      m_ds.InAutonomous(false);
      while (IsAutonomousEnabled()) {
        m_ds.WaitForData();
      }
    } else if (IsTest()) {
      lw.SetEnabled(true);
      frc::Shuffleboard::EnableActuatorWidgets();
      m_ds.InTest(true);
      Test();
      m_ds.InTest(false);
      while (IsTest() && IsEnabled()) {
        m_ds.WaitForData();
      }
      lw.SetEnabled(false);
      frc::Shuffleboard::DisableActuatorWidgets();
    } else {
      m_ds.InOperatorControl(true);
      Teleop();
      m_ds.InOperatorControl(false);
      while (IsOperatorControlEnabled()) {
        m_ds.WaitForData();
      }
    }
  }
}

void Robot::EndCompetition() {
  m_exit = true;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
