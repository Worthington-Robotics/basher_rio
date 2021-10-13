// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>

#include <frc/RobotBase.h>

#include "ROSHelper.h"
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <ctre/Phoenix.h>

struct custom_args
{
  int a;
};

TalonSRX* leftMotor;
TalonSRX* rightMotor;

class Robot : public frc::RobotBase {
 public:
  void RobotInit();
  void Disabled();
  void Autonomous();
  void Teleop();
  void Test();

  void StartCompetition() override;
  void EndCompetition() override;

  // void leftCallback(const void *);
  // void rightCallback(const void *);

protected:
  bool startOK;

  std::shared_ptr<ros::RosHelper> helper;

  rcl_publisher_t *twistPub;
  rcl_subscription_t *leftSub;
  rcl_subscription_t *rightSub;


  std_msgs__msg__Float32 *left;
  std_msgs__msg__Float32 *right;
  geometry_msgs__msg__Twist velocity;


  custom_args args;

 private:
  std::atomic<bool> m_exit{false};
};
