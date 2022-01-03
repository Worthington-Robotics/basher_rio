#include "subsystems/drivetrain.h"
#include "Constants.h"
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/userinput.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

using std::placeholders::_1;

namespace robot
{

    Drivetrain::Drivetrain()
    {
        frontRMod = std::make_shared<SModule>(DRIVE_FR_DRIVE, DRIVE_FR_ANGLE, DRIVE_FR_ENCOD, FR_ABS_OFFSET,
                                              PIDF{DRIVE_LEFT_KP, DRIVE_LEFT_KI, DRIVE_LEFT_KD, DRIVE_LEFT_KF}, PIDF{DRIVE_LEFT_KP, DRIVE_LEFT_KI, DRIVE_LEFT_KD, DRIVE_LEFT_KF});
        frontRMod->setInvertDrive(true);

        frontLMod = std::make_shared<SModule>(DRIVE_FL_DRIVE, DRIVE_FL_ANGLE, DRIVE_FL_ENCOD, FL_ABS_OFFSET,
                                              PIDF{DRIVE_LEFT_KP, DRIVE_LEFT_KI, DRIVE_LEFT_KD, DRIVE_LEFT_KF}, PIDF{DRIVE_LEFT_KP, DRIVE_LEFT_KI, DRIVE_LEFT_KD, DRIVE_LEFT_KF});
        frontLMod->setInvertDrive(true);

        rearRMod = std::make_shared<SModule>(DRIVE_RR_DRIVE, DRIVE_RR_ANGLE, DRIVE_RR_ENCOD, RR_ABS_OFFSET,
                                             PIDF{DRIVE_LEFT_KP, DRIVE_LEFT_KI, DRIVE_LEFT_KD, DRIVE_LEFT_KF}, PIDF{DRIVE_LEFT_KP, DRIVE_LEFT_KI, DRIVE_LEFT_KD, DRIVE_LEFT_KF});
        rearRMod->setInvertDrive(true);

        rearLMod = std::make_shared<SModule>(DRIVE_RL_DRIVE, DRIVE_RL_ANGLE, DRIVE_RL_ENCOD, RL_ABS_OFFSET,
                                             PIDF{DRIVE_LEFT_KP, DRIVE_LEFT_KI, DRIVE_LEFT_KD, DRIVE_LEFT_KF}, PIDF{DRIVE_LEFT_KP, DRIVE_LEFT_KI, DRIVE_LEFT_KD, DRIVE_LEFT_KF});
        rearLMod->setInvertDrive(true);

        imu = std::make_shared<PigeonIMU>(IMU_ID);

        reset();
    }

    void Drivetrain::createRosBindings(rclcpp::Node *node)
    {
        // Create sensor data publishers
        imuPub = node->create_publisher<sensor_msgs::msg::Imu>("/drive/imu", rclcpp::SensorDataQoS());
        wheelStatePub = node->create_publisher<sensor_msgs::msg::JointState>("/drive/wheel_state", rclcpp::SensorDataQoS());

        // Create subscribers
        trajectorySub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>("/drive/active_traj", rclcpp::SystemDefaultsQoS(), std::bind(&Drivetrain::trajectoryCallback, this, _1));
        twistSub = node->create_subscription<geometry_msgs::msg::Twist>("/drive/velocity_twist", rclcpp::SensorDataQoS(), std::bind(&Drivetrain::twistCallback, this, _1));
        stickSub = node->create_subscription<sensor_msgs::msg::Joy>(DRIVE_STICK_TOPIC, rclcpp::SensorDataQoS(), std::bind(&Drivetrain::stickCallback, this, _1));

        DriveModeSub = node->create_subscription<std_msgs::msg::Int16>("/drive/drive_mode", rclcpp::SensorDataQoS(), std::bind(&Drivetrain::driveModeCallback, this, _1));
    }

    void Drivetrain::reset()
    {
        // reset cached data to prevent nullptrs
        wheelState = sensor_msgs::msg::JointState();
        wheelState.name = {"left", "right"};
        wheelState.position = {0, 0};
        wheelState.velocity = {0, 0};
        wheelState.effort = {0, 0};

        // TODO reset sensors

        // Reset the IMU message and wait for data
        imuMsg = sensor_msgs::msg::Imu();
        //set covariances
        imuMsg.orientation_covariance = IMU_ORIENT_COVAR;
        imuMsg.linear_acceleration_covariance = IMU_ACCEL_COVAR;
        imuMsg.angular_velocity_covariance = IMU_ANG_VEL_COVAR;

        // default values to zero
        imu->SetFusedHeading(0);

        imuMsg.linear_acceleration.x = 0;
        imuMsg.linear_acceleration.y = 0;
        imuMsg.linear_acceleration.z = 0;
        imuMsg.orientation.x = 0;
        imuMsg.orientation.y = 0;
        imuMsg.orientation.z = 0;
        imuMsg.orientation.w = 1;
        imuMsg.angular_velocity.x = 0;
        imuMsg.angular_velocity.y = 0;
        imuMsg.angular_velocity.z = 0;

        // clear demands and reset time
        leftDemand = rightDemand = 0;
        lastTwistTime = 0;

        driveState = OPEN_LOOP_FIELD_REL;

        frontRMod->reset();
        frontLMod->reset();
        rearRMod->reset();
        rearLMod->reset();
    }

    void Drivetrain::onStart()
    {
    }

    void Drivetrain::updateSensorData()
    {
        //frc::DriverStation::ReportWarning("Updating drive sensor data");
        // read the current IMU state
        int16_t accelData[3];
        imu->GetBiasedAccelerometer(accelData);
        // Convert from 2^14 = 1g = 9.8 m/s^2
        imuMsg.linear_acceleration.x = accelData[0] * .000598784;
        imuMsg.linear_acceleration.y = accelData[1] * .000598784;
        imuMsg.linear_acceleration.z = accelData[2] * .000598784;

        double gyroData[3];
        imu->GetRawGyro(gyroData);
        // Convert from deg/s to rad/s
        imuMsg.angular_velocity.x = gyroData[0] * 0.01745329;
        imuMsg.angular_velocity.y = gyroData[1] * 0.01745329;
        imuMsg.angular_velocity.z = gyroData[2] * 0.01745329;

        double orientData[4];
        imu->Get6dQuaternion(orientData);
        imuMsg.orientation.w = orientData[0];
        imuMsg.orientation.x = orientData[1];
        imuMsg.orientation.y = orientData[2];
        imuMsg.orientation.z = orientData[3];

        yaw.data = -imu->GetFusedHeading();

       
    }

    // Average the wheel state velocities
    double getFwdVelocity(sensor_msgs::msg::JointState wheelState)
    {
        return (wheelState.velocity.at(0) + wheelState.velocity.at(0)) / 2.0;
    }

    frc::ChassisSpeeds Drivetrain::twistDrive(const geometry_msgs::msg::Twist &twist, const frc::Rotation2d &orientation)
    {
        double xSpeed = twist.linear.x;
        double ySpeed = twist.linear.y;
        double zTurn = twist.angular.z;
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            units::meters_per_second_t{xSpeed}, 
            units::meters_per_second_t{ySpeed}, 
            units::radians_per_second_t{zTurn}, 
            orientation);
        return speeds;
    }

    frc::ChassisSpeeds Drivetrain::twistDrive(const geometry_msgs::msg::Twist &twist)
    {
        double xSpeed = twist.linear.x;
        double ySpeed = twist.linear.y;
        double zTurn = twist.angular.z;
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds{
            units::meters_per_second_t{xSpeed},
            units::meters_per_second_t{ySpeed},
            units::radians_per_second_t{zTurn}};
        return speeds;
    }

    void Drivetrain::onLoop()
    {
        // Read sensors
        updateSensorData();

        frc::ChassisSpeeds speed;
        switch (driveState)
        {
        case OPEN_LOOP_FIELD_REL:
        {
            // if we are safe, set motor demands,
            if (lastStickTime + DRIVE_TIMEOUT > frc::Timer::GetFPGATimestamp())
            {
                // parse the joy message
                std::vector<double> joyData = UserInput::scalarCut(lastStick, DRIVE_STICK_DEADBAND,
                                                                   DRIVE_STICK_POWER, DRIVE_STICK_SCALAR);

                auto stickTwist = geometry_msgs::msg::Twist();
                stickTwist.linear.x = joyData.at(1);
                stickTwist.linear.y = joyData.at(0);
                stickTwist.angular.z = joyData.at(2);
                // convert to demands
                speed = twistDrive(stickTwist, frc::Rotation2d{units::degree_t{yaw.data}});
            }
            else
            { // otherwise force motors to zero, there is stale data
                speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
            }
            break;
        }
        case OPEN_LOOP_ROBOT_REL:
        {
            // if we are safe, set motor demands,
            if (lastStickTime + DRIVE_TIMEOUT > frc::Timer::GetFPGATimestamp())
            {
                // parse the joy message
                std::vector<double> joyData = UserInput::scalarCut(lastStick, DRIVE_STICK_DEADBAND,
                                                                   DRIVE_STICK_POWER, DRIVE_STICK_SCALAR);

                auto stickTwist = geometry_msgs::msg::Twist();
                stickTwist.linear.x = joyData.at(1);
                stickTwist.linear.y = joyData.at(0);
                stickTwist.angular.z = joyData.at(2);
                // convert to demands
                speed = twistDrive(stickTwist);
            }
            else
            { // otherwise force motors to zero, there is stale data
                speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
            }
            break;
        }
        case VELOCITY_TWIST:
            // if we are safe, set motor demands,
            if (lastTwistTime + DRIVE_TIMEOUT > frc::Timer::GetFPGATimestamp())
            {
                speed = twistDrive(lastTwist);
            }
            else
            { // otherwise force motors to zero, there is stale data
                speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
            }

            break;
        case PURSUIT: // for now have pursuit as an illegal mode
            frc::DriverStation::ReportWarning("Mode is not currently implemented");
            speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
            break;

        default:
            frc::DriverStation::ReportError("Drivetrain attempted to enter an illegal mode");
            speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
        }

        moduleStates = sKinematics.ToSwerveModuleStates(speed);
        sOdom.Update(frc::Rotation2d{units::degree_t{-yaw.data}}, frontRMod->getData(), frontLMod->getData(), rearRMod->getData(), rearLMod->getData());
        frontLMod->setMotors(moduleStates[0]);
        frontRMod->setMotors(moduleStates[1]);
        rearLMod->setMotors(moduleStates[2]);
        rearRMod->setMotors(moduleStates[3]);
    }

    void Drivetrain::publishData()
    {
        imuPub->publish(imuMsg);
        wheelStatePub->publish(wheelState);

        if(DEBUG){
            frc::SmartDashboard::PutNumber("Drive/Front/Left/AngleABS", frontLMod->getData().encAbs);
            frc::SmartDashboard::PutNumber("Drive/Front/Right/AngleABS", frontRMod->getData().encAbs);
            frc::SmartDashboard::PutNumber("Drive/Rear/Left/AngleABS", rearLMod->getData().encAbs);
            frc::SmartDashboard::PutNumber("Drive/Rear/Right/AngleABS", rearRMod->getData().encAbs);

            frc::SmartDashboard::PutNumber("Drive/Front/Left/Desired", moduleStates[0].angle.Degrees().to<double>());
            frc::SmartDashboard::PutNumber("Drive/Front/Right/Desired", moduleStates[1].angle.Degrees().to<double>());
            frc::SmartDashboard::PutNumber("Drive/Rear/Left/Desired", moduleStates[2].angle.Degrees().to<double>());
            frc::SmartDashboard::PutNumber("Drive/Rear/Right/Desired", moduleStates[3].angle.Degrees().to<double>());

            frc::SmartDashboard::PutNumber("Drive/Front/Left/UncalABS", std::fmod(frontLMod->getData().encAbs + FR_ABS_OFFSET, 360.0));
            frc::SmartDashboard::PutNumber("Drive/Front/Right/UncalABS", std::fmod(frontRMod->getData().encAbs + FL_ABS_OFFSET, 360.0));
            frc::SmartDashboard::PutNumber("Drive/Rear/Left/UncalABS", std::fmod(rearLMod->getData().encAbs + RL_ABS_OFFSET, 360.0));
            frc::SmartDashboard::PutNumber("Drive/Rear/Right/UncalABS", std::fmod(rearRMod->getData().encAbs + RR_ABS_OFFSET, 360.0));
        }

        frc::SmartDashboard::PutNumber("Drive/Front/Left/AngleRel", frontLMod->getData().angleRel);
        frc::SmartDashboard::PutNumber("Drive/Front/Right/AngleRel", frontRMod->getData().angleRel);
        frc::SmartDashboard::PutNumber("Drive/Rear/Left/AngleRel", rearRMod->getData().angleRel);
        frc::SmartDashboard::PutNumber("Drive/Rear/Right/AngleRel", rearLMod->getData().angleRel);

        frc::SmartDashboard::PutNumber("Drive/Pose/X", sOdom.GetPose().X().to<double>());
        frc::SmartDashboard::PutNumber("Drive/Pose/Y", sOdom.GetPose().Y().to<double>());
        frc::SmartDashboard::PutNumber("Drive/Pose/Theta1", sOdom.GetPose().Rotation().Degrees().to<double>());
        frc::SmartDashboard::PutNumber("Drive/Pose/Theta2", yaw.data);
    }

    void Drivetrain::trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
    }

    /**
     * Updates the twist lockout timer, and the latest twist information.
     * the units on the twist indicies are mode specific.
     **/
    void Drivetrain::twistCallback(const geometry_msgs::msg::Twist msg)
    {
        lastTwistTime = frc::Timer::GetFPGATimestamp();
        lastTwist = msg;
    }

    void Drivetrain::stickCallback(const sensor_msgs::msg::Joy msg)
    {
        lastStickTime = frc::Timer::GetFPGATimestamp();
        lastStick = msg;
    }

    void Drivetrain::driveModeCallback(const std_msgs::msg::Int16 msg)
    {
        std::cout << "changing drivetrain to mode " << msg.data << std::endl;
        driveState = static_cast<ControlState>(msg.data);
    }

    void Drivetrain::enableDebug(bool debugEnable){
        frc::DriverStation::ReportWarning("Driverstation setting debugEnabled");
        DEBUG = debugEnable;
    }

} // namespace robot
