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
        frontLeftAngle = std::make_shared<TalonFX>(DRIVE_FL_ANGLE);
        frontLeftDrive = std::make_shared<TalonFX>(DRIVE_FL_DRIVE);
        frontLeftEncod = std::make_shared<CANCoder>(DRIVE_FL_ENCOD);

        frontRightAngle = std::make_shared<TalonFX>(DRIVE_FR_ANGLE);
        frontRightDrive = std::make_shared<TalonFX>(DRIVE_FR_DRIVE);
        frontRightEncod = std::make_shared<CANCoder>(DRIVE_FR_ENCOD);

        rearLeftAngle = std::make_shared<TalonFX>(DRIVE_RL_ANGLE);
        rearLeftDrive = std::make_shared<TalonFX>(DRIVE_RL_DRIVE);
        rearLeftEncod = std::make_shared<CANCoder>(DRIVE_RL_ENCOD);

        rearRightAngle = std::make_shared<TalonFX>(DRIVE_RR_ANGLE);
        rearRightDrive = std::make_shared<TalonFX>(DRIVE_RR_DRIVE);
        rearRightEncod = std::make_shared<CANCoder>(DRIVE_RR_ENCOD);

        imu = std::make_shared<PigeonIMU>(IMU_ID);

        configMotors();

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

    void Drivetrain::configMotors()
    {
        // Configure front left drive falcon
        frontLeftDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
        frontLeftDrive->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 5, 0);
        frontLeftDrive->SetSensorPhase(true);
        frontLeftDrive->SetInverted(true);
        frontLeftDrive->SetNeutralMode(NeutralMode::Brake);
        frontLeftDrive->SelectProfileSlot(0, 0);
        frontLeftDrive->Config_kF(0, DRIVE_LEFT_KF, 0);
        frontLeftDrive->Config_kP(0, DRIVE_LEFT_KP, 0);
        frontLeftDrive->Config_kI(0, DRIVE_LEFT_KI, 0);
        frontLeftDrive->Config_kD(0, DRIVE_LEFT_KD, 0);
        frontLeftDrive->Config_IntegralZone(0, DRIVE_LEFT_IACCUM, 0);
        frontLeftDrive->ConfigVoltageCompSaturation(DRIVE_VCOMP_VOLTAGE, 0);
        frontLeftDrive->EnableVoltageCompensation(true);
        frontLeftDrive->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));

        // Configure front right drive falcon
        frontRightDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
        frontRightDrive->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 5, 0);
        frontRightDrive->SetSensorPhase(true);
        frontRightDrive->SetInverted(true);
        frontRightDrive->SetNeutralMode(NeutralMode::Brake);
        frontRightDrive->SelectProfileSlot(0, 0);
        frontRightDrive->Config_kF(0, DRIVE_LEFT_KF, 0);
        frontRightDrive->Config_kP(0, DRIVE_LEFT_KP, 0);
        frontRightDrive->Config_kI(0, DRIVE_LEFT_KI, 0);
        frontRightDrive->Config_kD(0, DRIVE_LEFT_KD, 0);
        frontRightDrive->Config_IntegralZone(0, DRIVE_LEFT_IACCUM, 0);
        frontRightDrive->ConfigVoltageCompSaturation(DRIVE_VCOMP_VOLTAGE, 0);
        frontRightDrive->EnableVoltageCompensation(true);
        frontRightDrive->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));

        // Configure rear left drive falcon
        rearLeftDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
        rearLeftDrive->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 5, 0);
        rearLeftDrive->SetSensorPhase(true);
        rearLeftDrive->SetInverted(true);
        rearLeftDrive->SetNeutralMode(NeutralMode::Brake);
        rearLeftDrive->SelectProfileSlot(0, 0);
        rearLeftDrive->Config_kF(0, DRIVE_LEFT_KF, 0);
        rearLeftDrive->Config_kP(0, DRIVE_LEFT_KP, 0);
        rearLeftDrive->Config_kI(0, DRIVE_LEFT_KI, 0);
        rearLeftDrive->Config_kD(0, DRIVE_LEFT_KD, 0);
        rearLeftDrive->Config_IntegralZone(0, DRIVE_LEFT_IACCUM, 0);
        rearLeftDrive->ConfigVoltageCompSaturation(DRIVE_VCOMP_VOLTAGE, 0);
        rearLeftDrive->EnableVoltageCompensation(true);
        rearLeftDrive->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));

        // Configure rear right drive falcon
        rearRightDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
        rearRightDrive->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 5, 0);
        rearRightDrive->SetSensorPhase(true);
        rearRightDrive->SetInverted(true);
        rearRightDrive->SetNeutralMode(NeutralMode::Brake);
        rearRightDrive->SelectProfileSlot(0, 0);
        rearRightDrive->Config_kF(0, DRIVE_LEFT_KF, 0);
        rearRightDrive->Config_kP(0, DRIVE_LEFT_KP, 0);
        rearRightDrive->Config_kI(0, DRIVE_LEFT_KI, 0);
        rearRightDrive->Config_kD(0, DRIVE_LEFT_KD, 0);
        rearRightDrive->Config_IntegralZone(0, DRIVE_LEFT_IACCUM, 0);
        rearRightDrive->ConfigVoltageCompSaturation(DRIVE_VCOMP_VOLTAGE, 0);
        rearRightDrive->EnableVoltageCompensation(true);
        rearRightDrive->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));

        // Configure front left angle falcon
        frontLeftEncod->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 5, 0);
        frontLeftEncod->ConfigSensorDirection(true, 0);

        frontLeftAngle->ConfigRemoteFeedbackFilter(DRIVE_FL_ENCOD, RemoteSensorSource::RemoteSensorSource_CANCoder, 0, 0);
        frontLeftAngle->ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0);
        frontLeftAngle->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 0);
        frontLeftAngle->SetSensorPhase(true);
        frontLeftAngle->SetInverted(true);
        frontLeftAngle->SetNeutralMode(NeutralMode::Brake);
        frontLeftAngle->SelectProfileSlot(0, 0);
        frontLeftAngle->Config_kF(0, DRIVE_LEFT_KF, 0);
        frontLeftAngle->Config_kP(0, DRIVE_LEFT_KP, 0);
        frontLeftAngle->Config_kI(0, DRIVE_LEFT_KI, 0);
        frontLeftAngle->Config_kD(0, DRIVE_LEFT_KD, 0);
        frontLeftAngle->Config_IntegralZone(0, DRIVE_LEFT_IACCUM, 0);
        frontLeftAngle->ConfigVoltageCompSaturation(DRIVE_VCOMP_VOLTAGE, 0);
        frontLeftAngle->EnableVoltageCompensation(true);
        frontLeftAngle->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));

        // Configure front right angle falcon
        frontRightEncod->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 5, 0);
        frontRightEncod->ConfigSensorDirection(true, 0);

        frontRightAngle->ConfigRemoteFeedbackFilter(DRIVE_FR_ENCOD, RemoteSensorSource::RemoteSensorSource_CANCoder, 0, 0);
        frontRightAngle->ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0);
        frontRightAngle->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 0);
        frontRightAngle->SetSensorPhase(true);
        frontRightAngle->SetInverted(true);
        frontRightAngle->SetNeutralMode(NeutralMode::Brake);
        frontRightAngle->SelectProfileSlot(0, 0);
        frontRightAngle->Config_kF(0, DRIVE_LEFT_KF, 0);
        frontRightAngle->Config_kP(0, DRIVE_LEFT_KP, 0);
        frontRightAngle->Config_kI(0, DRIVE_LEFT_KI, 0);
        frontRightAngle->Config_kD(0, DRIVE_LEFT_KD, 0);
        frontRightAngle->Config_IntegralZone(0, DRIVE_LEFT_IACCUM, 0);
        frontRightAngle->ConfigVoltageCompSaturation(DRIVE_VCOMP_VOLTAGE, 0);
        frontRightAngle->EnableVoltageCompensation(true);
        frontRightAngle->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));

        // Configure rear left angle falcon
        rearLeftEncod->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 5, 0);
        rearLeftEncod->ConfigSensorDirection(true, 0);

        rearLeftAngle->ConfigRemoteFeedbackFilter(DRIVE_RL_ENCOD, RemoteSensorSource::RemoteSensorSource_CANCoder, 0, 0);
        rearLeftAngle->ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0);
        rearLeftAngle->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 5, 0);
        rearLeftAngle->SetSensorPhase(true);
        rearLeftAngle->SetInverted(true);
        rearLeftAngle->SetNeutralMode(NeutralMode::Brake);
        rearLeftAngle->SelectProfileSlot(0, 0);
        rearLeftAngle->Config_kF(0, DRIVE_LEFT_KF, 0);
        rearLeftAngle->Config_kP(0, DRIVE_LEFT_KP, 0);
        rearLeftAngle->Config_kI(0, DRIVE_LEFT_KI, 0);
        rearLeftAngle->Config_kD(0, DRIVE_LEFT_KD, 0);
        rearLeftAngle->Config_IntegralZone(0, DRIVE_LEFT_IACCUM, 0);
        rearLeftAngle->ConfigVoltageCompSaturation(DRIVE_VCOMP_VOLTAGE, 0);
        rearLeftAngle->EnableVoltageCompensation(true);
        rearLeftAngle->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));

        // Configure rear right angle falcon
        rearRightEncod->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 5, 0);
        rearRightEncod->ConfigSensorDirection(true, 0);

        rearRightAngle->ConfigRemoteFeedbackFilter(DRIVE_RR_ENCOD, RemoteSensorSource::RemoteSensorSource_CANCoder, 0, 0);
        rearRightAngle->ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0);
        rearRightAngle->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 5, 0);
        rearRightAngle->SetSensorPhase(true);
        rearRightAngle->SetInverted(true);
        rearRightAngle->SetNeutralMode(NeutralMode::Brake);
        rearRightAngle->SelectProfileSlot(0, 0);
        rearRightAngle->Config_kF(0, DRIVE_LEFT_KF, 0);
        rearRightAngle->Config_kP(0, DRIVE_LEFT_KP, 0);
        rearRightAngle->Config_kI(0, DRIVE_LEFT_KI, 0);
        rearRightAngle->Config_kD(0, DRIVE_LEFT_KD, 0);
        rearRightAngle->Config_IntegralZone(0, DRIVE_LEFT_IACCUM, 0);
        rearRightAngle->ConfigVoltageCompSaturation(DRIVE_VCOMP_VOLTAGE, 0);
        rearRightAngle->EnableVoltageCompensation(true);
        rearRightAngle->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));
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

        // build the ramsete controller
        controller = frc::RamseteController();

        // clear demands and reset time
        leftDemand = rightDemand = 0;
        lastTwistTime = 0;

        driveState = OPEN_LOOP_STICK;
    }

    void Drivetrain::onStart()
    {
    }

    void Drivetrain::updateSensorData()
    {
        frc::SmartDashboard::PutNumber("Drive/Front/Left/Angle", frontLeftEncod->GetAbsolutePosition());
        frc::SmartDashboard::PutNumber("Drive/Front/Right/Angle", frontRightEncod->GetAbsolutePosition());
        frc::SmartDashboard::PutNumber("Drive/Rear/Left/Angle", rearLeftEncod->GetAbsolutePosition());
        frc::SmartDashboard::PutNumber("Drive/Rear/Right/Angle", rearRightEncod->GetAbsolutePosition());
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

        //Read the current left and right joint states 
        //TODO FIX
        // wheelState.position = {f->GetSelectedSensorPosition(), rightMaster->GetSelectedSensorPosition()};
        // wheelState.velocity = {leftMaster->GetSelectedSensorVelocity(), rightMaster->GetSelectedSensorVelocity()};
        // wheelState.effort = {leftMaster->GetStatorCurrent(), rightMaster->GetStatorCurrent()};
    }

    // Average the wheel state velocities
    double getFwdVelocity(sensor_msgs::msg::JointState wheelState)
    {
        return (wheelState.velocity.at(0) + wheelState.velocity.at(0)) / 2.0;
    }

    void twistToDemand(const geometry_msgs::msg::Twist twist, double &leftDemand, double &rightDemand)
    {
        leftDemand = twist.linear.x - DRIVE_TRACK_WIDTH / 2 * twist.angular.z;
        rightDemand = twist.linear.x + DRIVE_TRACK_WIDTH / 2 * twist.angular.z;
    }

    void arcadeDrive (const geometry_msgs::msg::Twist twist, double &leftDemand, double &rightDemand){
        const double maxInput = std::max(std::max(std::abs(twist.linear.x - twist.angular.z), std::abs(twist.linear.x + twist.angular.z)), 1.0);

        rightDemand = (twist.linear.x + twist.angular.z) / maxInput;
        leftDemand = (twist.linear.x - twist.angular.z) / maxInput;
    }

    frc::ChassisSpeeds Drivetrain::twistDrive (const geometry_msgs::msg::Twist twist){
        double xSpeed = twist.linear.x;
        double ySpeed = twist.linear.y;
        double zTurn = twist.angular.z;
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t{xSpeed}, units::meters_per_second_t{ySpeed}, units::radians_per_second_t{zTurn}, frc::Rotation2d{units::degree_t{imu->GetFusedHeading()}});
        return speeds;
    }

    void Drivetrain::onLoop()
    {
        // Read sensors
        updateSensorData();

        frc::ChassisSpeeds speed;
        switch (driveState)
        {
        case OPEN_LOOP_STICK:{
            // if we are safe, set motor demands,
            if (lastStickTime + DRIVE_TIMEOUT > frc::Timer::GetFPGATimestamp())
            {
                // parse the joy message
                std::vector<double> joyData = UserInput::scalarCut(lastStick, DRIVE_STICK_DEADBAND,
                                                                   DRIVE_STICK_POWER, DRIVE_STICK_SCALAR);

                std::cout << "joyData " << joyData.at(0) << " " << joyData.at(1) << std::endl;

                auto stickTwist = geometry_msgs::msg::Twist();
                stickTwist.linear.x = joyData.at(0);
                stickTwist.linear.y = joyData.at(1);
                stickTwist.angular.z = joyData.at(2);
                // convert to demands
                //arcadeDrive(stickTwist, leftDemand, rightDemand);
                speed = twistDrive(stickTwist);
            }
            else
            { // otherwise force motors to zero, there is stale data
                frc::DriverStation::ReportWarning("You've done fucked up");
                speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
            }   
            break;
        }
        case VELOCITY_TWIST:
            // if we are safe, set motor demands,
            if (lastTwistTime + DRIVE_TIMEOUT > frc::Timer::GetFPGATimestamp())
            {
                twistToDemand(lastTwist, leftDemand, rightDemand);
            }
            else
            { // otherwise force motors to zero, there is stale data
                leftDemand = rightDemand = 0;
            }
            speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
            break;

        case RAMSETE:
        case PURSUIT: // for now have pursuit as an illegal mode
            frc::DriverStation::ReportWarning("Mode is not currently implemented");
            speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
            break;

        default:
            frc::DriverStation::ReportError("Drivetrain attempted to enter an illegal mode");
            speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
        }
            auto [fr, fl, rr, rl] = sKinematics.ToSwerveModuleStates(speed);
            frontLeftAngle->Set(ControlMode::Position, (fl.angle.Degrees().to<double>() + FL_ABS_OFFSET + 360 % 360) * TICKS_PER_DEGREE);
            frontLeftDrive->Set(ControlMode::PercentOutput, fl.speed.to<double>());
            frontRightAngle->Set(ControlMode::Position, (fr.angle.Degrees().to<double>() + FR_ABS_OFFSET + 360 % 360) * TICKS_PER_DEGREE);
            frontRightDrive->Set(ControlMode::PercentOutput, fr.speed.to<double>());
            rearLeftAngle->Set(ControlMode::Position, (rl.angle.Degrees().to<double>() + RL_ABS_OFFSET + 360 % 360) * TICKS_PER_DEGREE);
            rearLeftDrive->Set(ControlMode::PercentOutput, rl.speed.to<double>());
            rearRightAngle->Set(ControlMode::Position, (rr.angle.Degrees().to<double>() + RR_ABS_OFFSET + 360 % 360) * TICKS_PER_DEGREE);
            rearRightDrive->Set(ControlMode::PercentOutput, rr.speed.to<double>());
            frc::SmartDashboard::PutNumber("Drive/Front/Left/AngleSet", fl.angle.Degrees().to<double>() + FL_ABS_OFFSET + 360 % 360);
            frc::SmartDashboard::PutNumber("Drive/Front/Right/AngleSet", fr.angle.Degrees().to<double>() + FR_ABS_OFFSET + 360 % 360);
            frc::SmartDashboard::PutNumber("Drive/Rear/Left/AngleSet", rl.angle.Degrees().to<double>() + RL_ABS_OFFSET + 360 % 360);
            frc::SmartDashboard::PutNumber("Drive/Rear/Right/AngleSet", rr.angle.Degrees().to<double>() + RR_ABS_OFFSET + 360 % 360);
    }

    void Drivetrain::publishData()
    {
        imuPub->publish(imuMsg);
        wheelStatePub->publish(wheelState);
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

    void Drivetrain::stickCallback(const sensor_msgs::msg::Joy msg){
        lastStickTime = frc::Timer::GetFPGATimestamp();
        lastStick = msg;
    }

    void Drivetrain::driveModeCallback(const std_msgs::msg::Int16 msg)
    {
        std::cout << "changing drivetrain to mode " << msg.data << std::endl;
        driveState = static_cast<ControlState>(msg.data);
    }

} // namespace robot
