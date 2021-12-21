#include "subsystems/drivetrain.h"
#include "Constants.h"
#include <frc/DriverStation.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Rotation2d.h>
#include "subsystems/userinput.h"


#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

using std::placeholders::_1;

namespace robot
{

    Drivetrain::Drivetrain()
    {
        leftMaster = std::make_shared<TalonFX>(DRIVE_LEFT_MASTER);
        rightMaster = std::make_shared<TalonFX>(DRIVE_RIGHT_MASTER);

        leftFollower = std::make_shared<TalonFX>(DRIVE_LEFT_FOLLOWER);
        rightFollower = std::make_shared<TalonFX>(DRIVE_RIGHT_FOLLOWER);

        imu = std::make_shared<PigeonIMU>(IMU_ID);

        shifter = std::make_shared<frc::DoubleSolenoid>(DRIVE_SHIFT_LOW, DRIVE_SHIFT_HIGH);

        driveOdom = std::make_shared<frc::DifferentialDriveOdometry>(frc::Rotation2d());

        configMotors();

        reset();
    }

    void Drivetrain::createRosBindings(rclcpp::Node *node)
    {
        // Create sensor data publishers
        imuPub = node->create_publisher<sensor_msgs::msg::Imu>("/drive/imu", rclcpp::SensorDataQoS());
        wheelStatePub = node->create_publisher<sensor_msgs::msg::JointState>("/drive/wheel_state", rclcpp::SensorDataQoS());
        poseStatePub = node->create_publisher<geometry_msgs::msg::Pose>("/drive/pose", rclcpp::SensorDataQoS());
        yawPub = node->create_publisher<std_msgs::msg::Float32>("/drive/heading", rclcpp::SensorDataQoS());

        // Create subscribers
        trajectorySub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>("/drive/active_traj", rclcpp::SystemDefaultsQoS(), std::bind(&Drivetrain::trajectoryCallback, this, _1));
        twistSub = node->create_subscription<geometry_msgs::msg::Twist>("/drive/velocity_twist", rclcpp::SensorDataQoS(), std::bind(&Drivetrain::twistCallback, this, _1));
        stickSub = node->create_subscription<sensor_msgs::msg::Joy>(DRIVE_STICK_TOPIC, rclcpp::SensorDataQoS(), std::bind(&Drivetrain::stickCallback, this, _1));

        transModeSub = node->create_subscription<std_msgs::msg::Int16>("/drive/trans_mode", rclcpp::SensorDataQoS(), std::bind(&Drivetrain::transModeCallback, this, _1));
        DriveModeSub = node->create_subscription<std_msgs::msg::Int16>("/drive/drive_mode", rclcpp::SensorDataQoS(), std::bind(&Drivetrain::driveModeCallback, this, _1));
    }

    void Drivetrain::configMotors()
    {
        // Configure left master falcon
        leftMaster->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
        leftMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 5, 0);
        leftMaster->SetSensorPhase(true);
        leftMaster->SetInverted(true);
        leftMaster->SetNeutralMode(NeutralMode::Brake);
        leftMaster->SelectProfileSlot(0, 0);
        leftMaster->Config_kF(0, DRIVE_LEFT_KF, 0);
        leftMaster->Config_kP(0, DRIVE_LEFT_KP, 0);
        leftMaster->Config_kI(0, DRIVE_LEFT_KI, 0);
        leftMaster->Config_kD(0, DRIVE_LEFT_KD, 0);
        leftMaster->Config_IntegralZone(0, DRIVE_LEFT_IACCUM, 0);
        leftMaster->ConfigVoltageCompSaturation(DRIVE_VCOMP_VOLTAGE, 0);
        leftMaster->EnableVoltageCompensation(true);
        leftMaster->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));

        // configure right master falcon
        rightMaster->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
        rightMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 5, 0);
        rightMaster->SetSensorPhase(false);
        rightMaster->SetInverted(true);
        rightMaster->SetNeutralMode(NeutralMode::Brake);
        rightMaster->SelectProfileSlot(0, 0);
        rightMaster->Config_kF(0, DRIVE_RIGHT_KF, 0);
        rightMaster->Config_kP(0, DRIVE_RIGHT_KP, 0);
        rightMaster->Config_kI(0, DRIVE_RIGHT_KI, 0);
        rightMaster->Config_kD(0, DRIVE_RIGHT_KD, 0);
        rightMaster->Config_IntegralZone(0, DRIVE_RIGHT_IACCUM, 0);
        rightMaster->ConfigVoltageCompSaturation(DRIVE_VCOMP_VOLTAGE, 0);
        rightMaster->EnableVoltageCompensation(true);
        rightMaster->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));

        //configure left follower falcon
        leftFollower->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 0);
        leftFollower->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 0);
        leftFollower->SetSensorPhase(true);
        leftFollower->SetInverted(true);
        leftFollower->ConfigVoltageCompSaturation(DRIVE_VCOMP_VOLTAGE, 0);
        leftFollower->EnableVoltageCompensation(true);
        leftFollower->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));
        leftFollower->Follow(*leftMaster);

        //configure right follower falcon
        rightFollower->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 0);
        rightFollower->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 0);
        rightFollower->SetSensorPhase(false);
        rightFollower->SetInverted(true);
        rightFollower->ConfigVoltageCompSaturation(DRIVE_VCOMP_VOLTAGE, 0);
        rightFollower->EnableVoltageCompensation(true);
        rightFollower->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));
        rightFollower->Follow(*rightMaster);
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
        imu->SetFusedHeading(0);
        imuMsg = sensor_msgs::msg::Imu();
        //set covariances
        imuMsg.orientation_covariance = IMU_ORIENT_COVAR;
        imuMsg.linear_acceleration_covariance = IMU_ACCEL_COVAR;
        imuMsg.angular_velocity_covariance = IMU_ANG_VEL_COVAR;

        // default values to zero
        imuMsg.linear_acceleration.x = 0;
        imuMsg.linear_acceleration.y = 0;
        imuMsg.linear_acceleration.z = 0;
        imuMsg.angular_velocity.x = 0;
        imuMsg.angular_velocity.y = 0;
        imuMsg.angular_velocity.z = 0;

        // build the ramsete controller
        controller = frc::RamseteController();

        //reset the pose
        driveOdom->ResetPosition(frc::Pose2d(), frc::Rotation2d());

        // clear demands and reset time
        leftDemand = rightDemand = 0;
        lastTwistTime = 0;
        shifterDemand = frc::DoubleSolenoid::Value::kForward; // default to low gear

        shiftState = VELOCITY_THRESH;
        driveState = OPEN_LOOP_STICK;

        //reseting encoders, sensors, and the IMU

        leftMaster->SetSelectedSensorPosition(0);
        rightMaster->SetSelectedSensorPosition(0);


        double orientData[4];
        imu->Get6dQuaternion(orientData);
        imuMsg.orientation.w = orientData[0];
        imuMsg.orientation.x = orientData[1];
        imuMsg.orientation.y = orientData[2];
        imuMsg.orientation.z = orientData[3];

        //this calculates the conjugate quaternion which is used to offset the rotation
        double norm = imuMsg.orientation.w * imuMsg.orientation.w + imuMsg.orientation.x * imuMsg.orientation.x
        + imuMsg.orientation.y * imuMsg.orientation.y + imuMsg.orientation.z * imuMsg.orientation.z;
        offsetQuaternion.w = imuMsg.orientation.w / norm;
        offsetQuaternion.x = -imuMsg.orientation.x / norm;
        offsetQuaternion.y = -imuMsg.orientation.y / norm;
        offsetQuaternion.z = -imuMsg.orientation.z / norm;

        imuMsg.orientation = hamiltonProduct(offsetQuaternion, imuMsg.orientation);

        poseState.orientation = imuMsg.orientation;
        poseState.position.x = driveOdom->GetPose().X().value();
        poseState.position.y = driveOdom->GetPose().Y().value();
        poseState.position.z = 0;
        yawMsg.data = 0;
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

        imuMsg.orientation = hamiltonProduct(offsetQuaternion, imuMsg.orientation);

        //Read the current left and right joint states and adjust them using the shifter state
        //kReverse is high gear and the *10 is used to convert from ticks / 100ms to m/s
        //the negitives are to set the sensor phase because I am lazy
        if(shifterDemand == frc::DoubleSolenoid::kReverse) {
            wheelState.position = {leftMaster->GetSelectedSensorPosition() / DRIVE_TICKS_PER_METER_HIGH, -rightMaster->GetSelectedSensorPosition() / DRIVE_TICKS_PER_METER_HIGH};
            wheelState.velocity = {leftMaster->GetSelectedSensorVelocity() / DRIVE_TICKS_PER_METER_HIGH * 10, -rightMaster->GetSelectedSensorVelocity() / DRIVE_TICKS_PER_METER_HIGH * 10};
            wheelState.effort = {leftMaster->GetStatorCurrent(), rightMaster->GetStatorCurrent()};
        } else {
            wheelState.position = {leftMaster->GetSelectedSensorPosition() / DRIVE_TICKS_PER_METER_LOW, -rightMaster->GetSelectedSensorPosition() / DRIVE_TICKS_PER_METER_LOW};
            wheelState.velocity = {leftMaster->GetSelectedSensorVelocity() / DRIVE_TICKS_PER_METER_LOW * 10, -rightMaster->GetSelectedSensorVelocity() / DRIVE_TICKS_PER_METER_LOW * 10};
            wheelState.effort = {leftMaster->GetStatorCurrent(), rightMaster->GetStatorCurrent()};
        }
        double ypr = 0;
        ypr = imu->GetFusedHeading();
        driveOdom -> Update(frc::Rotation2d{units::degree_t(ypr)}, units::length::meter_t(wheelState.position.at(0)), units::length::meter_t(wheelState.position.at(1)));
        poseState.orientation = imuMsg.orientation;
        poseState.position.x = driveOdom->GetPose().X().value();
        poseState.position.y = driveOdom->GetPose().Y().value();
        poseState.position.z = 0;
        yawMsg.data = ypr;
    }


    // Average the wheel state velocities
    double getFwdVelocity(sensor_msgs::msg::JointState wheelState)
    {
        return (wheelState.velocity.at(0) + wheelState.velocity.at(1)) / 2.0;
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

    void Drivetrain::onLoop()
    {
        // Read sensors
        updateSensorData();
        bool button1UserInput = lastStick.buttons.at(1);
        if(button1UserInput){
            shiftState = USER_INPUT;
        }

        switch (driveState)
        {
        case OPEN_LOOP_STICK:
            // if we are safe, set motor demands,
            if (lastStickTime + DRIVE_TIMEOUT > frc::Timer::GetFPGATimestamp())
            {
                // parse the joy message
                std::vector<double> joyData = UserInput::scalarCut(lastStick, DRIVE_STICK_DEADBAND,
                                                                   DRIVE_STICK_POWER, DRIVE_STICK_SCALAR);

                //std::cout << "joyData " << joyData.at(0) << " " << joyData.at(1) << std::endl;

                auto stickTwist = geometry_msgs::msg::Twist();
                stickTwist.linear.x = joyData.at(0);
                stickTwist.angular.z = joyData.at(1);

                // convert to demands
                arcadeDrive(stickTwist, leftDemand, rightDemand);
            }
            else
            { // otherwise force motors to zero, there is stale data
                leftDemand = rightDemand = 0;
            }

            leftMaster->Set(ControlMode::PercentOutput, leftDemand);
            rightMaster->Set(ControlMode::PercentOutput, rightDemand);
            break;

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

            leftMaster->Set(ControlMode::PercentOutput, leftDemand);
            rightMaster->Set(ControlMode::PercentOutput, rightDemand);
            break;

        case RAMSETE:
        case PURSUIT: // for now have pursuit as an illegal mode
            frc::DriverStation::ReportWarning("Mode is not currently implemented");
            leftMaster->Set(ControlMode::PercentOutput, 0);
            rightMaster->Set(ControlMode::PercentOutput, 0);
            break;

        default:
            leftMaster->Set(ControlMode::PercentOutput, 0);
            rightMaster->Set(ControlMode::PercentOutput, 0);
            frc::DriverStation::ReportError("Drivetrain attempted to enter an illegal mode");
        }

        switch (shiftState)
        {
        case DISABLED:
            // set into off state
            shifterDemand = frc::DoubleSolenoid::Value::kOff;
            break;

        case USER_INPUT:
        case VELOCITY_THRESH:
            if (getFwdVelocity(wheelState) > DRIVE_SHIFT_HIGH_THRESH)
            {
                // set into high gear
                shifterDemand = frc::DoubleSolenoid::Value::kReverse;
            }
            else if (getFwdVelocity(wheelState) < DRIVE_SHIFT_LOW_THRESH)
            {
                // set into low gear
                shifterDemand = frc::DoubleSolenoid::Value::kForward;
            }
            break;

        case FIXED_HIGH:
            // set into high gear
            shifterDemand = frc::DoubleSolenoid::Value::kReverse;
            break;

        case FIXED_LOW:
            // set into low gear
            shifterDemand = frc::DoubleSolenoid::Value::kForward;
            break;
        
        // case USER_INPUT:
        //     if (getFwdVelocity(wheelState) > DRIVE_SHIFT_HIGH_THRESH)
        //     {
        //         // set into high gear
        //         shifterDemand = frc::DoubleSolenoid::Value::kReverse;
        //     }
        //     else if (getFwdVelocity(wheelState) < DRIVE_SHIFT_LOW_THRESH)
        //     {
        //         // set into low gear
        //         shifterDemand = frc::DoubleSolenoid::Value::kForward;
        //     }
        //     break;

        default:
            frc::DriverStation::ReportError("Transmissions attempted to enter an illegal mode");
        }

        shifter->Set(shifterDemand);
    }

    void Drivetrain::publishData()
    {
        imuPub->publish(imuMsg);
        wheelStatePub->publish(wheelState);
        poseStatePub->publish(poseState);
        yawPub->publish(yawMsg);
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

    void Drivetrain::transModeCallback(const std_msgs::msg::Int16 msg)
    {
        std::cout << "changing transmission to mode " << msg.data << std::endl;
        shiftState = static_cast<ShifterState>(msg.data);
    }

    void Drivetrain::driveModeCallback(const std_msgs::msg::Int16 msg)
    {
        std::cout << "changing drivetrain to mode " << msg.data << std::endl;
        driveState = static_cast<ControlState>(msg.data);
    }

} // namespace robot
