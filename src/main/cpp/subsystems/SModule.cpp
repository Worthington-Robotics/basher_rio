#include "subsystems/SModule.h"
#include "Constants.h"
#include <units/math.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/geometry/Rotation2d.h>

namespace robot
{

    SModule::SModule(int driveID, int angleID, int encodID, double offset, PIDF dValues, PIDF aValues)
    {
        angle = std::make_shared<TalonFX>(angleID);
        drive = std::make_shared<TalonFX>(driveID);
        encod = std::make_shared<CANCoder>(encodID);
        configMotors(offset, dValues, aValues);
        reset();
    }

    void SModule::configMotors(double offset, PIDF dValues, PIDF aValues)
    {
        //to convert from ticks to radians (2 radians per revolution) / (2048 ticks per revolution) * (GEAR REDUCTION)
        //to convert from ticks / 100ms to rps
        double bootPos = 0;
        // Configure front left drive falcon
        drive->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
        drive->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 5, 0);
        drive->SetSensorPhase(true);
        drive->SetInverted(true);
        drive->SetNeutralMode(NeutralMode::Brake);
        drive->SelectProfileSlot(0, 0);
        drive->Config_kF(0, dValues.f, 0);
        drive->Config_kP(0, dValues.p, 0);
        drive->Config_kI(0, dValues.i, 0);
        drive->Config_kD(0, dValues.d, 0);
        drive->Config_IntegralZone(0, 300, 0);
        drive->ConfigVoltageCompSaturation(11, 0);
        drive->EnableVoltageCompensation(true);
        drive->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));

        // Configure front left angle falcon
        encod->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 10, 0);
        encod->ConfigSensorDirection(false, 0);
        encod->ConfigAbsoluteSensorRange(AbsoluteSensorRange::Unsigned_0_to_360);
        encod->ConfigMagnetOffset(offset);

        angle->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
        angle->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 5, 0);
        angle->SetSensorPhase(true);
        angle->SetInverted(true);
        angle->SetNeutralMode(NeutralMode::Brake);
        angle->SelectProfileSlot(0, 0);
        angle->Config_kF(0, aValues.f, 0);
        angle->Config_kP(0, aValues.p, 0);
        angle->Config_kI(0, aValues.i, 0);
        angle->Config_kD(0, aValues.d, 0);
        angle->Config_IntegralZone(0, 300, 0);
        angle->ConfigVoltageCompSaturation(11, 0);
        angle->EnableVoltageCompensation(true);
        angle->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40, 0, 0.02));
        bootPos = std::fmod((encod->GetAbsolutePosition() / 180 * M_PI + 2 * M_PI), (2 * M_PI));
        angle->SetSelectedSensorPosition(bootPos / SWERVE_ANGLE_POS_TTR, 0, 0);
    }

    void SModule::setInvertDrive(bool invert){
        drive->SetInverted(invert);
    }

    void SModule::reset()
    {
    }

    frc::SwerveModuleState Optimize(const frc::SwerveModuleState &desiredState,
                                    const frc::Rotation2d &currentAngle)
    {
        auto delta = desiredState.angle - currentAngle;
        if (units::math::abs(delta.Degrees()) > 90_deg)
        {
            return {-desiredState.speed, desiredState.angle + frc::Rotation2d{180_deg}};
        }
        else
        {
            return {desiredState.speed, desiredState.angle};
        }
    }

    void SModule::setMotors(frc::SwerveModuleState ss)
    {
        auto ssO = Optimize(ss, units::degree_t(angle->GetSelectedSensorPosition() / TICKS_PER_DEGREE / (64 / 5)));
        angle->Set(ControlMode::Position, ssO.angle.Degrees().to<double>() * TICKS_PER_DEGREE * (64 / 5));
        drive->Set(ControlMode::PercentOutput, ssO.speed.to<double>());
    }

    frc::SwerveModuleState SModule::getState()
    {
        //(ticks / 100ms) / (ticks / revolution) * (Circumfrence [Diameter * Pi] / revolution) * (1s / 100ms) / (12in / foot)
        return frc::SwerveModuleState{units::feet_per_second_t{drive->GetSelectedSensorVelocity() / 2048 * 4 * M_PI * 10 / 12}, frc::Rotation2d{units::degree_t{angle->GetSelectedSensorPosition() / TICKS_PER_DEGREE / (64 / 5)}}};
    }

    sSensorData SModule::getData()
    {
        return sSensorData{angle->GetSelectedSensorPosition(), drive->GetSelectedSensorPosition(), drive->GetSelectedSensorVelocity(), encod->GetAbsolutePosition()};
    }

    // frc::SwerveModuleState optimize(frc::SwerveModuleState ss, double currDegree){
    //     if(std::abs(ss.angle.Degrees().to<double>() - currDegree) > 90){
    //         ss.angle = ss.angle.RotateBy(units::degree_t{180});
    //         ss.speed = ss.speed.
    //     }

    // }

} // namespace robot
