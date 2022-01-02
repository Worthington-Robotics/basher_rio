#pragma once

#include <ctre/Phoenix.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Rotation2d.h>

#define _USE_MATH_DEFINES
#include <cmath>

namespace robot
{

    struct PIDF
    {
        double p;
        double i;
        double d;
        double f;
    };

    struct sSensorData
    {
        double angleRel;
        double drivePos;
        double driveVel;
        double encAbs;
    };

    class SModule
    {
        public:

        SModule(int, int, int, double, PIDF, PIDF);
        /**
         * Override this function with all the nessecary code needed to reset a subsystem
         **/
        void reset();

        void setMotors(frc::SwerveModuleState);

        sSensorData getData();

        void setInvertDrive(bool);

    private:
        /**
         * Configure the associated motor controllers with their settings as specified in constants
         **/ 
        void configMotors(double, PIDF, PIDF);

        void updateSensorData();

        frc::SwerveModuleState optimize(frc::SwerveModuleState, double);

        //IO devices
        std::shared_ptr<TalonFX> drive, angle;
        std::shared_ptr<CANCoder> encod;
        
    };

} // namespace robot

