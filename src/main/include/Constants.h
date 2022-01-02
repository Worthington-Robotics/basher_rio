#pragma once
#include <units/time.h>
#include <units/velocity.h>

// Canbus ID mappings
#define DRIVE_FR_DRIVE 1
#define DRIVE_FR_ANGLE 2
#define DRIVE_FR_ENCOD 1

#define DRIVE_FL_DRIVE 3
#define DRIVE_FL_ANGLE 4
#define DRIVE_FL_ENCOD 2

#define DRIVE_RR_DRIVE 5
#define DRIVE_RR_ANGLE 6
#define DRIVE_RR_ENCOD 3

#define DRIVE_RL_DRIVE 7
#define DRIVE_RL_ANGLE 8
#define DRIVE_RL_ENCOD 4


#define IMU_ID 0

// Solenoid ID mappings
#define DRIVE_SHIFT_LOW 2
#define DRIVE_SHIFT_HIGH 3

// Which sticks to watch from the driverstation
#define USER_STICKS {0}

#define DRIVE_STICK_TOPIC "/sticks/stick0"
#define DRIVE_STICK_SCALAR {1, -1, 1, 1}
#define DRIVE_STICK_DEADBAND 0.15
#define DRIVE_STICK_POWER 2

/**
 *  Constants for the drivetrain
 **/ 

// How long before the drivetrain locks up after not recieving a new twist packet.
// Applies for both velocity, and open loop twist mode
#define DRIVE_TIMEOUT 0.030 // seconds

// Transmission gearing histeresis. low thresh should be lower than high thresh
#define DRIVE_SHIFT_LOW_THRESH 6.5 // lower Limit of high gear (rad/s)
#define DRIVE_SHIFT_HIGH_THRESH 8.5 // upper limit of low gear (rad/s)

// IMU covariance matricies (3x3) Row major about x, y, z axes
#define IMU_ORIENT_COVAR {1, 0, 0, 0, 1, 0, 0, 0, 1} // only show variances of data
#define IMU_ACCEL_COVAR {1, 0, 0, 0, 1, 0, 0, 0, 1} // only show variances of data
#define IMU_ANG_VEL_COVAR {1, 0, 0, 0, 1, 0, 0, 0, 1} // only show variances of data

// PID constants for left and right transmission velocity control
#define DRIVE_LEFT_KF 0
#define DRIVE_LEFT_KP 2.5
#define DRIVE_LEFT_KI 0.0
#define DRIVE_LEFT_KD 0
#define DRIVE_LEFT_IACCUM 300
#define DRIVE_RIGHT_KF 0.0
#define DRIVE_RIGHT_KP 0.0
#define DRIVE_RIGHT_KI 0.0
#define DRIVE_RIGHT_KD 0.0
#define DRIVE_RIGHT_IACCUM 300

// voltage compensation voltage
#define DRIVE_VCOMP_VOLTAGE 11.0

// physical constants
#define DRIVE_TRACK_WIDTH 0.5 // (meters)
#define TICKS_PER_DEGREE 2048.0/360
#define DRIVE_CPR_TO_RAD

#define SWERVE_ANGLE_GEARING 5 / 64
#define SWERVE_ANGLE_POS_TTR 2 * 3.14159 / 2048.0 * SWERVE_ANGLE_GEARING
#define SWERVE_ANGLE_VEL_TTR SWERVE_ANGLE_POS_TTR * 10

#define FR_ABS_OFFSET 291.8
#define FL_ABS_OFFSET 265.6
#define RR_ABS_OFFSET 34.2
#define RL_ABS_OFFSET 350.8
