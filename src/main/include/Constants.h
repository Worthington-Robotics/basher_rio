#pragma once
#include <units/time.h>
#include <units/velocity.h>

// Canbus ID mappings
#define DRIVE_LEFT_MASTER 1
#define DRIVE_RIGHT_MASTER 2
#define DRIVE_LEFT_FOLLOWER 3
#define DRIVE_RIGHT_FOLLOWER 4

// Solenoid ID mappings
#define DRIVE_SHIFT_LOW 1
#define DRIVE_SHIFT_HIGH 2

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
#define DRIVE_LEFT_KF 0.0
#define DRIVE_LEFT_KP 0.0
#define DRIVE_LEFT_KI 0.0
#define DRIVE_LEFT_KD 0.0
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
#define DRIVE_CPR_TO_RAD
