#pragma once
#include <units/time.h>
#include <units/velocity.h>

// Canbus ID mappings
#define DRIVE_LEFT_MASTER 1
#define DRIVE_LEFT_FOLLOWER 2
#define DRIVE_RIGHT_MASTER 3
#define DRIVE_RIGHT_FOLLOWER 4
#define IMU_ID 0

// Solenoid ID mappings
#define DRIVE_SHIFT_LOW 4 //4
#define DRIVE_SHIFT_HIGH 5 //5

// Which sticks to watch from the driverstation
#define USER_STICKS {0}

#define DRIVE_STICK_TOPIC "/sticks/stick0"
#define DRIVE_STICK_SCALAR {0.5, 1, 1, 1}
#define DRIVE_STICK_DEADBAND 0.05
#define DRIVE_STICK_POWER 2

/**
 *  Constants for the drivetrain
 **/ 

// How long before the drivetrain locks up after not recieving a new twist packet.
// Applies for both velocity, and open loop twist mode
#define DRIVE_TIMEOUT 0.030 // seconds

// Transmission gearing histeresis. low thresh should be lower than high thresh
#define DRIVE_SHIFT_LOW_THRESH 2 // lower Limit of high gear (rad/s)
#define DRIVE_SHIFT_HIGH_THRESH 2.5 // upper limit of low gear (rad/s)

// IMU covariance matricies (3x3) Row major about x, y, z axes
#define IMU_ORIENT_COVAR {.00000021103, 0, 0, 0, .000000077753, 0, 0, 0, .00000031994} // only show variances of data
#define IMU_ACCEL_COVAR {.000041837, 0, 0, 0, .00002734, 0, 0, 0, .000065791} // only show variances of data
#define IMU_ANG_VEL_COVAR {.00000069934, 0, 0, 0, .0000018495, 0, 0, 0, .00000088928} // only show variances of data

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
#define DRIVE_TICKS_PER_METER_LOW 65071 // ticks / meter
#define DRIVE_TICKS_PER_METER_HIGH 30306 // ticks / meter
