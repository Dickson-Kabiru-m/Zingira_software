#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/*
================================================
 WHEEL INDEXING
================================================
 Every array in this firmware (encoders, motors, PID
 controllers, serial payloads) is ordered:

   0 = front_left  (FL)
   1 = front_right (FR)
   2 = back_left   (BL)
   3 = back_right  (BR)

 This MUST match the joint order assumed by
 robot_2_hardware/Robot2System on the ROS 2 side.
*/
#define NUM_WHEELS 4

enum WheelIndex
{
    FL = 0,
    FR = 1,
    BL = 2,
    BR = 3
};

/*
================================================
 MOTOR DRIVER PINS (2x L298N, one per side)
================================================
 Unlike the Uno build, front and back motors on each
 side are wired to SEPARATE channels of the same L298N
 board, so each wheel gets its own PWM + direction pins
 instead of sharing one channel.

 LEFT L298N board:
   Channel A -> Front Left
   Channel B -> Back Left

 RIGHT L298N board:
   Channel A -> Front Right
   Channel B -> Back Right
*/

// LEFT L298N - Channel A (Front Left)
#define FL_PWM   5   // ENA
#define FL_IN1   30
#define FL_IN2   31

// LEFT L298N - Channel B (Back Left)
#define BL_PWM   6   // ENB
#define BL_IN1   32
#define BL_IN2   33

// RIGHT L298N - Channel A (Front Right)
#define FR_PWM   7   // ENA
#define FR_IN1   34
#define FR_IN2   35

// RIGHT L298N - Channel B (Back Right)
#define BR_PWM   8   // ENB
#define BR_IN1   36
#define BR_IN2   37

/*
================================================
 ENCODERS
================================================
 The Mega has 6 hardware-interrupt pins: 2, 3, 18, 19,
 20, 21. All 4 A-channels use one each, leaving 20/21
 free and dedicated to I2C for the IMU below.
*/

#define FL_ENCODER_A   2    // INT (hardware interrupt)
#define FL_ENCODER_B   22

#define FR_ENCODER_A   3    // INT (hardware interrupt)
#define FR_ENCODER_B   24

#define BL_ENCODER_A   18   // INT (hardware interrupt)
#define BL_ENCODER_B   26

#define BR_ENCODER_A   19   // INT (hardware interrupt)
#define BR_ENCODER_B   28

/*
================================================
 RESERVED
================================================
*/

// Future servo
#define SERVO_PIN 9

// MPU6050 (Mega hardware I2C - do not reuse for anything else)
// SDA = 20
// SCL = 21

/*
================================================
 ROBOT PARAMETERS
================================================
*/

#define ENCODER_TICKS_PER_REV 1980
#define MOTOR_MAX_PWM 255

// Measured minimum PWM from testing.
// NOTE: re-measure this on the Mega build - different L298N
// boards/motors than the Uno rig may have a different deadband.
#define MOTOR_MIN_PWM 60

/*
 PID
*/
#define PID_PERIOD 50   // ms

/*
 SERIAL
*/
#define SERIAL_BAUD 57600

#endif
