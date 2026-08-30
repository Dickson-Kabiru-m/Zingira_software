#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "config.h"

void motorBegin();

// Sets all 4 wheels at once. Order: FL, FR, BL, BR.
void motorSetPWM(int flPWM, int frPWM, int blPWM, int brPWM);

// Sets a single wheel by WheelIndex (FL/FR/BL/BR).
void setWheelMotor(int wheel, int pwm);

#endif

/*
******** LEFT L298N ********
 Channel A (ENA/IN1/IN2) -> Front Left
 Channel B (ENB/IN3/IN4) -> Back Left

******** RIGHT L298N ********
 Channel A (ENA/IN1/IN2) -> Front Right
 Channel B (ENB/IN3/IN4) -> Back Right

 See config.h for exact pin numbers.

 ******* Encoders ********
 Each wheel has its own quadrature encoder now (no more
 shared front/rear channel). See config.h.

 NOTE: The encoder vcc - 5V and GND - GND

 ******* GND SHARING *******
 Arduino GND + both L298N motor driver GNDs + all 4 encoder GNDs
*/

/*
FL encoder A / B	  2 (int) / 22
FR encoder A / B	  3 (int) / 24
BL encoder A / B	  18 (int) / 26
BR encoder A / B	  19 (int) / 28

Left board: ENA(FL) / IN1 / IN2	    5 (PWM) / 30 / 31
Left board: ENB(BL) / IN3 / IN4	    6 (PWM) / 32 / 33
Right board: ENA(FR) / IN1 / IN2	  7 (PWM) / 34 / 35
Right board: ENB(BR) / IN3 / IN4	  8 (PWM) / 36 / 37

Servo	  9 (PWM)

MPU6050 SDA / SCL	  20 / 21
*/
