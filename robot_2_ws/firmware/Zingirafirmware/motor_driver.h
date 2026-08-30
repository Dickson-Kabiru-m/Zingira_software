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
