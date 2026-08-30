#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

class PIDController
{
public:
    PIDController();
    void setTarget(float target);
    float update(float measured);
    void reset();
    void setTunings(float kp, float ki, float kd);

private:
    float kp;
    float ki;
    float kd;
    float target;
    float integral;
    float previousError;
    float outputLimit;
};

// One controller per wheel, indexed by WheelIndex (FL, FR, BL, BR).
extern PIDController pid[NUM_WHEELS];

#endif
