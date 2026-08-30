#include "pid_controller.h"
#include <math.h>

PIDController pid[NUM_WHEELS];

PIDController::PIDController()
{
    kp = 1.0;
    ki = 0.0;
    kd = 0.0;
    target = 0;
    integral = 0;
    previousError = 0;
    outputLimit = MOTOR_MAX_PWM;
}

void PIDController::setTarget(float value)
{
    target = value;
}

float PIDController::update(float measured)
{
    float error = target - measured;

    /*
      Error deadzone - see the Uno firmware's original comment.
      Prevents PWM chatter from encoder jitter around a held
      target.
    */
    if (fabs(error) < 1.0)
    {
        integral = 0;
        previousError = error;
        return 0;
    }

    integral += error;

    if (integral > 100.0) integral = 100.0;
    if (integral < -100.0) integral = -100.0;

    float derivative = error - previousError;
    float output = (kp * error) + (ki * integral) + (kd * derivative);
    previousError = error;

    if (output > outputLimit)  output = outputLimit;
    if (output < -outputLimit) output = -outputLimit;

    return output;
}

void PIDController::reset()
{
    integral = 0;
    previousError = 0;
}

void PIDController::setTunings(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}
