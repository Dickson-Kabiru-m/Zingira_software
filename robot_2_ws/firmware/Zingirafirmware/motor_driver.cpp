#include "motor_driver.h"

/*
================================================
 Per-wheel pin lookup tables, indexed by WheelIndex
 (FL, FR, BL, BR). Lets setWheelMotor() stay generic
 instead of 4 near-identical copies of the same function.
================================================
*/
static const int PWM_PIN[NUM_WHEELS] = {FL_PWM, FR_PWM, BL_PWM, BR_PWM};
static const int IN1_PIN[NUM_WHEELS] = {FL_IN1, FR_IN1, BL_IN1, BR_IN1};
static const int IN2_PIN[NUM_WHEELS] = {FL_IN2, FR_IN2, BL_IN2, BR_IN2};

/*
================================================
 Initialize both L298N drivers (4 channels total)
================================================
*/
void motorBegin()
{
    for (int i = 0; i < NUM_WHEELS; i++)
    {
        pinMode(PWM_PIN[i], OUTPUT);
        pinMode(IN1_PIN[i], OUTPUT);
        pinMode(IN2_PIN[i], OUTPUT);
    }

    motorSetPWM(0, 0, 0, 0);
}

/*
================================================
 Single wheel control (used by the PID loop and by
 motorSetPWM() below)
================================================
*/
void setWheelMotor(int wheel, int pwm)
{
    bool reverse = false;

    if (pwm < 0)
    {
        reverse = true;
        pwm = -pwm;
    }

    if (pwm > MOTOR_MAX_PWM)
        pwm = MOTOR_MAX_PWM;

    /*
      Deadband compensation - see the Uno firmware's original
      comment. Below MOTOR_MIN_PWM the motor draws current but
      does not actually turn, which stalls the PID loop.
    */
    if (pwm > 0 && pwm < MOTOR_MIN_PWM)
        pwm = MOTOR_MIN_PWM;

    if (reverse)
    {
        digitalWrite(IN1_PIN[wheel], LOW);
        digitalWrite(IN2_PIN[wheel], HIGH);
    }
    else
    {
        digitalWrite(IN1_PIN[wheel], HIGH);
        digitalWrite(IN2_PIN[wheel], LOW);
    }

    analogWrite(PWM_PIN[wheel], pwm);
}

/*
================================================
 Main interface used by PID / open-loop commands
================================================
*/
void motorSetPWM(int flPWM, int frPWM, int blPWM, int brPWM)
{
    setWheelMotor(FL, flPWM);
    setWheelMotor(FR, frPWM);
    setWheelMotor(BL, blPWM);
    setWheelMotor(BR, brPWM);
}
