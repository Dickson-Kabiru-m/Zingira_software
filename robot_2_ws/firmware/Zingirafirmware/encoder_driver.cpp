#include "encoder_driver.h"

EncoderDriver encoders;

volatile long EncoderDriver::counter[NUM_WHEELS] = {0, 0, 0, 0};

/*
 Each ISR triggers on CHANGE of that wheel's channel A pin
 and reads channel B to resolve direction. The sign convention
 (A==B -> decrement vs increment) mirrors the Uno firmware's
 left/right ISRs - FL and BL follow the old "left" convention,
 FR and BR follow the old "right" convention, since they sit on
 the same physical side of the robot.
*/

void flEncoderISR()
{
    int a = digitalRead(FL_ENCODER_A);
    int b = digitalRead(FL_ENCODER_B);

    if (a == b) EncoderDriver::counter[FL]--;
    else        EncoderDriver::counter[FL]++;
}

void blEncoderISR()
{
    int a = digitalRead(BL_ENCODER_A);
    int b = digitalRead(BL_ENCODER_B);

    if (a == b) EncoderDriver::counter[BL]--;
    else        EncoderDriver::counter[BL]++;
}

void frEncoderISR()
{
    int a = digitalRead(FR_ENCODER_A);
    int b = digitalRead(FR_ENCODER_B);

    if (a == b) EncoderDriver::counter[FR]++;
    else        EncoderDriver::counter[FR]--;
}

void brEncoderISR()
{
    int a = digitalRead(BR_ENCODER_A);
    int b = digitalRead(BR_ENCODER_B);

    if (a == b) EncoderDriver::counter[BR]++;
    else        EncoderDriver::counter[BR]--;
}

EncoderDriver::EncoderDriver()
{
    for (int i = 0; i < NUM_WHEELS; i++)
    {
        lastTicks[i] = 0;
        delta[i] = 0;
    }
}

void EncoderDriver::begin()
{
    pinMode(FL_ENCODER_A, INPUT_PULLUP);
    pinMode(FL_ENCODER_B, INPUT_PULLUP);
    pinMode(FR_ENCODER_A, INPUT_PULLUP);
    pinMode(FR_ENCODER_B, INPUT_PULLUP);
    pinMode(BL_ENCODER_A, INPUT_PULLUP);
    pinMode(BL_ENCODER_B, INPUT_PULLUP);
    pinMode(BR_ENCODER_A, INPUT_PULLUP);
    pinMode(BR_ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(FL_ENCODER_A), flEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FR_ENCODER_A), frEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BL_ENCODER_A), blEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BR_ENCODER_A), brEncoderISR, CHANGE);
}

long EncoderDriver::getTicks(int wheel)
{
    noInterrupts();
    long value = counter[wheel];
    interrupts();
    return value;
}

void EncoderDriver::reset()
{
    noInterrupts();
    for (int i = 0; i < NUM_WHEELS; i++)
    {
        counter[i] = 0;
    }
    interrupts();

    for (int i = 0; i < NUM_WHEELS; i++)
    {
        lastTicks[i] = 0;
        delta[i] = 0;
    }
}

void EncoderDriver::update()
{
    for (int i = 0; i < NUM_WHEELS; i++)
    {
        long current = getTicks(i);
        delta[i] = current - lastTicks[i];
        lastTicks[i] = current;
    }
}

long EncoderDriver::getDelta(int wheel)
{
    return delta[wheel];
}
