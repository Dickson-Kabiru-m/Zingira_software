#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>
#include "config.h"

class EncoderDriver
{
public:
    EncoderDriver();
    void begin();

    long getTicks(int wheel);
    void reset();
    void update();
    long getDelta(int wheel);

    // Static counters accessed directly by the 4 ISRs.
    // Indexed by WheelIndex (FL, FR, BL, BR).
    static volatile long counter[NUM_WHEELS];

private:
    long lastTicks[NUM_WHEELS];
    long delta[NUM_WHEELS];
};

extern EncoderDriver encoders;

/*
 One ISR per wheel - attachInterrupt() needs a distinct
 function pointer per interrupt, so these can't be collapsed
 into a single indexed handler.
*/
void flEncoderISR();
void frEncoderISR();
void blEncoderISR();
void brEncoderISR();

#endif
