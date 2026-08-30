#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include "config.h"

/*
  Control mode - tracks whether the last motion command was a
  PID velocity target ('m') or a raw open-loop PWM value ('o').
  The main loop uses this to decide whether it's allowed to run
  the PID block, so 'o' commands aren't immediately overwritten
  by the next PID cycle.
*/
enum ControlMode
{
    MODE_PID,
    MODE_OPEN_LOOP
};

ControlMode getControlMode();

void serialBegin();
void serialUpdate();

/*
  args holds up to 4 whitespace-separated floats parsed from the
  line, in wheel order (FL, FR, BL, BR) for 'm'/'o', or (kp, ki, kd)
  for 'p'. Unused slots are 0.0.
*/
void processCommand(char command, float args[4]);

#endif
