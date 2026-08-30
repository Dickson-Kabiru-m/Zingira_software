#include <Arduino.h>
#include "config.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "pid_controller.h"
#include "serial_protocol.h"
#include "watchdog.h"

/*
================================================
 PID LOOP
================================================
*/
unsigned long lastPIDUpdate = 0;

/*
================================================
 SETUP
================================================
*/
void setup()
{
    /*
      Start serial communication
    */
    serialBegin();

    /*
      Initialize both L298N drivers (4 independent channels)
    */
    motorBegin();

    /*
      Initialize all 4 encoders
    */
    encoders.begin();

    /*
      Initial PID tuning - same starting values as the Uno
      firmware, applied to all 4 wheels. Will be re-tuned once
      the Mega chassis is running (deadband/inertia will differ
      from the Uno rig).
    */
    for (int i = 0; i < NUM_WHEELS; i++)
    {
        pid[i].setTunings(2.0, 0.0, 0.2);
    }

    /*
      Start watchdog timer
    */
    watchdogReset();

    // Serial buffer stays 100% clean of text strings for ROS 2 -
    // no Serial.println() banner here.
}

/*
================================================
 MAIN LOOP
================================================
*/
void loop()
{
    /*
      Always listen for commands from Serial
    */
    serialUpdate();

    /*
      PID update loop (20Hz)
      Only runs in MODE_PID. In MODE_OPEN_LOOP, motorSetPWM()
      was already called directly by the 'o' command handler,
      so we must NOT overwrite it here every 50ms.
    */
    if (getControlMode() == MODE_PID && millis() - lastPIDUpdate >= PID_PERIOD)
    {
        lastPIDUpdate = millis();

        /*
          Update encoder differences for all 4 wheels -
          ticks travelled since last PID cycle.
        */
        encoders.update();

        int pwmOut[NUM_WHEELS];

        for (int i = 0; i < NUM_WHEELS; i++)
        {
            long ticks = encoders.getDelta(i);
            pwmOut[i] = (int)pid[i].update((float)ticks);
        }

        motorSetPWM(pwmOut[FL], pwmOut[FR], pwmOut[BL], pwmOut[BR]);
    }

    /*
      Safety watchdog
      If the Pi stops sending motion commands ('m' or 'o'), stop
      all 4 wheels. Note: querying encoders ('e') does NOT reset
      this timer, so a dead teleop link is caught even if odometry
      polling keeps running.
    */
    if (watchdogExpired())
    {
        motorSetPWM(0, 0, 0, 0);
        for (int i = 0; i < NUM_WHEELS; i++)
        {
            pid[i].reset();
        }
    }
}
