#include "serial_protocol.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "pid_controller.h"
#include "watchdog.h"

String inputString = "";
bool commandReady = false;

static ControlMode currentMode = MODE_PID;

ControlMode getControlMode()
{
    return currentMode;
}

void serialBegin()
{
    Serial.begin(SERIAL_BAUD);
    inputString.reserve(60);
}

/*
  Manually split "arg1 arg2 arg3 arg4" into up to 4 floats using
  String::toFloat(). Avoids sscanf("%f"), which is unreliable on
  AVR (see the Uno firmware's original comment - same reasoning
  applies on the Mega's avr-libc).

  Any args beyond the 4th are ignored. Missing trailing args are
  left at 0.0 (the caller pre-zeroes the array).
*/
void parseArgs(const String &args, float values[4])
{
    int start = 0;

    for (int i = 0; i < 4; i++)
    {
        int spaceIndex = args.indexOf(' ', start);

        if (spaceIndex == -1)
        {
            values[i] = args.substring(start).toFloat();
            return;
        }

        values[i] = args.substring(start, spaceIndex).toFloat();
        start = spaceIndex + 1;
    }
}

void serialUpdate()
{
    while (Serial.available())
    {
        char c = Serial.read();

        // Check for carriage return or newline BEFORE adding the character
        // to the string. This stops whitespace/hidden formatting chars
        // from corrupting the argument parsing.
        if (c == '\r' || c == '\n')
        {
            if (inputString.length() > 0)
            {
                commandReady = true;
            }
        }
        else
        {
            inputString += c;

            // Safety guard: if a huge amount of data arrives with no
            // terminator, drop it rather than growing forever.
            // Widened vs the Uno firmware's 40-char limit since a
            // 4-value command line is longer than a 2-value one.
            if (inputString.length() > 60)
            {
                inputString = "";
            }
        }

        if (commandReady)
        {
            char command = inputString.charAt(0);

            float args[4] = {0.0, 0.0, 0.0, 0.0};

            int firstSpace = inputString.indexOf(' ');

            if (firstSpace != -1)
            {
                String rest = inputString.substring(firstSpace + 1);
                parseArgs(rest, args);
            }

            processCommand(command, args);

            inputString = "";
            commandReady = false;
        }
    }
}

void processCommand(char command, float args[4])
{
    switch (command)
    {
        case 'e':
            // ROS asks for encoders. We return cumulative ticks for
            // all 4 wheels, space separated, in FL FR BL BR order.
            // NO "OK" text is permitted here.
            // NOTE: does NOT reset the watchdog - see 'm'/'o' below.
            Serial.print(encoders.getTicks(FL));
            Serial.print(" ");
            Serial.print(encoders.getTicks(FR));
            Serial.print(" ");
            Serial.print(encoders.getTicks(BL));
            Serial.print(" ");
            Serial.println(encoders.getTicks(BR));
            break;

        case 'm':
            // ROS sets 4 target velocities (ticks/PID-period), one
            // per wheel, FL FR BL BR order. Live motion command, so
            // it resets the watchdog and switches to PID mode.
            watchdogReset();
            currentMode = MODE_PID;
            pid[FL].setTarget(args[0]);
            pid[FR].setTarget(args[1]);
            pid[BL].setTarget(args[2]);
            pid[BR].setTarget(args[3]);
            break;

        case 'o':
            // Raw open-loop PWM, one per wheel, FL FR BL BR order.
            // Live motion command, so it resets the watchdog and
            // switches to open-loop mode so the PID block in loop()
            // does not immediately overwrite this.
            watchdogReset();
            currentMode = MODE_OPEN_LOOP;
            motorSetPWM((int)args[0], (int)args[1], (int)args[2], (int)args[3]);
            break;

        case 'r':
            // Resetting encoder counters is not a motion command -
            // does NOT reset the watchdog.
            encoders.reset();
            break;

        case 'p':
            // Same kp/ki/kd applied to all 4 wheels (args[0..2]).
            for (int i = 0; i < NUM_WHEELS; i++)
            {
                pid[i].setTunings(args[0], args[1], args[2]);
            }
            break;

        case 's':
            currentMode = MODE_PID;
            motorSetPWM(0, 0, 0, 0);
            for (int i = 0; i < NUM_WHEELS; i++)
            {
                pid[i].reset();
            }
            break;

        default:
            // Do nothing to avoid polluting the buffer if trash data is
            // received.
            break;
    }
}
