// Put professional header here
// Include some type of copyright


#include <Herkulex.h>
#include <TimerOne.h>

#include "rc_includes.h"


const int camServoID = 253;
#define MAX_CAM_SERVO_ANGLE 159

int camServoSetpoint = 0;
volatile int servoSweepDirection = 0;
const int servoSweepStep = 30;  // Degrees
int servoSweepCmd = 0;  // Degrees
int ledState = LOW;


void setup() {
    Serial.begin(9600);
    setupRC();

    Herkulex.beginSerial1(115200);
    Herkulex.reboot(camServoID);  // Reboot first motor
    delay(500);
    Herkulex.initialize();  // Initialize motors
    delay(200);
    Herkulex.moveOneAngle(camServoID, camServoSetpoint, 1000, LED_BLUE);
    Herkulex.setLed(camServoID, LED_CYAN);
    delay(2000);

    pinMode(LIGHT_OUT_PIN, OUTPUT);
    Timer1.initialize(1000000);  // Timer period in microseconds (max 8.3s)
    Timer1.attachInterrupt(handleLightsAndServoSweep);
}

void loop() {
    readComputerCommands(&camServoSetpoint);

    if (getAuto()) {
        int command = camServoSetpoint;  // Make copy in case calue is updated
        if (command == 255) {
            noInterrupts();
            if (servoSweepDirection == 0) {
                servoSweepDirection = 1;  // When command is 255, sweep servo
            }
            interrupts();
        } else {
            noInterrupts();
            if (servoSweepDirection != 0) {
                servoSweepDirection = 0;
            }
            interrupts();
            Herkulex.moveOneAngle(camServoID, command, 500, LED_GREEN);
        }
    } else {
        Herkulex.moveOneAngle(camServoID, 0, 1000, LED_BLUE);
        Herkulex.setLed(camServoID, LED_PINK);
    }

    printDataToComputer(Herkulex.getAngle(camServoID), getPaused());
}

// Blinks the LED when paused, leaves it solidly on while running
// Sweeps the cam servo when a sweep direction is set.
void handleLightsAndServoSweep(void) {
    if (ledState == LOW) {
        ledState = HIGH;
    } else if (getPaused()) {
        ledState = LOW;
    }
    digitalWrite(LIGHT_OUT_PIN, ledState);

    if (servoSweepDirection != 0) {
        if (abs(servoSweepCmd + servoSweepDirection * servoSweepStep) > MAX_CAM_SERVO_ANGLE) {
            servoSweepDirection *= -1;
        }
        servoSweepCmd += servoSweepDirection * servoSweepStep;
        Herkulex.moveOneAngle(camServoID, servoSweepCmd, 500, LED_GREEN);
    }
}
