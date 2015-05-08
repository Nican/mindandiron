// Put professional header here
// Include some type of copyright


#include <Herkulex.h>
#include <TimerOne.h>

#include "battery_includes.h"
#include "rc_includes.h"
#include "camera_servo_includes.h"


int camServoSetpoint = 0;
int ledState = LOW;
volatile int ledCounter = 0;
uint16_t loopCounter = 0;


void setup() {
    Serial.begin(9600);
    setupRC();

    Herkulex.beginSerial1(115200);
    Herkulex.reboot(CAM_SERVO_ID);  // Reboot first motor
    delay(500);
    Herkulex.initialize();  // Initialize motors
    delay(200);
    Herkulex.moveOneAngle(CAM_SERVO_ID, camServoSetpoint, 500, LED_BLUE);
    Herkulex.setLed(CAM_SERVO_ID, LED_CYAN);
    delay(2000);

    // This interrupt is going too fast, even when set to 5000000
    Timer1.initialize(5000000);  // Timer period in microseconds (max 8.3s)
    Timer1.attachInterrupt(handleLights);
    pinMode(LIGHT_OUT_PIN, OUTPUT);
}

void loop() {
    readComputerCommands(&camServoSetpoint);

//    REMOVE COMMENTS WHEN NOT TESTING UNDER RC
//    if (getAuto()) {
        int command = camServoSetpoint;  // Make copy in case calue is updated
        Herkulex.moveOneAngle(CAM_SERVO_ID, command, 500, LED_GREEN);
//    } else {
//        Herkulex.moveOneAngle(CAM_SERVO_ID, 0, 1000, LED_BLUE);
//        Herkulex.setLed(CAM_SERVO_ID, LED_PINK);
//    }

    printDataToComputer(Herkulex.getAngle(CAM_SERVO_ID),
                        getCurrent(), getPaused());
    delay(5);

    // Periodically reboots the servo, every three minutes (roughly)
    loopCounter++;
    if (loopCounter == 15000) {
        Herkulex.reboot(CAM_SERVO_ID);  // Reboot first motor
        delay(500);
        Herkulex.initialize();  // Initialize motors
        delay(200);
        loopCounter = 0;
    }
}

// Blinks the LED when paused, leaves it solidly on while running
// Sweeps the cam servo when a sweep direction is set.
void handleLights(void) {
    if (ledCounter++ >= 2) {
        if (ledState == LOW) {
            ledState = HIGH;
        } else if (getPaused()) {
            ledState = LOW;
        }
        digitalWrite(LIGHT_OUT_PIN, ledState);
        ledCounter = 0;
    }
}
