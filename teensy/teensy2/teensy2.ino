// Put professional header here
// Include some type of copyright


#include <Herkulex.h>
#include <TimerOne.h>

#include "battery_includes.h"
#include "rc_includes.h"
#include "camera_servo_includes.h"


#define SORTER_SWITCH_1_PIN A9
#define SORTER_1_CUTOFF_HIGH 30  // For Schmitt trigger
#define SORTER_1_CUTOFF_LOW 4
#define SORTER_1_OUT_PIN 20
#define SORTER_SWITCH_2_PIN A8
#define SORTER_2_CUTOFF_HIGH 30  // For Schmitt trigger
#define SORTER_2_CUTOFF_LOW 4
#define SORTER_2_OUT_PIN 21


int camServoSetpoint = 0;
int ledState = LOW;
volatile int ledCounter = 0;
uint16_t loopCounter = 0;
int sorterSwitch1Value = 0;  // For the switch 1 readings
int sorter1CMDValue = 0;
int sorterSwitch2Value = 0;  // For the switch 2 readings
int sorter2CMDValue = 0;

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
    
    // SET UP THE SORTER COMMUNICATION
    pinMode(SORTER_1_OUT_PIN, OUTPUT);
    pinMode(SORTER_2_OUT_PIN, OUTPUT);
}

void loop() {
    readComputerCommands(&camServoSetpoint);

    if (getAuto()) {
        int command = camServoSetpoint;  // Make copy in case calue is updated
        Herkulex.moveOneAngle(CAM_SERVO_ID, command, 500, LED_GREEN);
    } else {
        Herkulex.moveOneAngle(CAM_SERVO_ID, 0, 1000, LED_BLUE);
        Herkulex.setLed(CAM_SERVO_ID, LED_PINK);
    }


    sorterSwitch1Value = analogRead(SORTER_SWITCH_1_PIN);
    sorterSwitch2Value = analogRead(SORTER_SWITCH_2_PIN);
   
    if (sorterSwitch1Value > SORTER_1_CUTOFF_HIGH) {
        sorter1CMDValue = HIGH;
    } else if (sorterSwitch1Value < SORTER_1_CUTOFF_LOW) {
        sorter1CMDValue = LOW;
    }
    if (sorterSwitch2Value > SORTER_2_CUTOFF_HIGH) {
        sorter2CMDValue = HIGH;
    } else if (sorterSwitch2Value < SORTER_2_CUTOFF_LOW) {
        sorter2CMDValue = LOW;
    }

    digitalWrite(SORTER_1_OUT_PIN, sorter1CMDValue);
    digitalWrite(SORTER_2_OUT_PIN, sorter2CMDValue);

//    Serial.print("Sorter 1: "); Serial.print(sorterSwitch1Value);
//        Serial.print("\t"); Serial.print(sorter1CMDValue);
//    Serial.print("\tSorter 2: "); Serial.print(sorterSwitch2Value);
//        Serial.print("\t"); Serial.println(sorter2CMDValue);

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
        } else if (!getPaused()) {
            ledState = LOW;
        }
        digitalWrite(LIGHT_OUT_PIN, ledState);
        ledCounter = 0;
    }
}
