// Put professional header here
// Include some type of copyright


/*
 * Teensy 1
 *     Reads the encoders on the wheels and prints position to serial
 *     Reads the RC receiver and decides to commands the motors directly or
 *          use the control signal from the cumputer based on the override line.
 *          Also kills the drive relay if the pause button is flipped
 *
 * Encoder code based on the Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 */


#include <Encoder.h>
#include <Servo.h>
#include <TimerOne.h>
#include <TimerThree.h>

#include "collector_sorter_includes.h"
#include "rc_includes.h"


#define VELOCITY_PERIOD_MICRO 10000  // Period of vel calc, microseconds
#define PID_PERIOD_MICRO      10000  // Period of wheel PID, microseconds
#define WHEEL_TICKS_PER_REV  23330  // Determined experimentally for encoder


// DRIVE SETUP
int isSystemAuto = 0;
Servo servoLeft;
Servo servoRight;
int lastLeftCmd;
int lastRightCmd;

// WHEEL PID SETUP
int leftVelocitySetpoint = 0;   // Velocity setpoint from the computer
int rightVelocitySetpoint = 0;  // Velocity setpoint from the computer

// COLLECTOR AND SORTER SETUP
Servo servoSorter;
int collectorAutoCmd = 0;
int sorterAutoSlot = 0;  // 10 discrete slots (position control)


void setup() {
    Serial.begin(9600);

    // DRIVE SETUP
    pinMode(LEFT_CMD_IN, INPUT);
    pinMode(RIGHT_CMD_IN, INPUT);
    pinMode(AUTO_SWITCH_IN, INPUT);
    pinMode(COLLECTOR_OUT_A, OUTPUT);
    pinMode(COLLECTOR_OUT_B, OUTPUT);
    commandCollector(0);  // 0 stops the collector (drives both pins LOW)
    servoLeft.attach(LEFT_OUT_PIN); servoLeft.write(MIN_SERVO_SPEED);
    lastLeftCmd = MIN_SERVO_SPEED;
    servoRight.attach(RIGHT_OUT_PIN); servoRight.write(MIN_SERVO_SPEED);
    lastRightCmd = MIN_SERVO_SPEED;
    servoSorter.attach(SORTER_OUT_PIN); servoSorter.write(MIN_SERVO_SPEED);
    Timer1.initialize(VELOCITY_PERIOD_MICRO);
    Timer1.attachInterrupt(calculateVelocity);

    // WHEEL PID SETUP
    Timer3.initialize(PID_PERIOD_MICRO);
    Timer3.attachInterrupt(calculateWheelPIDControl);
}


void loop() {
    readAccelerometer();  // Updates accelerometerAxes
    readComputerCommands(&leftVelocitySetpoint,
                         &rightVelocitySetpoint,
                         &collectorAutoCmd,
                         &sorterAutoSlot);
    setLeftWheelPIDSetpoint(leftVelocitySetpoint);
    setRightWheelPIDSetpoint(rightVelocitySetpoint);

    if (switchOn(AUTO_SWITCH_IN)) {
        if (!isSystemAuto) {
            resetWheelIntegralError();  // Resets integral when starting AUTO
            isSystemAuto = 1;
        } else {
          servoLeft.write(getLeftAutoWheelCmd());
          servoRight.write(getRightAutoWheelCmd());
          commandCollector(collectorAutoCmd);
          // commandSorter(servoSorter, sorterAutoSlot);  Uncomment when wired
        }
    } else {
        if (isSystemAuto) {
            isSystemAuto = 0;
        }
        lastLeftCmd = passThroughRC(servoLeft, LEFT_CMD_IN, lastLeftCmd);
        lastRightCmd = passThroughRC(servoRight, RIGHT_CMD_IN, lastRightCmd);
    }

    printDataToComputer(getLeftPosition(), getRightPosition(),
                        getAX(), getAY(), getAZ(),
                        isSystemAuto);
}
