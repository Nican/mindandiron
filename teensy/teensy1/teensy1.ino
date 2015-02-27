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


// WHEEL ENCODER SETUP
Encoder wheelLeft(17, 16);
Encoder wheelRight(14, 15);
const int encoderHistLength = 5;
volatile long oldLeft[encoderHistLength] = {-999, -999, -999, -999, -999};
volatile long oldRight[encoderHistLength] = {-999, -999, -999, -999, -999};
double leftVelocity = 0;
double rightVelocity = 0;

// DRIVE SETUP
int isSystemAuto = 0;
Servo servoLeft;
Servo servoRight;
Servo servoSorter;
int lastLeftCmd;
int lastRightCmd;

// WHEEL PID SETUP
const double Kp = 0;  // Proportional PI constant for wheel velocity
const double Ki = -0.001;  // Integral PI constant for wheel velocity
int leftVelocitySetpoint = 0;   // Velocity setpoint from the computer
int rightVelocitySetpoint = 0;  // Velocity setpoint from the computer
const int maxWheelSpeed = 130;
const int integralBounds = 100 * abs(1 / Ki);  // To prevent wild reactions
volatile int leftIntegralError = 0;
volatile int rightIntegralError = 0;
volatile int leftAutoWheelCmd = 0; 
volatile int rightAutoWheelCmd = 0;

// COLLECTOR AND SORTER SETUP
int collectorAutoCmd = 0;
int sorterAutoSlot = 0;  // 10 discrete slots (position control)

// ACCELEROMETER SETUP
int accelerometerAxes[3] = {0, 0, 0};  // Stores {x, y, z} accelerometer data


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
    readAccelerometer(accelerometerAxes);  // Updates accelerometerAxes
    readComputerCommands(&leftVelocitySetpoint,
                         &rightVelocitySetpoint,
                         &collectorAutoCmd,
                         &sorterAutoSlot);

    if (switchOn(AUTO_SWITCH_IN)) {
        if (!isSystemAuto) {
            leftIntegralError = 0;  // Resets integral when starting AUTO
            rightIntegralError = 0;  // Resets integral when starting AUTO
            isSystemAuto = 1;
        }
        else {
          servoLeft.write(leftAutoWheelCmd);
          servoRight.write(rightAutoWheelCmd);
          commandCollector(collectorAutoCmd);
          // commandSorter(servoSorter, sorterAutoSlot);  Uncomment when wired up
        }
    }
    else {
        if (isSystemAuto) {
            isSystemAuto = 0;
        }
        lastLeftCmd = passThroughRC(servoLeft, LEFT_CMD_IN, lastLeftCmd);
        lastRightCmd = passThroughRC(servoRight, RIGHT_CMD_IN, lastRightCmd);
    }

    printDataToComputer(leftVelocity, rightVelocity,
                        accelerometerAxes[0],
                        accelerometerAxes[1],
                        accelerometerAxes[2],
                        isSystemAuto);
}


// Averages the last encoder counts and reports velocity
void calculateVelocity(void) {
    for (int i = encoderHistLength - 1; i > 0; i--) {
        oldLeft[i] = oldLeft[i - 1];
        oldRight[i] = oldRight[i - 1];
    }
    oldLeft[0] = wheelLeft.read();;
    oldRight[0] = wheelRight.read();;

    int leftSum = 0;
    int rightSum = 0;
    for (int i = 0; i < (encoderHistLength - 2); i++) {
        leftSum += oldLeft[i] - oldLeft[i + 1];
        rightSum += oldRight[i] - oldRight[i + 1];
    }
    leftVelocity = leftSum / (encoderHistLength - 1);
    rightVelocity = rightSum / (encoderHistLength - 1);
}


void calculateWheelPIDControl() {
    int leftError = leftVelocitySetpoint - leftVelocity;
    if (!(abs(leftIntegralError + leftError) > integralBounds)) {
        leftIntegralError += leftError;
    }
    leftAutoWheelCmd = calcServoCmdFromDesiredVelocity(Kp * leftError +
                                                       Ki * leftIntegralError);
    int rightError = rightVelocitySetpoint - rightVelocity;
    if (!(abs(rightIntegralError + rightError) > integralBounds)) {
        rightIntegralError += rightError;
    }
    rightAutoWheelCmd = calcServoCmdFromDesiredVelocity(Kp * rightError +
                                                        Ki * rightIntegralError);
}
