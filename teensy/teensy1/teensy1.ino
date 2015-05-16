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
#include "wheel_PID.h"


// DRIVE SETUP
int isSystemAuto = 0;
Servo servoLeft;
Servo servoRight;
int lastLeftCmd;
int lastRightCmd;

// WHEEL PID SETUP
float leftVelocitySetpoint = 0;   // Velocity setpoint from the computer, m/s
float rightVelocitySetpoint = 0;  // Velocity setpoint from the computer, m/s
int leftPositionSetpoint = 0;     // Position setpoint from the computer, global ticks
int rightPositionSetpoint = 0;    // Position setpoint from the computer, global ticks
int useVelocityControl = 1;       // 1 to use velocity PID, 0 to use position PID (unfinished)

// COLLECTOR AND SORTER SETUP
Servo servoSorter;
Servo servoCollector;
int lastSorterCmd = 0;
int sorterAutoSlot = 0;  // 10 discrete slots (position control)
int collectorAutoCmd = 0;

// TURN ON LIGHT WHEN ON
int ledPin = 13;


void setup() {
    Serial.begin(9600);

    // DRIVE SETUP
    pinMode(LEFT_CMD_IN, INPUT);
    pinMode(RIGHT_CMD_IN, INPUT);
    pinMode(COLLECTOR_CMD_IN, INPUT);
    pinMode(SORTER_CMD_IN, INPUT);
    pinMode(SORTER_MAGNET_PIN, INPUT);
    pinMode(AUTO_SWITCH_IN, INPUT);
    pinMode(AUTO_SWITCH_OUT, OUTPUT);
    pinMode(COLLECTOR_OUT_PIN, OUTPUT);
    servoLeft.attach(LEFT_OUT_PIN); servoLeft.write(MIN_SERVO_SPEED);
    lastLeftCmd = MIN_SERVO_SPEED;
    servoRight.attach(RIGHT_OUT_PIN); servoRight.write(MIN_SERVO_SPEED);
    lastRightCmd = MIN_SERVO_SPEED;
    Timer1.initialize(VELOCITY_PERIOD_MICRO);
    Timer1.attachInterrupt(calculateVelocity);

    // WHEEL PID SETUP
    // COMMENTED OUT, BUNDLED IN WITH calculateVelocity
    // Timer3.initialize(PID_PERIOD_MICRO);
    // Timer3.attachInterrupt(calculateVelocityPIDControl);

    // SORTER AND COLLECTOR SETUP
    servoSorter.attach(SORTER_OUT_PIN);
    servoSorter.write(MIN_SERVO_SPEED);
    lastSorterCmd = MIN_SERVO_SPEED;
    servoCollector.attach(COLLECTOR_OUT_PIN);
    servoCollector.write(MIN_SERVO_SPEED);
    Timer3.initialize(MAGNET_POLL_PERIOD_MICRO);
    Timer3.attachInterrupt(checkSorterSlotChange);

    // TURN ON LIGHT
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
}


void loop() {
    readAccelerometer();  // Updates accelerometerAxes
    readComputerCommands(&leftVelocitySetpoint,
                         &rightVelocitySetpoint,
                         &leftPositionSetpoint,
                         &rightPositionSetpoint,
                         &useVelocityControl,
                         &collectorAutoCmd,
                         &sorterAutoSlot);
    setLeftWheelVelocityPIDSetpoint(leftVelocitySetpoint);
    setRightWheelVelocityPIDSetpoint(rightVelocitySetpoint);
//    setLeftWheelPositionPIDSetpoint(leftPositionSetpoint);
//    setRightWheelPositionPIDSetpoint(rightPositionSetpoint);
    setUseVelocityPID(useVelocityControl);

    if (switchOn(AUTO_SWITCH_IN)) {
        if (!isSystemAuto) {
            resetWheelIntegralError();  // Resets integral when starting AUTO
            isSystemAuto = 1;
            digitalWrite(AUTO_SWITCH_OUT, isSystemAuto);
        } else {
          servoLeft.write(getLeftAutoWheelCmd());
          servoRight.write(getRightAutoWheelCmd());
          // commandSorter(servoSorter, sorterAutoSlot);  Uncomment when wired
          if (collectorAutoCmd) {
              commandCollectorSimple(servoCollector, SERVO_OUTPUT_LARGE_DELTA);
          } else {
              commandCollectorFromWheelV(servoCollector, leftVelocitySetpoint,
                                                         rightVelocitySetpoint);
          }
        }
    } else {
        if (isSystemAuto) {
            isSystemAuto = 0;
            digitalWrite(AUTO_SWITCH_OUT, isSystemAuto);
        }
        lastLeftCmd = passThroughRC(servoLeft, LEFT_CMD_IN, lastLeftCmd);
        lastRightCmd = passThroughRC(servoRight, RIGHT_CMD_IN, lastRightCmd);
        lastSorterCmd = passThroughRC(servoSorter, SORTER_CMD_IN, lastSorterCmd);
        roughlyControlWithRC(servoCollector, COLLECTOR_CMD_IN);
    }

    printDataToComputer(getLeftPosition(), getRightPosition(),
                        getLeftVelocity(), getRightVelocity(),
                        getCurrentSlot(), getAX(), getAY(), getAZ(),
                        isSystemAuto);
}
