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

#include <TimerOne.h>
#include <Encoder.h>
#include <Servo.h>


#define VELOCITY_PERIOD_MICRO 10000  // Period of vel calc, microseconds
#define LEFT_CMD_IN      18  // Pin 2 on the receiver
#define RIGHT_CMD_IN     19  // Pin 3 on the receiver
#define AUTO_SWITCH_IN   20  // Pin 5 on the receiver
#define PAUSE_SWITCH_IN  21  // Pin 6 on the receiver
#define LEFT_OUT_PIN     23
#define RIGHT_OUT_PIN    22
#define PAUSE_SWITCH_OUT 13  // Goes to the switch nMOS gate


int isSystemPaused = 0;
int isSystemAuto = 0;

// ENCODER SETUP
Encoder wheelLeft(14, 15);
Encoder wheelRight(16, 17);
const int ticksPerRev = 23330;  // Determined experimentally
const int encoderHistLength = 5;
volatile long oldLeft[encoderHistLength] = {-999, -999, -999, -999, -999};
volatile long oldRight[encoderHistLength] = {-999, -999, -999, -999, -999};
volatile int leftVelocity = 0;
volatile int rightVelocity = 0;

// DRIVE SETUP
Servo servoLeft;
Servo servoRight;
int lastLeftCmd;
int lastRightCmd;

const int MAX_WHEEL_SPEED = 115;
const int MIN_WHEEL_SPEED = 95;
const int MAX_SIGNAL = 1950;
const int MID_SIGNAL = 1525;
const int MIN_SIGNAL = 1100;
const int IN_DEADBAND = 100;
const int OUT_DEADBAND = 2;
const float INTERP_SLOPE = (MAX_WHEEL_SPEED - MIN_WHEEL_SPEED) /
                           (float)(MAX_SIGNAL - MID_SIGNAL);
const float INTERP_OFFSET = MIN_WHEEL_SPEED - (MID_SIGNAL * INTERP_SLOPE);

void setup() {
  Timer1.initialize(VELOCITY_PERIOD_MICRO);
  Timer1.attachInterrupt(calculateVelocity);
  Serial.begin(9600);
  pinMode(LEFT_CMD_IN, INPUT);
  pinMode(RIGHT_CMD_IN, INPUT);
  pinMode(AUTO_SWITCH_IN, INPUT);
  pinMode(PAUSE_SWITCH_IN, INPUT);
  pinMode(PAUSE_SWITCH_OUT, OUTPUT);
  servoLeft.attach(LEFT_OUT_PIN); servoLeft.write(MIN_WHEEL_SPEED);
  lastLeftCmd = MIN_WHEEL_SPEED;
  servoRight.attach(RIGHT_OUT_PIN); servoRight.write(MIN_WHEEL_SPEED);
  lastRightCmd = MIN_WHEEL_SPEED;
}

void loop() {
  // DEAL WITH PAUSING
  if (switchOn(PAUSE_SWITCH_IN)) {
    if (!isSystemPaused) {
      isSystemPaused = 1;
      digitalWrite(PAUSE_SWITCH_OUT, LOW);
    }
  }
  else {
    if (isSystemPaused) {
      isSystemPaused = 0;
      digitalWrite(PAUSE_SWITCH_OUT, HIGH);
    }
  }

  // DEAL WITH COMPUTER COMMANDS
  if (switchOn(AUTO_SWITCH_IN)) {
    // TODO: Fill with computer commands
    if (!isSystemAuto) {
      servoLeft.write(MIN_WHEEL_SPEED);
      servoRight.write(MIN_WHEEL_SPEED);
      isSystemAuto = 1;
    }
  }
  else {
    lastLeftCmd = passThroughRC(servoLeft, LEFT_CMD_IN, lastLeftCmd);
    lastRightCmd = passThroughRC(servoRight, RIGHT_CMD_IN, lastRightCmd);
    if (isSystemAuto) {
      isSystemAuto = 0;
    }
  }

  printDataToComputer();
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

// Checks whether an RC switch is on/off and returns a boolean
bool switchOn(int switchPin) {
  int duration = pulseIn(switchPin, HIGH);
  if (duration > MID_SIGNAL) {
    return true;
  }
  return false;
}

// Interpolates the input RC values as servo values for the motors
int passThroughRC(Servo servo, int pin, int lastCmd) {
  int duration = pulseIn(pin, HIGH);
  int cmd;

  // Brakes the robot when in the deadband
  if (within(duration, MID_SIGNAL, IN_DEADBAND)) {
    cmd = MIN_WHEEL_SPEED;
  }
  else {
    // Adjusts the duration to prevent a jump on either side of the deadband
    if (duration > MID_SIGNAL) { duration -= IN_DEADBAND; }
    else { duration += IN_DEADBAND; }
    cmd = INTERP_SLOPE * duration + INTERP_OFFSET;
  }
  
  if (within(cmd, MIN_WHEEL_SPEED, (MAX_WHEEL_SPEED - MIN_WHEEL_SPEED))) {
    servo.write(cmd);
    return cmd;
  }
  return lastCmd;
}

// Checks whether the given value is within a given offset from a goal value
bool within(int value, int goal, int offset) {
  if ((goal + offset > value) && (goal - offset < value)) {
    return true;
  }
  return false;
}

void printDataToComputer() {
  Serial.print("LVEL,"); Serial.print(leftVelocity);
  Serial.print(",RVEL,"); Serial.print(rightVelocity);
  Serial.print(",PAUSE,"); Serial.print(isSystemPaused);
  Serial.print(",AUTO,"); Serial.println(isSystemAuto);
}

