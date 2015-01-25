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

Encoder wheelLeft(14, 15);
Encoder wheelRight(7, 8);
const int ticksPerRev = 23300;  // Determined experimentally
long positionLeft  = -999;
long positionRight = -999;

#define LEFT_CMD_IN     5  // Pin 2 on the receiver
#define RIGHT_CMD_IN    6  // Pin 3 on the receiver
#define AUTO_SWITCH_IN  7  // Pin 5 on the receiver
#define PAUSE_SWITCH_IN 8  // Pin 6 on the receiver
#define LEFT_OUT_PIN    9
#define RIGHT_OUT_PIN   10
Servo servoLeft;
Servo servoRight;
int lastLeftCmd;
int lastRightCmd;

const int MAX_WHEEL_SPEED = 110;
const int MIN_WHEEL_SPEED = 90;
const int MAX_SIGNAL = 1950;
const int MID_SIGNAL = 1510;
const int MIN_SIGNAL = 1100;
const int IN_DEADBAND = 100;
const int OUT_DEADBAND = 2;
const float INTERP_SLOPE = (MAX_WHEEL_SPEED - MIN_WHEEL_SPEED) / (float)(MAX_SIGNAL - MID_SIGNAL);
const float INTERP_OFFSET = MIN_WHEEL_SPEED - (MID_SIGNAL * INTERP_SLOPE);

void setup() {
  Serial.begin(9600);
  pinMode(LEFT_CMD_IN, INPUT);
  pinMode(RIGHT_CMD_IN, INPUT);
  pinMode(AUTO_SWITCH_IN, INPUT);
  pinMode(PAUSE_SWITCH_IN, INPUT);
  servoLeft.attach(LEFT_OUT_PIN); servoLeft.write(MIN_WHEEL_SPEED); lastLeftCmd = MIN_WHEEL_SPEED;
  servoRight.attach(RIGHT_OUT_PIN); servoRight.write(MIN_WHEEL_SPEED);
  lastRightCmd = MIN_WHEEL_SPEED;
}

void loop() {
  long newLeft, newRight;
  newLeft = wheelLeft.read();
  newRight = wheelRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    checkEncoderReset(newLeft, newRight);
    reportEncoder(newLeft, newRight);
  }
  
  if (switchOn(AUTO_SWITCH_IN)) {
    Serial.println("Auto is on!");  // TODO: Remove once tested
  }
  else {
    lastLeftCmd = passThroughRC(servoLeft, LEFT_CMD_IN, lastLeftCmd);
    lastRightCmd = passThroughRC(servoRight, RIGHT_CMD_IN, lastRightCmd);
    Serial.print("Left motor set to "); Serial.println(lastLeftCmd);
    Serial.print("Right motor set to "); Serial.println(lastRightCmd);
    Serial.println();
  }

  if (switchOn(PAUSE_SWITCH_IN)) {
    Serial.println("Pause is on!");  // TODO: Remove once tested
  }
}

// Checks whether the encoders have gone 360 deg and resets encoders if so
void checkEncoderReset(int newLeft, int newRight) {
  if (abs(newLeft) >= ticksPerRev) {
    wheelLeft.write(newLeft % ticksPerRev);
  }
  if (abs(newRight) >= ticksPerRev) {
    wheelRight.write(newRight % ticksPerRev);
  }
}

// Prints the encoder data for left and right wheels, resets the stored angle
void reportEncoder(int newLeft, int newRight) {
  Serial.print("Left = ");    Serial.print(newLeft);
  Serial.print(", Right = "); Serial.println(newRight);
  positionLeft = newLeft;
  positionRight = newRight;
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
  if (within(duration, MID_SIGNAL, IN_DEADBAND)) {
    cmd = MIN_WHEEL_SPEED;
  }
  else {
    cmd = INTERP_SLOPE * duration + INTERP_OFFSET;
  }
  
  if (within(cmd, MIN_WHEEL_SPEED, (MAX_WHEEL_SPEED - MIN_WHEEL_SPEED)) &&
      !within(cmd, lastCmd, OUT_DEADBAND)) {
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
