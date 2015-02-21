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


#define VELOCITY_PERIOD_MICRO 10000  // Period of vel calc, microseconds
#define PID_PERIOD_MICRO      5000  // Period of wheel PID, microseconds
#define LEFT_CMD_IN      11  // Pin 2 on the receiver
#define RIGHT_CMD_IN     12  // Pin 3 on the receiver
#define AUTO_SWITCH_IN   10  // Pin 5 on the receiver
#define LEFT_OUT_PIN     3   // PWM control on the left wheel
#define RIGHT_OUT_PIN    2   // PWM control on the right wheel
#define SORTER_OUT_PIN   4   // PWM control on the sorter
#define COLLECTOR_OUT_A  5   // Goes to one side of the collector
#define COLLECTOR_OUT_B  6   // Goes to one side of the collector


// ENCODER SETUP
Encoder wheelLeft(17, 16);
Encoder wheelRight(14, 15);
const int ticksPerRev = 23330;  // Determined experimentally
const int encoderHistLength = 5;
volatile long oldLeft[encoderHistLength] = {-999, -999, -999, -999, -999};
volatile long oldRight[encoderHistLength] = {-999, -999, -999, -999, -999};
double leftVelocity = 0;
double rightVelocity = 0;

// ACCELEROMETER SETUP
int accelerometerAxes[3] = {0, 0, 0};
const int accelerometerXPin = A9;
const int accelerometerYPin = A8;
const int accelerometerZPin = A7;

// DRIVE SETUP
int isSystemAuto = 0;
Servo servoLeft;
Servo servoRight;
Servo servoSorter;
int lastLeftCmd;
int lastRightCmd;
const int MAX_SERVO_SPEED = 115;
const int MIN_SERVO_SPEED = 95;
const int MAX_SIGNAL = 1950;
const int MID_SIGNAL = 1525;
const int MIN_SIGNAL = 1100;
const int IN_DEADBAND = 100;  // Gives a deadband around zero for RC stability
const float INTERP_SLOPE = (MAX_SERVO_SPEED - MIN_SERVO_SPEED) /
                           (float)(MAX_SIGNAL - MID_SIGNAL);
const float INTERP_OFFSET = MIN_SERVO_SPEED - (MID_SIGNAL * INTERP_SLOPE);

// WHEEL PID SETUP
const double Kp = 0;  // Proportional PI constant for wheel velocity
const double Ki = -0.0005;  // Integral PI constant for wheel velocity
int leftVelocitySetpoint = 100;   // Velocity setpoint from the computer
int rightVelocitySetpoint = -100;  // Velocity setpoint from the computer
const int integralBounds = 100000;  // To prevent wild reactions
volatile int leftIntegralError = 0;
volatile int rightIntegralError = 0;
volatile int leftAutoWheelCmd = 0; 
volatile int rightAutoWheelCmd = 0;


void setup() {
    Serial.begin(9600);

    // DRIVE SETUP
    pinMode(LEFT_CMD_IN, INPUT);
    pinMode(RIGHT_CMD_IN, INPUT);
    pinMode(AUTO_SWITCH_IN, INPUT);
    pinMode(COLLECTOR_OUT_A, OUTPUT);
    digitalWrite(COLLECTOR_OUT_A, LOW);
    pinMode(COLLECTOR_OUT_B, OUTPUT);
    digitalWrite(COLLECTOR_OUT_B, LOW);
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
    readAccelerometer();  // Updates the accelerometerAxes variable

    // TODO: READ COMPUTER COMMANDS
    // leftComputerWheelCommand = X     // Uncomment when cmds from comp. come
    // rightComputerWheelCommand = X    // Uncomment when cmds from comp. come
    if (switchOn(AUTO_SWITCH_IN)) {
        if (!isSystemAuto) {
            leftIntegralError = 0;  // Resets integral when starting AUTO
            rightIntegralError = 0;  // Resets integral when starting AUTO
            isSystemAuto = 1;
        }
        else {
          // TODO: Fill with computer commands
          servoLeft.write(leftAutoWheelCmd);      // Uncomment when cmds from comp. come
          servoRight.write(rightAutoWheelCmd);    // Uncomment when cmds from comp. come
        }
    }
    else {
        if (isSystemAuto) {
            leftVelocitySetpoint -= 20;
            isSystemAuto = 0;
        }
        Serial.print("leftVelocitySetpoint: "); Serial.println(leftVelocitySetpoint);
        lastLeftCmd = passThroughRC(servoLeft, LEFT_CMD_IN, lastLeftCmd);
        lastRightCmd = passThroughRC(servoRight, RIGHT_CMD_IN, lastRightCmd);
    }

    printDataToComputer();
}


void readAccelerometer() {
    accelerometerAxes[0] = readAccelerometerSingleAxis(accelerometerXPin);
    accelerometerAxes[1] = readAccelerometerSingleAxis(accelerometerYPin);
    accelerometerAxes[2] = readAccelerometerSingleAxis(accelerometerZPin);
}


// TODO: Make more complex with conversions/filtering after testing
int readAccelerometerSingleAxis(int pin) {
    int val = analogRead(pin);
    return val;
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
// Servo value of 95 = Stop, 80 = Forward, 110 = backwards
int passThroughRC(Servo servo, int pin, int lastCmd) {
    int duration = pulseIn(pin, HIGH);
    int cmd;

    // Brakes the robot when in the deadband
    if (within(duration, MID_SIGNAL, IN_DEADBAND)) {
        cmd = MIN_SERVO_SPEED;
    }
    else {
        // Adjusts the duration to prevent a jump on either side of the deadband
        if (duration > MID_SIGNAL) { duration -= IN_DEADBAND; }
        else { duration += IN_DEADBAND; }
        cmd = INTERP_SLOPE * duration + INTERP_OFFSET;
    }
    
    if (within(cmd, MIN_SERVO_SPEED, (MAX_SERVO_SPEED - MIN_SERVO_SPEED))) {
        servo.write(cmd);
        return cmd;
    }
    else {
        return lastCmd;
    }
}


void calculateWheelPIDControl() {
    int leftError = leftVelocitySetpoint - leftVelocity;
    if (!(abs(leftIntegralError + leftError) > integralBounds)) {
        leftIntegralError += leftError;
    }
    leftAutoWheelCmd = calcServoCmdFromDesiredVelocity(Kp * leftError +
                                                       Ki * leftIntegralError);
    int rightError = rightVelocitySetpoint - rightVelocity;
    rightIntegralError += rightError;
    rightAutoWheelCmd = calcServoCmdFromDesiredVelocity(Kp * rightError +
                                                        Ki * rightIntegralError);
}


// Translates the commanded velocity (negative and positive around 0) into
// Servo commands
int calcServoCmdFromDesiredVelocity(int cmd) {
    cmd += MIN_SERVO_SPEED;
    if (cmd > MAX_SERVO_SPEED) { cmd = MAX_SERVO_SPEED; }
    if (cmd < (2 * MIN_SERVO_SPEED - MAX_SERVO_SPEED)) { cmd = MIN_SERVO_SPEED; }
// Serial.print("Commanding motor: "); Serial.println(cmd);
    return cmd;
}


// Checks whether the given value is within a given offset from a goal value
bool within(int value, int goal, int offset) {
    if ((goal + offset > value) && (goal - offset < value)) {
        return true;
    }
    return false;
}


void printDataToComputer() {
    Serial.print("LVEL\t"); Serial.print(leftVelocity);
    Serial.print("\tRVEL\t"); Serial.print(rightVelocity);
    Serial.print("\tAX\t"); Serial.print(accelerometerAxes[0]);
    Serial.print("\tAY\t"); Serial.print(accelerometerAxes[1]);
    Serial.print("\tAZ\t"); Serial.print(accelerometerAxes[2]);
    Serial.print("\tAUTO\t"); Serial.println(isSystemAuto);
}

