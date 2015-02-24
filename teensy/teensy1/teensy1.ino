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
#define PID_PERIOD_MICRO      10000  // Period of wheel PID, microseconds
#define LEFT_CMD_IN      11  // Pin 2 on the receiver
#define RIGHT_CMD_IN     12  // Pin 3 on the receiver
#define AUTO_SWITCH_IN   10  // Pin 5 on the receiver
#define LEFT_OUT_PIN     3   // PWM control on the left wheel
#define RIGHT_OUT_PIN    2   // PWM control on the right wheel
#define SORTER_OUT_PIN   4   // PWM control on the sorter
#define COLLECTOR_OUT_A  5   // Goes to one side of the collector
#define COLLECTOR_OUT_B  6   // Goes to one side of the collector
#define WHEEL_TICKS_PER_REV  23330  // Determined experimentally for encoder
#define SORTER_TICKS_PER_REV 4095   // Determined experimentally for encoder
#define NUM_SORTER_SLOTS 10  // Number of slots in the sorter


// WHEEL ENCODER SETUP
Encoder wheelLeft(17, 16);
Encoder wheelRight(14, 15);
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
Encoder sorterEncoder(18, 19);  // NOT YET WIRED. TODO: Redo with actual system
int collectorAutoCmd = 0;
int sorterAutoSlot = 0;  // 10 discrete slots (position control)
const double sorterKp = 0.1;
const int sorterDeadband = 5;  // Allowable slop
const int sorterSlotPositions[NUM_SORTER_SLOTS] =
    {0, 410, 820, 1230, 1640, 2050, 2460, 2870, 3280, 3690};  // TODO: Redo with actual system


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
    readAccelerometer();  // Updates the accelerometerAxes variable
    readComputerCommands();  // Updates leftVelocitySetpoint
                             //         rightVelocitySetpoint
                             //         collectorAutoCmd

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
          // commandSorter(sorterAutoSlot);  Uncomment when wired up
        }
    }
    else {
        if (isSystemAuto) {
            isSystemAuto = 0;
        }
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


void readComputerCommands() {
    char incomingByte;
    int tabCounter = 0;
    int leftVelocityCmd = 0;
    int rightVelocityCmd = 0;
    int collectorCmd = 0;
    int sorterCmd = 0;
    while (Serial.available()) {
        incomingByte = Serial.read();
        if (incomingByte == '\t') {
            if (tabCounter == 0) {
                leftVelocityCmd = Serial.parseInt();  // 0 when passed non-int
            }
            else if (tabCounter == 2) {
                rightVelocityCmd = Serial.parseInt();
            }
            else if (tabCounter == 4) {
                collectorCmd = Serial.parseInt();
            }
            else if (tabCounter == 6) {
                sorterCmd = Serial.parseInt();
            }
            tabCounter++;
        }
    }
    if (tabCounter == 8) {
        leftVelocitySetpoint = boundVelocity(leftVelocityCmd);
        rightVelocitySetpoint = boundVelocity(rightVelocityCmd);
        collectorAutoCmd = boundCollectorSignal(collectorCmd);
        sorterAutoSlot = boundSorterSignal(sorterCmd);
    }
}


int boundVelocity(int cmd) {
    // Decided to treat out-of-bound commands as illegitimate to prevent
    // runaways. Could implement this as a clamp function as needed
    if (abs(cmd) > maxWheelSpeed) { return 0; }
    else { return cmd; }
}


int boundCollectorSignal(int cmd) {
    // Decided to treat out-of-bound commands as illegitimate. Could
    // implement this as a clamp function as needed
    if (abs(cmd) > 1) { return 0; }
    else { return cmd; }
}


int boundSorterSignal(int cmd) {
    // Decided to treat out-of-bound commands as illegitimate. Could
    // implement this as a clamp function as needed
    if (cmd >= 0 && < NUM_SORTER_SLOTS) { return cmd; }
    else { return 0; }
}


// Drives collector in for 1, out for -1, and stops for 0
void commandCollector(int cmd) {
    if (cmd > 0) {
        digitalWrite(COLLECTOR_OUT_A, HIGH);
        digitalWrite(COLLECTOR_OUT_B, LOW);
    }
    else if (cmd < 0) {
        digitalWrite(COLLECTOR_OUT_A, LOW);
        digitalWrite(COLLECTOR_OUT_B, HIGH);
    }
    else {
        digitalWrite(COLLECTOR_OUT_A, LOW);
        digitalWrite(COLLECTOR_OUT_B, LOW);
    }
}


// TODO: put in a timer loop?
void commandSorter(int slot) {
    if (slot >= 0 && < NUM_SORTER_SLOTS) {
        int currentPosition = sorterEncoder.read()
        int sortError = sorterSlotPositions[slot] - currentPosition;
        if (abs(sortError) > sorterDeadband) {
            servoSorter.write(
                calcServoCmdFromDesiredVelocity(sorterKp * sortError));
        }
    }
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
    if (!(abs(rightIntegralError + rightError) > integralBounds)) {
        rightIntegralError += rightError;
    }
    rightAutoWheelCmd = calcServoCmdFromDesiredVelocity(Kp * rightError +
                                                        Ki * rightIntegralError);
}


// Translates the commanded velocity (negative and positive around 0) into
// Servo commands
int calcServoCmdFromDesiredVelocity(int cmd) {
    cmd += MIN_SERVO_SPEED;
    if (cmd > MAX_SERVO_SPEED) { cmd = MAX_SERVO_SPEED; }
    if (cmd < (2 * MIN_SERVO_SPEED - MAX_SERVO_SPEED)) { cmd = MIN_SERVO_SPEED; }
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
