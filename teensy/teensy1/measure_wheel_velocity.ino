// Put professional header here
// Include some type of copyright

Encoder wheelLeft(9, 10);
Encoder wheelRight(12, 11);
const int encoderHistLength = 5;
volatile int32_t oldLeft[encoderHistLength] = {-999, -999, -999, -999, -999};
volatile int32_t oldRight[encoderHistLength] = {-999, -999, -999, -999, -999};
volatile double leftVelocity;
volatile double rightVelocity;


// Averages the last encoder counts and reports velocity
void calculateVelocity() {
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
    calculateVelocityPIDControl();
}

// Returns in ticks/hundreth of a second
double getLeftVelocity() {
    return leftVelocity;
}

// Returns in ticks/hundreth of a second
double getRightVelocity() {
    return rightVelocity;
}

// Returns in ticks from the start
int getLeftPosition() {
    return wheelLeft.read();
}

// Returns in ticks from the start
int getRightPosition() {
    return wheelRight.read();
}
