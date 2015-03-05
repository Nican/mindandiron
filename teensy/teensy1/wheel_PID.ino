// Put professional header here
// Include some type of copyright

int leftVelocityGoal  = 0;
int rightVelocityGoal = 0;
const double Kp           = 0;  // Proportional constant for wheel velocity
const double Ki           = -0.001;  // Integral constant for wheel velocity
const int integralBounds  = 100 * abs(1 / Ki);  // To prevent wild reactions
int leftIntegralError     = 0;
int rightIntegralError    = 0;
int leftAutoWheelCmd      = 0;
int rightAutoWheelCmd     = 0;


void calculateWheelPIDControl() {
    int leftError = leftVelocityGoal - getLeftVelocity();
    if (!(abs(leftIntegralError + leftError) > integralBounds)) {
        leftIntegralError += leftError;
    }
    leftAutoWheelCmd = calcServoCmdFromDesiredVelocity(Kp * leftError +
                                                       Ki * leftIntegralError);
    int rightError = rightVelocityGoal - getRightVelocity();
    if (!(abs(rightIntegralError + rightError) > integralBounds)) {
        rightIntegralError += rightError;
    }
    rightAutoWheelCmd = calcServoCmdFromDesiredVelocity(Kp * rightError +
                                                        Ki * rightIntegralError);
}

void resetWheelIntegralError() {
    leftIntegralError = 0;
    rightIntegralError = 0;
}

void setLeftWheelPIDSetpoint(int velocity) {
    leftVelocityGoal = velocity;
}

void setRightWheelPIDSetpoint(int velocity) {
    rightVelocityGoal = velocity;
}

int getLeftAutoWheelCmd() {
    return leftAutoWheelCmd;
}

int getRightAutoWheelCmd() {
    return rightAutoWheelCmd;
}
