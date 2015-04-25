// Put professional header here
// Include some type of copyright


bool useVelocityPID = 1;                   // 1 for velocity control, 0 for position control

int leftVelocityGoal  = 0;
int rightVelocityGoal = 0;
const double velocityKp          = 0;  // Proportional constant for wheel velocity
const double velocityKi          = -0.001;  // Integral constant for wheel velocity
const int velocityIntegralBounds = 100 * abs(1 / velocityKi);  // To prevent wild reactions
int leftVelocityIntegralError    = 0;
int rightVelocityIntegralError   = 0;

int leftPositionGoal  = 0;
int rightPositionGoal = 0;
const double positionKp          = -0.0005;  // Proportional constant for wheel position
const double positionKi          = -0;  // Integral constant for wheel position
const int positionIntegralBounds = 100 * abs(1 / positionKi);  // To prevent wild reactions
int leftPositionIntegralError    = 0;
int rightPositionIntegralError   = 0;

int leftAutoWheelCmd      = 0;
int rightAutoWheelCmd     = 0;

// This constant scalar converts the units
// (m / s) * (1 rev / 2 PI R m) * (ticks / rev) * (1 s / samples per s)
const float mPerSToTicks = (1.0 / (WHEEL_RADIUS * 2.0 * PI)) *
                           WHEEL_TICKS_PER_REV *
                           (VELOCITY_PERIOD_MICRO / 1000000.0);


void calculateVelocityPIDControl() {
    int leftError;
    int rightError;

    if (useVelocityPID) {
        leftError = leftVelocityGoal - getLeftVelocity();
        if (!(abs(leftVelocityIntegralError + leftError) > velocityIntegralBounds)) {
            leftVelocityIntegralError += leftError;
        }
        leftAutoWheelCmd = calcServoCmdFromDesiredVelocity(velocityKp * leftError +
                                                           velocityKi * leftVelocityIntegralError);
        rightError = rightVelocityGoal - getRightVelocity();
        if (!(abs(rightVelocityIntegralError + rightError) > velocityIntegralBounds)) {
            rightVelocityIntegralError += rightError;
        }
        rightAutoWheelCmd = calcServoCmdFromDesiredVelocity(velocityKp * rightError +
                                                            velocityKi * rightVelocityIntegralError);
        resetPositionIntegralError();
    } else {
        leftError = leftPositionGoal - getLeftPosition();  // In units of ticks
        if (!(abs(leftPositionIntegralError + leftError) > positionIntegralBounds)) {
            leftPositionIntegralError += leftError;
        }
        leftAutoWheelCmd = calcServoCmdFromDesiredVelocity((int) (positionKp * leftError +
                                                            positionKi * leftPositionIntegralError));
        rightError = rightPositionGoal - getRightPosition();  // In units of ticks
        if (!(abs(rightPositionIntegralError + rightError) > positionIntegralBounds)) {
            rightPositionIntegralError += rightError;
        }
        rightAutoWheelCmd = calcServoCmdFromDesiredVelocity((int) (positionKp * rightError +
                                                            positionKi * rightPositionIntegralError));
        resetVelocityIntegralError();
    }
}

void resetWheelIntegralError() {
    resetVelocityIntegralError();
    resetPositionIntegralError();
}

void resetVelocityIntegralError() {
    leftVelocityIntegralError = 0;
    rightVelocityIntegralError = 0;
}

void resetPositionIntegralError() {
    leftPositionIntegralError = 0;
    rightPositionIntegralError = 0;
}

// Velocity comes in as m/s
void setLeftWheelVelocityPIDSetpoint(float velocity) {
    leftVelocityGoal = mPerSVelocityToTicks(velocity);
}

// Velocity comes in as m/s
void setRightWheelVelocityPIDSetpoint(float velocity) {
    rightVelocityGoal = mPerSVelocityToTicks(velocity);
}

// Tick goal is absolute ticks, as tracked by the encoder
void setLeftWheelPositionPIDSetpoint(int tickGoal) {
    leftPositionGoal = tickGoal;
}

// Tick goal is absolute ticks, as tracked by the encoder
void setRightWheelPositionPIDSetpoint(int tickGoal) {
    rightPositionGoal = tickGoal;
}

void setUseVelocityPID(int useVelocity){
    useVelocityPID = useVelocity;
}

// Velocity comes in as m/s, comes out as ticks per time unit
int mPerSVelocityToTicks(float velocity) {
    return (int) (velocity * mPerSToTicks);
}

int getLeftAutoWheelCmd() {
    return leftAutoWheelCmd;
}

int getRightAutoWheelCmd() {
    return rightAutoWheelCmd;
}
