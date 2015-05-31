// Put professional header here
// Include some type of copyright


bool useVelocityPID = 1;                   // 1 for velocity control, 0 for position control

int leftVelocityGoal  = 0;
int rightVelocityGoal = 0;
const double velocityKp          = -0;  // Proportional constant for wheel velocity
const double velocityKi          = -0.02;  // Integral constant for wheel velocity
const int velocityIntegralBounds = 100 * abs(1 / velocityKi);  // To prevent wild reactions
int leftVelocityIntegralError    = 0;
int rightVelocityIntegralError   = 0;

// PI position control is not currently working
// int leftPositionGoal  = 0;
// int rightPositionGoal = 0;
// const double positionKp          = -0.0001;  // Proportional constant for wheel position
// const double positionKi          = -0.000001;  // Integral constant for wheel position
// const int positionIntegralBounds = 100 * abs(1 / positionKi);  // To prevent wild reactions
// int leftPositionIntegralError    = 0;
// int rightPositionIntegralError   = 0;

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
        if (abs(leftVelocityGoal) < mPerSVelocityToTicks(PID_CUTOFF_VELOCITY)) {
            leftAutoWheelCmd = calcServoCmdFromDesiredVelocity(0.0);
            leftVelocityIntegralError = 0;
        } else {
            leftError = leftVelocityGoal - getLeftVelocity();
            if (!(abs(leftVelocityIntegralError + leftError) > velocityIntegralBounds)) {
                leftVelocityIntegralError += leftError;
            }
            leftAutoWheelCmd = calcServoCmdFromDesiredVelocity(velocityKp * leftError +
                                                               velocityKi * leftVelocityIntegralError);
        }

        if (abs(rightVelocityGoal) < mPerSVelocityToTicks(PID_CUTOFF_VELOCITY)) {
            rightAutoWheelCmd = calcServoCmdFromDesiredVelocity(0.0);
            leftVelocityIntegralError = 0;
        } else {
            rightError = rightVelocityGoal - getRightVelocity();
            if (!(abs(rightVelocityIntegralError + rightError) > velocityIntegralBounds)) {
                rightVelocityIntegralError += rightError;
            }
            rightAutoWheelCmd = calcServoCmdFromDesiredVelocity(velocityKp * rightError +
                                                                velocityKi * rightVelocityIntegralError);
            // resetPositionIntegralError();
        }
    } else {
        // PI position control is not currently working. Ideas for future
        // Create a one pole tranfer function with c2d
        // Implement a railed proportional control with a rampup and rampdown.
        leftAutoWheelCmd = calcServoCmdFromDesiredVelocity(0);
        rightAutoWheelCmd = calcServoCmdFromDesiredVelocity(0);

        // leftError = leftPositionGoal - getLeftPosition();  // In units of ticks
        // if (!(abs(leftPositionIntegralError + leftError) > positionIntegralBounds)) {
        //     leftPositionIntegralError += leftError;
        // }
        // leftAutoWheelCmd = calcServoCmdFromDesiredVelocity((int) (positionKp * leftError +
        //                                                     positionKi * leftPositionIntegralError));
        // rightError = rightPositionGoal - getRightPosition();  // In units of ticks
        // if (!(abs(rightPositionIntegralError + rightError) > positionIntegralBounds)) {
        //     rightPositionIntegralError += rightError;
        // }
        // rightAutoWheelCmd = calcServoCmdFromDesiredVelocity((int) (positionKp * rightError +
        //                                                     positionKi * rightPositionIntegralError));
        // resetVelocityIntegralError();
    }
}

void resetWheelIntegralError() {
    resetVelocityIntegralError();
//    resetPositionIntegralError();
}

void resetVelocityIntegralError() {
    leftVelocityIntegralError = 0;
    rightVelocityIntegralError = 0;
}

//void resetPositionIntegralError() {
//    leftPositionIntegralError = 0;
//    rightPositionIntegralError = 0;
//}

// Velocity comes in as m/s
void setLeftWheelVelocityPIDSetpoint(float velocity) {
    leftVelocityGoal = mPerSVelocityToTicks(velocity);
}

// Velocity comes in as m/s
void setRightWheelVelocityPIDSetpoint(float velocity) {
    rightVelocityGoal = mPerSVelocityToTicks(velocity);
}

//// Tick goal is absolute ticks, as tracked by the encoder
//void setLeftWheelPositionPIDSetpoint(int tickGoal) {
//    leftPositionGoal = tickGoal;
//}
//
//// Tick goal is absolute ticks, as tracked by the encoder
//void setRightWheelPositionPIDSetpoint(int tickGoal) {
//    rightPositionGoal = tickGoal;
//}

void setUseVelocityPID(int useVelocity){
    useVelocityPID = useVelocity;
}

// Velocity comes in as m/s, comes out as ticks per time unit
int mPerSVelocityToTicks(float velocity) {
    return (int) (velocity * mPerSToTicks);
}

// Returns the current commanded velocity. Returns 0 if there are no recent commands
int getLeftAutoWheelCmd() {
    long currentTime = millis();
    if (currentTime < (timeOfLastComputerCommand + allowableComputerCommandLag)) {
        return leftAutoWheelCmd;
    } else {
        return calcServoCmdFromDesiredVelocity(0);
    }
}

// Returns the current commanded velocity. Returns 0 if there are no recent commands
int getRightAutoWheelCmd() {
    long currentTime = millis();
    if (currentTime < (timeOfLastComputerCommand + allowableComputerCommandLag)) {
        return rightAutoWheelCmd;
    } else {
        return calcServoCmdFromDesiredVelocity(0);
    }
}
