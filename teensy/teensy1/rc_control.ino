// Put professional header here
// Include some type of copyright

const float INTERP_SLOPE = (MAX_SERVO_SPEED - MIN_SERVO_SPEED) /
                           (float)(MAX_SIGNAL - MID_SIGNAL);
const float INTERP_OFFSET = MIN_SERVO_SPEED - (MID_SIGNAL * INTERP_SLOPE);


// Translates the commanded velocity (negative and positive around 0) into
// Servo commands
int calcServoCmdFromDesiredVelocity(int cmd) {
    cmd += MIN_SERVO_SPEED;
    if (cmd > MAX_SERVO_SPEED) {
        cmd = MAX_SERVO_SPEED;
    }
    if (cmd < (2 * MIN_SERVO_SPEED - MAX_SERVO_SPEED)) {
        cmd = 2 * MIN_SERVO_SPEED - MAX_SERVO_SPEED;
    }
    return cmd;
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
    } else {
        // Adjusts the duration to prevent a jump on either side of the deadband
        if (duration > MID_SIGNAL) {
            duration -= IN_DEADBAND;
        } else {
            duration += IN_DEADBAND;
        }
        cmd = INTERP_SLOPE * duration + INTERP_OFFSET;
    }

    if (within(cmd, MIN_SERVO_SPEED, (MAX_SERVO_SPEED - MIN_SERVO_SPEED))) {
        servo.write(cmd);
        return cmd;
    } else {
        return lastCmd;
    }
}


void onOffRCControl(Servo servo, int pin) {
    int duration = pulseIn(pin, HIGH);
    int cmd;

    // Brakes the motor when in the deadband
    if (within(duration, MID_SIGNAL, IN_DEADBAND) || duration < MID_SIGNAL) {
        cmd = MIN_SERVO_SPEED;
    } else {
        cmd = 180;
    }

    servo.write(cmd);
}


// Controls the direction and speed of the collector
void roughlyControlWithRC(Servo servo, int signalPin) {
    int duration = pulseIn(signalPin, HIGH);
    int direction;
    int speed;

    // Adjusts the direction as appropriate
    if (duration < MID_SIGNAL) {
        direction = 1;
    } else {
        direction = -1;
    }

    // Brakes the collector when in the deadband
    if (within(duration, MID_SIGNAL, IN_DEADBAND)) {
        speed = 0;
    } else if (within(duration, MID_SIGNAL, 2*IN_DEADBAND)) {
        speed = SERVO_OUTPUT_SMALL_DELTA;
    } else if (duration < MAX_SIGNAL && duration > MIN_SIGNAL) {
        speed = SERVO_OUTPUT_LARGE_DELTA;
    }

    commandCollectorSimple(servo, direction*speed);
}


// Checks whether the given value is within a given offset from a goal value
bool within(int value, int goal, int offset) {
    if ((goal + offset > value) && (goal - offset < value)) {
        return true;
    }
    return false;
}
