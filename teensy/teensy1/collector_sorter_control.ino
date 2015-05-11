// Put professional header here
// Include some type of copyright

const int sorterDeadband = 15;  // Allowable slop

// TODO(Eric): Redo with actual system
const int sorterSlotPositions[NUM_SORTER_SLOTS] =
    {0, 102, 204, 306, 409, 511, 613, 716, 818, 920};


// Speed is a servo, speed goes from -85 to 85, recentered around servo 0 (95)
void commandCollector(Servo servo, int speed) {
    if (abs(speed) <= SERVO_OUTPUT_LARGE_DELTA) {
        servo.write(MIN_SERVO_SPEED + speed);
    } else {
        servo.write(MIN_SERVO_SPEED);
    }
}


// Returns center position of desired slot
int getSlotCenterPosition(int slot) {
    int centerPosition;
    if (slot == NUM_SORTER_SLOTS - 1) {
        centerPosition = (sorterSlotPositions[slot] + 1023) / 2;
    } else {
        centerPosition = (sorterSlotPositions[slot] + 
                          sorterSlotPositions[slot+1]) / 2;
    }
    return centerPosition;
}


// TODO(Eric): put in a timer loop?
void commandSorter(Servo servo, int slot) {
    if (slot >= 0 && slot < NUM_SORTER_SLOTS) {
        int currentPosition = 0; //analogRead(SORTER_POT_PIN_IN);
        int sortError = getSlotCenterPosition(slot) - currentPosition;
        if (abs(sortError) > sorterDeadband) {
            if (sortError > 0) {
                servo.write(MIN_SERVO_SPEED + 20);
            } else if (sortError < 0) {
                servo.write(MIN_SERVO_SPEED - 20);
            } else {
                servo.write(MIN_SERVO_SPEED);
            }
        }
    }
}
