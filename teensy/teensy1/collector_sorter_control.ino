// Put professional header here
// Include some type of copyright

const int sorterDeadband = 15;  // Allowable slop

// TODO(Eric): Redo with actual system
const int sorterSlotPositions[NUM_SORTER_SLOTS] =
    {0, 102, 204, 306, 409, 511, 613, 716, 818, 920};


// Drives collector in for 1, out for -1, and stops for 0
void commandCollector(int cmd) {
    if (cmd > 0) {
        digitalWrite(COLLECTOR_OUT_A, HIGH);
        digitalWrite(COLLECTOR_OUT_B, LOW);
    } else if (cmd < 0) {
        digitalWrite(COLLECTOR_OUT_A, LOW);
        digitalWrite(COLLECTOR_OUT_B, HIGH);
    } else {
        digitalWrite(COLLECTOR_OUT_A, LOW);
        digitalWrite(COLLECTOR_OUT_B, LOW);
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
        int currentPosition = analogRead(SORTER_POT_PIN_IN);
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
