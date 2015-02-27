// Put professional header here
// Include some type of copyright

Encoder sorterEncoder(18, 19);  // TODO(Eric): Redo with actual system
const double sorterKp = 0.1;
const int sorterDeadband = 5;  // Allowable slop

// TODO(Eric): Redo with actual system
const int sorterSlotPositions[NUM_SORTER_SLOTS] =
    {0, 410, 820, 1230, 1640, 2050, 2460, 2870, 3280, 3690};


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


// TODO(Eric): put in a timer loop?
void commandSorter(Servo servo, int slot) {
    if (slot >= 0 && slot < NUM_SORTER_SLOTS) {
        int currentPosition = sorterEncoder.read();
        int sortError = sorterSlotPositions[slot] - currentPosition;
        if (abs(sortError) > sorterDeadband) {
            servo.write(calcServoCmdFromDesiredVelocity(sorterKp * sortError));
        }
    }
}
