// Put professional header here
// Include some type of copyright


volatile int lastMagnetState = HIGH;
volatile int currentSlot = 0;


void checkSorterSlotChange() {
    int currentMagnetState = digitalRead(SORTER_MAGNET_PIN);
    if (lastMagnetState == HIGH and currentMagnetState == LOW) {
        currentSlot++;
    }
    lastMagnetState = currentMagnetState;
}


int getCurrentSlot() {
    return currentSlot;
}


// TODO(Eric): Test
void commandSorter(Servo servo, int slot) {
    if (slot == currentSlot) {
        servo.write(MIN_SERVO_SPEED);
    } else if (slot < currentSlot) {
        servo.write(MIN_SERVO_SPEED + SORTER_SPEED);
    } else {
        servo.write(MIN_SERVO_SPEED - SORTER_SPEED);
    }
}


// Speed is a servo, speed goes from -85 to 85, recentered around servo 0 (95)
void commandCollector(Servo servo, int speed) {
    if (abs(speed) <= SERVO_OUTPUT_LARGE_DELTA) {
        servo.write(MIN_SERVO_SPEED + speed);
    } else {
        servo.write(MIN_SERVO_SPEED);
    }
}
