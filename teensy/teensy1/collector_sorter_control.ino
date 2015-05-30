// Put professional header here
// Include some type of copyright


volatile int lastMagnetState = HIGH;
volatile int travelDirection = HIGH;  // HIGH when increasing numbers
volatile int currentSlot = 0;


void checkSorterSlotChange() {
    int currentMagnetState = digitalRead(SORTER_MAGNET_PIN);
    if (lastMagnetState == HIGH and currentMagnetState == LOW) {
        if (travelDirection) {
            currentSlot++;
        } else {
            currentSlot--;
        }
    }
    lastMagnetState = currentMagnetState;
}


int getCurrentSlot() {
    return currentSlot;
}


// TODO(Eric): Test
void commandSorter(Servo servo, int slot) {
    travelDirection = HIGH;
    if (slot == currentSlot) {
        servo.write(MIN_SERVO_SPEED);
    } else if (slot < currentSlot) {
        servo.write(MIN_SERVO_SPEED + SORTER_SPEED);
    } else {
        servo.write(MIN_SERVO_SPEED - SORTER_SPEED);
        travelDirection = LOW;
    }
}


// Speed is a servo, speed goes from -85 to 85, recentered around servo 0 (95)
void commandCollectorSimple(Servo servo, int speed) {
    if (abs(speed) <= SERVO_OUTPUT_LARGE_DELTA) {
        servo.write(COLLECTOR_MIN_SERVO_SPEED + speed);
    } else {
        servo.write(COLLECTOR_MIN_SERVO_SPEED);
    }
}


// Speed is a servo, speed goes from -85 to 85, recentered around servo 0 (95)
void commandCollectorFromWheelV(Servo servo, float left, float right) {
    int speed = (int) ((left + right) * COLLECTOR_MPERS_TO_SPEED);
    long currentTime = millis();
    if (currentTime < (timeOfLastComputerCommand + allowableComputerCommandLag)) {
        if (abs(speed) <= SERVO_OUTPUT_LARGE_DELTA) {
            servo.write(COLLECTOR_MIN_SERVO_SPEED + speed);
        } else {
            servo.write(COLLECTOR_MIN_SERVO_SPEED);
        }
    } else {
        servo.write(COLLECTOR_MIN_SERVO_SPEED);
    }
}
