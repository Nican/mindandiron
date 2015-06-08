// Put professional header here
// Include some type of copyright

int commandedSlot = 0;
volatile int sorter1State = LOW;
volatile int lastSorter1State = LOW;
volatile int sorter2State = LOW;
volatile int lastSorter2State = LOW;
volatile int travelDirection = HIGH;  // HIGH when increasing numbers
volatile int currentSlot = 0;
int i = 0;  // For counting how long it takes to get to next state
const int iScalar = 10;  // Scales i effect on sorter speed
int speedAdd = 0;  // Adds increasingly strong commands as time goes on, to deal with sticking


void checkSorterSlotChange() {
    int sorter1State = digitalRead(SORTER_1_IN_PIN);
    int sorter2State = digitalRead(SORTER_2_IN_PIN);

    if (lastSorter1State == HIGH and sorter1State == LOW) {
        if (travelDirection) {
            Serial.println("INCREMENTING slot");
            currentSlot++;
        } else {
            Serial.println("DECREMENTING slot");
            currentSlot--;
        }
    }

    lastSorter1State = sorter1State;
    lastSorter2State = sorter2State;
}


int getCurrentSlot() {
    return currentSlot;
}


// TODO(Eric): Test
void commandSorter(Servo servo, int slot) {
    travelDirection = HIGH;

    i++;
//    if (i < 200) {
//        speedAdd = 0;
//    } else {
//        speedAdd = (i - 200) / iScalar;
//    }
    speedAdd = i / iScalar;
//    Serial.print("i: "); Serial.print(i); Serial.print("\tSlot CMD: "); Serial.print(slot); Serial.print("\tcurrentSlot: "); Serial.println(getCurrentSlot());

    if (i % 20 == 0) {
        if (slot == currentSlot) {
            servo.write(MIN_SERVO_SPEED);
            i = 0;
        } else if (slot > currentSlot) {
            servo.write(140);
//            servo.write(MIN_SERVO_SPEED + SORTER_SPEED);
//            if ((MIN_SERVO_SPEED + SORTER_SPEED + speedAdd) < 180) {
//                servo.write(MIN_SERVO_SPEED + SORTER_SPEED + speedAdd);
//            } else {
//                servo.write(180);
//            }
        } else {
            servo.write(0);
//            servo.write(MIN_SERVO_SPEED - SORTER_SPEED);
//            if ((MIN_SERVO_SPEED - SORTER_SPEED - speedAdd) > 0) {
//                servo.write(MIN_SERVO_SPEED - SORTER_SPEED - speedAdd);
//            } else {
//                servo.write(0);
//            }
//            travelDirection = LOW;
        }
    } else {
        servo.write(MIN_SERVO_SPEED);
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
