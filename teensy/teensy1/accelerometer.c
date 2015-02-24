// Put professional header here

#include "accelerometer.h"

int accelerometerAxes[3] = {0, 0, 0};  // Done in teensy1.ino
const int accelerometerXPin = A9;
const int accelerometerYPin = A8;
const int accelerometerZPin = A7;

void readAccelerometer() {
    accelerometerAxes[0] = readAccelerometerSingleAxis(accelerometerXPin);
    accelerometerAxes[1] = readAccelerometerSingleAxis(accelerometerYPin);
    accelerometerAxes[2] = readAccelerometerSingleAxis(accelerometerZPin);
}

// TODO: Make more complex with conversions/filtering after testing
int readAccelerometerSingleAxis(int pin) {
    int val = analogRead(pin);
    return val;
}
