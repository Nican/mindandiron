// Put professional header here
// Include some type of copyright

#include <Arduino.h>

// int accelerometerAxes[3] = {0, 0, 0};  // Done in teensy1.ino
#define accelerometerXPin A9
#define accelerometerYPin A8
#define accelerometerZPin A7

// TODO(Eric): Make more complex with conversions/filtering after testing
int readAccelerometerSingleAxis(int pin) {
    int val = analogRead(pin);
    return val;
}

void readAccelerometer(int *axes) {
    axes[0] = readAccelerometerSingleAxis(accelerometerXPin);
    axes[1] = readAccelerometerSingleAxis(accelerometerYPin);
    axes[2] = readAccelerometerSingleAxis(accelerometerZPin);
}
