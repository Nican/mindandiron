// Put professional header here
// Include some type of copyright

#include <Arduino.h>

#define accelerometerXPin A3
#define accelerometerYPin A2
#define accelerometerZPin A1

int accelerometerAxes[3] = {0, 0, 0};  // Stores {x, y, z} accelerometer data


// TODO(Eric): Make more complex with conversions/filtering after testing
int readAccelerometerSingleAxis(int pin) {
    Serial.print("Pin: "); Serial.println(pin);
    int val = analogRead(pin);
    return val;
}


void readAccelerometer() {
    accelerometerAxes[0] = readAccelerometerSingleAxis(accelerometerXPin);
    accelerometerAxes[1] = readAccelerometerSingleAxis(accelerometerYPin);
    accelerometerAxes[2] = readAccelerometerSingleAxis(accelerometerZPin);
}

int getAX() {
    return accelerometerAxes[0];
}

int getAY() {
    return accelerometerAxes[1];
}

int getAZ() {
    return accelerometerAxes[2];
}
