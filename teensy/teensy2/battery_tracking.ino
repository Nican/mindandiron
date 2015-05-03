// Put professional header here
// Include some type of copyright


// Amount that analogRead goes up by every amp in 0-1023 scale.
// Equivalent to 0.028V if VCC == 5
const float voltsPerAmp = 0.028 * 1023 / VCC;  

float getCurrent(void) {
    int sensorVal = analogRead(CURRENT_IN_PIN);
    int sensorRef = analogRead(CURRENT_REF_PIN);
    sensorVal -= sensorRef;  // Remove 0 amp DC offset
    return sensorVal / voltsPerAmp;
}

// Voltage sensor not currently wired, didn't seem like priority - 5/3/15
// float getBatteryVoltage(void) {
//     int sensorVal = analogRead(BATT_VOLTAGE_IN_PIN);
//     float measuredVoltage = (sensorVal * VCC) / 1023.0;  // Converts 0-1023 to volts
//     return ( measuredVoltage * (R1 + R2) / R2 );  // Calculates battery voltage from voltage divider
// }

