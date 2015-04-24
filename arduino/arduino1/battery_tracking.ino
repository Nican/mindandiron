// Put professional header here
// Include some type of copyright


// Amount that analogRead goes up by every amp in 0-1023 scale.
// Equivalent to 0.028V if VCC == 5
const float voltsPerAmp = 0.028 * 1023 / VCC;  

float getCurrent(void) {
    int sensorVal = analogRead(CURRENT_IN_PIN);
    int sensorRef = analogRead(CURRENT_REF_PIN);
    sensorVal -= sensorRef;  // Remove 0 amp DC offset
//    Serial.print("--- voltsPerAmp\t"); Serial.println(voltsPerAmp);
//    Serial.print("--- Current (A)\t"); Serial.println(sensorVal / voltsPerAmp);
    return sensorVal / voltsPerAmp;
}

float getBatteryVoltage(void) {
    int sensorVal = analogRead(BATT_VOLTAGE_IN_PIN);
    float measuredVoltage = (sensorVal * VCC) / 1023.0;  // Converts 0-1023 to volts
   Serial.print("--- sensorVal (tick)\t"); Serial.println(sensorVal);
   Serial.print("--- Voltage (V)\t"); Serial.println(measuredVoltage);
   Serial.print("--- VCC (V)\t"); Serial.println(VCC);
    return ( measuredVoltage * (R1 + R2) / R2 );  // Calculates battery voltage from voltage divider
}

