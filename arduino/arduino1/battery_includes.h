// Put professional header here
// Include some type of copyright

#ifndef ARDUINO_ARDUINO1_BATTERY_INCLUDES_H_
#define ARDUINO_ARDUINO1_BATTERY_INCLUDES_H_

#define VCC     5     // If we use the Teensy, switch to 3.3
#define R1      ####  // Ohms. The resister between pos. terminal and microcontroller
#define R2      ####  // Ohms. The resistor between microcontroller and neg. terminal

#define CURRENT_IN_PIN       A0  // Voltage goes up with current
#define CURRENT_REF_PIN      A1  // Provides a reference "zero" voltage for current
#define BATT_VOLTAGE_IN_PIN  A2  // Voltage bridge for measuring battery voltage

#endif  // ARDUINO_ARDUINO1_BATTERY_INCLUDES_H_
