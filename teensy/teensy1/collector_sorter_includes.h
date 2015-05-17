// Put professional header here
// Include some type of copyright

#ifndef TEENSY_TEENSY1_COLLECTOR_SORTER_INCLUDES_H_
#define TEENSY_TEENSY1_COLLECTOR_SORTER_INCLUDES_H_

#define SORTER_OUT_PIN    22  // Servo PWM control on the sorter
#define NUM_SORTER_SLOTS  10  // Number of slots in the sorter
#define SORTER_SPEED      30  // Servo distance from MID (95)
#define SORTER_MAGNET_PIN 6   // Digital pin for magnetic signal
#define MAGNET_POLL_PERIOD_MICRO 1000  // micros between magnet polls
#define COLLECTOR_OUT_PIN        23    // Servo PWM control on the collector
#define COLLECTOR_MPERS_TO_SPEED 38    // Guesstimated constant to translate
									   // m/s velocity into Servo commands
#define SERVO_OUTPUT_SMALL_DELTA 15    // Servo output deviation from MID (95)
#define SERVO_OUTPUT_LARGE_DELTA 85    // Servo output deviation from MID (95)

#endif  // TEENSY_TEENSY1_COLLECTOR_SORTER_INCLUDES_H_
