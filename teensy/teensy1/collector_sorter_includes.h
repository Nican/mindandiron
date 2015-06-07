// Put professional header here
// Include some type of copyright

#ifndef TEENSY_TEENSY1_COLLECTOR_SORTER_INCLUDES_H_
#define TEENSY_TEENSY1_COLLECTOR_SORTER_INCLUDES_H_

#define SORTER_OUT_PIN    22  // Servo PWM control on the sorter
#define NUM_SORTER_SLOTS  10  // Number of slots in the sorter
#define SORTER_SPEED      45  // Servo distance from MID (95)
#define SORTER_1_IN_PIN   0
#define SORTER_2_IN_PIN   1
#define MAGNET_POLL_PERIOD_MICRO 1000  // micros between magnet polls
#define COLLECTOR_OUT_PIN        23    // Servo PWM control on the collector
#define COLLECTOR_MPERS_TO_SPEED 38    // Guesstimated constant to translate
									   // m/s velocity into Servo commands

#endif  // TEENSY_TEENSY1_COLLECTOR_SORTER_INCLUDES_H_
