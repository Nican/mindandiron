// Put professional header here
// Include some type of copyright

#ifndef TEENSY_TEENSY1_COLLECTOR_SORTER_INCLUDES_H_
#define TEENSY_TEENSY1_COLLECTOR_SORTER_INCLUDES_H_

#define SORTER_OUT_PIN        22  // PWM control on the sorter
#define COLLECTOR_OUT_A       5   // Goes to one side of the collector
#define COLLECTOR_OUT_B       6   // Goes to one side of the collector
#define COLLECTOR_OUT_PWM     23   // Goes to speed control of the collector
#define COLLECTOR_HIGH_SPEED  100 // Analog written value to PWM enable pin
#define COLLECTOR_LOW_SPEED   40 // Analog written value to PWM enable pin
#define NUM_SORTER_SLOTS      10  // Number of slots in the sorter
#define SORTER_POT_PIN_IN     50  // Redo with the correct number

#endif  // TEENSY_TEENSY1_COLLECTOR_SORTER_INCLUDES_H_
