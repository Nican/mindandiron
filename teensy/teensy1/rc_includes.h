// Put professional header here
// Include some type of copyright

#ifndef TEENSY_TEENSY1_RC_INCLUDES_H_
#define TEENSY_TEENSY1_RC_INCLUDES_H_

#define LEFT_CMD_IN      18  // Pin 3 on the receiver
#define RIGHT_CMD_IN     17  // Pin 2 on the receiver
#define COLLECTOR_CMD_IN 3   // Pin X on the reciever
#define SORTER_CMD_IN    4   // Pin X on the reciever
#define AUTO_SWITCH_IN   19  // Pin 5 on the receiver
#define AUTO_SWITCH_OUT  7   // Pin 7 on the Teensy, goes to Pin 4 on the Arduino
#define LEFT_OUT_PIN     20  // PWM control on the left wheel
#define RIGHT_OUT_PIN    21  // PWM control on the right wheel
#define MAX_SERVO_SPEED  130
#define MIN_SERVO_SPEED  95  // Though it seems 90 should be centered, the motors move at 90, stop at 95
#define COLLECTOR_MIN_SERVO_SPEED  93
#define MAX_SIGNAL 1950
#define MID_SIGNAL 1525
#define MIN_SIGNAL 1100
#define IN_DEADBAND 100  // Gives a deadband around zero for RC stability

#endif  // TEENSY_TEENSY1_RC_INCLUDES_H_
