// Put professional header here
// Include some type of copyright

#ifndef TEENSY_TEENSY1_RC_INCLUDES_H_
#define TEENSY_TEENSY1_RC_INCLUDES_H_

#define LEFT_CMD_IN      11  // Pin 2 on the receiver
#define RIGHT_CMD_IN     12  // Pin 3 on the receiver
#define AUTO_SWITCH_IN   10  // Pin 5 on the receiver
#define AUTO_SWITCH_OUT  4 
#define LEFT_OUT_PIN     3   // PWM control on the left wheel
#define RIGHT_OUT_PIN    2   // PWM control on the right wheel
#define MAX_SERVO_SPEED 115
#define MIN_SERVO_SPEED 95
#define MAX_SIGNAL 1950
#define MID_SIGNAL 1525
#define MIN_SIGNAL 1100
#define IN_DEADBAND 100  // Gives a deadband around zero for RC stability

#endif  // TEENSY_TEENSY1_RC_INCLUDES_H_
