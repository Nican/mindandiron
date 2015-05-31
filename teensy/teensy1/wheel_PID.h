// Put professional header here
// Include some type of copyright

#define VELOCITY_PERIOD_MICRO 	10000  // Period of vel calc, microseconds
// Ignored, bundled in with VELOCITY_PERIOD_MICRO
// #define PID_PERIOD_MICRO      	10000  // Period of wheel PID, microseconds
#define WHEEL_TICKS_PER_REV   	23330  // Determined experimentally for encoder
#define WHEEL_RADIUS    		0.155  // meters
#define PI              		3.1416
#define PID_CUTOFF_VELOCITY		0.001  // meters/second command at which robot will brake abruptly
