
#include "msgpack.h"


namespace Robot{

struct TeenseyStatus
{
	double leftPosition;
	double rightPosition;
	double leftVelocity;
	double rightVelocity;

	Eigen::Vector3d acceleration;
	bool autoFlag;

	MSGPACK_DEFINE(leftPosition, rightPosition, leftVelocity, rightVelocity, acceleration, autoFlag);
};

struct Teensy2Status
{
	double servoAngle;
	double current;
	double voltage;
	int isPaused;

	MSGPACK_DEFINE(servoAngle, current, voltage, isPaused);
};

}