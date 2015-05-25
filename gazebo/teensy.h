
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

	//MSGPACK_DEFINE(leftPosition, rightPosition, leftVelocity, rightVelocity, acceleration, autoFlag);
};

struct Teensy2Status
{
	double servoAngle;
	double current;
	double voltage;
	int isPaused;

	//MSGPACK_DEFINE(servoAngle, current, voltage, isPaused);
};

}

QDataStream &operator<<(QDataStream &out, const Robot::TeenseyStatus &item);
QDataStream &operator>>(QDataStream &in, Robot::TeenseyStatus &item);

QDataStream &operator<<(QDataStream &out, const Robot::Teensy2Status &item);
QDataStream &operator>>(QDataStream &in, Robot::Teensy2Status &item);
