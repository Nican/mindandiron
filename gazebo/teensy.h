
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
};

struct TeensyCommand
{
	double leftVel;
	double rightVel;
	int leftPos;
	int rightPos;
	int useVelocity;
	int collector;
	int sorter;
};

struct Teensy2Status
{
	double servoAngle;
	double current;
	double voltage;
	int isPaused;
};

}

QDataStream &operator<<(QDataStream &out, const Robot::TeenseyStatus &item);
QDataStream &operator>>(QDataStream &in, Robot::TeenseyStatus &item);

QDataStream &operator<<(QDataStream &out, const Robot::TeensyCommand &item);
QDataStream &operator>>(QDataStream &in, Robot::TeensyCommand &item);

QDataStream &operator<<(QDataStream &out, const Robot::Teensy2Status &item);
QDataStream &operator>>(QDataStream &in, Robot::Teensy2Status &item);
