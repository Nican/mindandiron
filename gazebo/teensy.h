
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

}