
#include "msgpack.h"


namespace Robot{

struct TeenseyStatus
{
	double leftPosition;
	double rightPosition;
	Eigen::Vector3d acceleration;
	bool autoFlag;

	MSGPACK_DEFINE(leftPosition, rightPosition, acceleration, autoFlag);
};

}