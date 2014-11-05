#include "robot.h"

using namespace Robot;

Kratos::Kratos(const RobotMotion& motion, const RobotSensors& sensors) 
	: mMotion(motion), mSensors(sensors)
{


}

void Kratos::Update(double simTime)
{
	mMotion.mLeftWheel->SetForce(0.0);
	mMotion.mRightWheel->SetForce(0.0);
}