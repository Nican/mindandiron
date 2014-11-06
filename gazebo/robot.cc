#include "robot.h"

#include <iostream>

using namespace Robot;

Kratos::Kratos(const RobotMotion& motion, const RobotSensors& sensors) 
	: mMotion(motion), mSensors(sensors)
{


}

void Kratos::Update(double simTime)
{
	mMotion.mLeftWheel->SetForce(0.1);
	mMotion.mRightWheel->SetForce(0.1);

	std::cout << mSensors.mTRS->GetPosition().transpose() << std::endl;
}