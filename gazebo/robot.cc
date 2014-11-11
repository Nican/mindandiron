#include "robot.h"

#include <iostream>
#include <algorithm>

using namespace Robot;

Eigen::Vector2d gDesiredGoal(1.5, -1.5);
double desiredRotation = M_PI / 2;

Kratos::Kratos(const RobotMotion& motion, const RobotSensors& sensors) 
	: mMotion(motion), mSensors(sensors), mStartSimTime(0.0), mZmqContext(1), mZmqSocket(mZmqContext, ZMQ_PUB)
{
	mZmqSocket.bind ("tcp://*:5555");
}

void Kratos::Update(double simTime)
{
	if(mStartSimTime == 0.0)
	{
		mLastSimTime = simTime;
		mStartSimTime = simTime;

		//Just initialization, we skip the first loop to get the time step correctly
		return;
	}

	LocationDataPoint historyPoint{simTime, mSensors.mTRS->GetPosition()};
	mLocationHistory.mPoints.push_back(historyPoint);


	msgpack::sbuffer sbuf;
	msgpack::pack(sbuf, historyPoint);
	zmq::message_t msg(sbuf.size());
	memcpy(msg.data(), sbuf.data(), sbuf.size());
	mZmqSocket.send (msg);


	double curTime = simTime - mStartSimTime;

	//First state
	if(curTime <= 3.0)
	{
		mMotion.mLeftWheel->SetForce(0.1);
		mMotion.mRightWheel->SetForce(0.1);
	}
	else
	{
		mMotion.mLeftWheel->SetForce(0.2);
		mMotion.mRightWheel->SetForce(0.1);

		Eigen::Vector3d last5Average;
		Eigen::Vector3d last5beforeAverage;

		for(int i = 1; i <= 5; i++)
		{
			auto end = mLocationHistory.mPoints.end();

			last5Average += (end - i)->mPosition;
			last5beforeAverage += (end - i - 5)->mPosition;
		}

		last5Average /= 5.0;
		last5beforeAverage /= 5.0;

		Eigen::Vector3d diff = last5beforeAverage - last5Average;
		std::cout << "Angle: " << std::atan2(diff.y(), diff.x()) << std::endl;
	}


	//std::cout << mSensors.mTRS->GetPosition().transpose() << std::endl;

	mLastSimTime = simTime;
}