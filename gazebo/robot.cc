#include "robot.h"

#include <iostream>
#include <algorithm>
#include <stdlib.h>

using namespace Robot;

Eigen::Vector2d gDesiredGoal(5.0, 5.0);
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

	Eigen::Vector3d pos = mSensors.mTRS->GetPosition();
	double angle = mSensors.mTRS->GetOrientation();
	LocationDataPoint historyPoint{simTime, pos, angle};
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
		//mMotion.mLeftWheel->SetForce(0.0);
		//mMotion.mRightWheel->SetForce(0.0);

		/*
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
		*/
		//Eigen::Vector3d diff = last5beforeAverage - last5Average;
		//double angle = std::atan2(diff.y(), diff.x());
		
		//std::cout << "Angle: " << angle << " at: (" << pos.head<2>().transpose() << ")" << std::endl;

		Eigen::Vector2d diff2 = gDesiredGoal - pos.head<2>();
		double desiredAngle = std::atan2(diff2.y(), diff2.x());

		double angleDiff = fmod(desiredAngle - angle, M_PI / 2.0);



		std::cout << "\tAngle diff: " << diff2.transpose() << std::endl;

		
		if(angleDiff < 0.01)
		{
			mMotion.mLeftWheel->SetForce(0.2);
			mMotion.mRightWheel->SetForce(0.4);
		}
		else if(angleDiff > 0.01)
		{
			mMotion.mLeftWheel->SetForce(0.4);
			mMotion.mRightWheel->SetForce(0.2);
		} 
		else 
		{
			mMotion.mLeftWheel->SetForce(0.4);
			mMotion.mRightWheel->SetForce(0.4);
		}
	
		//mMotion.mLeftWheel->SetForce(0.4);
		//mMotion.mRightWheel->SetForce(0.2);
		
		
	}


	//std::cout << mSensors.mTRS->GetPosition().transpose() << std::endl;

	mLastSimTime = simTime;
}