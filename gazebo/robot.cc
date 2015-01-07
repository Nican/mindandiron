#include "robot.h"

#include <iostream>
#include <algorithm>
#include <stdlib.h>
#include "trajectory.h"


using namespace Robot;



Kratos::Kratos(const RobotMotion& motion, const RobotSensors& sensors) 
	: mMotion(motion), mSensors(sensors), mStartSimTime(0.0), mZmqContext(1), mZmqSocket(mZmqContext, ZMQ_PUB)
{
	mZmqSocket.bind ("tcp://*:5555");

	//mState.reset(new State::MoveToWaypoint(this));
	mState.reset(new State::MoveForward(this));
	mState->Initialize();
	
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

	SendTelemetry(0, historyPoint);

	mCurTime = simTime - mStartSimTime;

	mState->Think();

	mLastSimTime = simTime;
}