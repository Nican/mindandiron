#include "robot.h"
#include "state.h"
#include "msgpack.h"

#include <iostream>

namespace Robot {
namespace State {


void MoveToWaypoint::Initialize()
{

}

void MoveToWaypoint::Think()
{
	Eigen::Vector3d pos = mRobot->mSensors.mTRS->GetPosition();
	double angle = mRobot->mSensors.mTRS->GetOrientation();

	if(mRobot->mCurTime - lastReplan > 5.0 )
	{
		Replan();
		lastReplan = mRobot->mCurTime;
	}

	if(mCurrentPoint < mPath.size())
	{
		auto nextPoint = mPath[mPath.size() - mCurrentPoint - 1];

		Eigen::Vector2d diff2 = nextPoint->mPoint - pos.head<2>();
		double desiredAngle = std::atan2(diff2.y(), diff2.x());
		double angleDiff = fmod(desiredAngle - angle, M_PI / 2.0);

		//std::cout << "Current goal: " << nextPoint->mPoint.transpose() << "\n";

		//std::cout << "\tAngle diff: " << diff2.transpose() << std::endl;
		
		if(angleDiff < 0.01)
		{
			mRobot->mMotion.mLeftWheel->SetForce(0.2);
			mRobot->mMotion.mRightWheel->SetForce(0.4);
		}
		else if(angleDiff > 0.01)
		{
			mRobot->mMotion.mLeftWheel->SetForce(0.4);
			mRobot->mMotion.mRightWheel->SetForce(0.2);
		} 
		else 
		{
			mRobot->mMotion.mLeftWheel->SetForce(0.4);
			mRobot->mMotion.mRightWheel->SetForce(0.4);
		}

		if(diff2.norm() < 1.0)
		{
			mCurrentPoint++;
			std::cout << " New target id: " << mCurrentPoint << "\n"; 

			GetTelemetry();
		}
	} 
	else 
	{
		mRobot->mMotion.mLeftWheel->SetForce(0.0);
		mRobot->mMotion.mRightWheel->SetForce(0.0);
	}

}

std::shared_ptr<msgpack::sbuffer> MoveToWaypoint::GetTelemetry()
{
	/*
	std::vector<Eigen::Vector2d> waypoints;
	waypoints.reserve(mPath.size());

	for(auto pt : mPath)
		waypoints.push_back(pt->mPoint);

	auto sbuf = std::make_shared<msgpack::sbuffer>();
	msgpack::pack(*sbuf, waypoints);

	return sbuf;	
	*/
	/*
	std::vector<Eigen::Vector2d> waypoints;
	waypoints.reserve(mPath.size());

	for(auto pt : mPath)
		waypoints.push_back(pt->mPoint);

	mRobot->SendTelemetry(1, MoveToWaypointTelemetry({mCurrentPoint, waypoints}));
	*/

	msgpack::sbuffer sbuf;
	char id = 1;
	sbuf.write(&id, sizeof(id));

	{
		msgpack::packer<msgpack::sbuffer> stream(sbuf);
		mPlanner->pack(stream);	
	}

	zmq::message_t msg(sbuf.size());
	memcpy(msg.data(), sbuf.data(), sbuf.size());
	mRobot->mZmqSocket.send (msg);

	std::cout << "Sending message out ("<< sbuf.size() <<")!\n";

	return nullptr;
}

void MoveToWaypoint::Replan()
{
	Eigen::Vector3d pos = mRobot->mSensors.mTRS->GetPosition();
	double angle = mRobot->mSensors.mTRS->GetOrientation();

	std::cout << "Replan from " << angle << "\n";

	mPlanner.reset( new TrajectoryPlanner(pos.head<2>(), std::polar(angle, 1.0), {1.0,6.0}));
	mPlanner->run(200);

	auto newResult = mPlanner->getResult();

	if(newResult.size() > 0)
	{
		mPath = newResult;
		mCurrentPoint = 0;
	}

	std::cout << "Replan with: " << mPath.size() << " points\n";

	GetTelemetry();
	
}

} //State
} //Robot