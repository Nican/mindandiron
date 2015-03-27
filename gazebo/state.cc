#include "robot.h"
#include "state.h"
#include "msgpack.h"

#include <iostream>

namespace Robot {
namespace State {




void MoveForward::Initialize()
{
	std::cout << "Moving forward!\n";
}

void MoveForward::Think()
{
	mRobot->mMotion.mLeftWheel->SetForce(50.0);
	mRobot->mMotion.mRightWheel->SetForce(50.0);
}




void ReturnToBase::Initialize()
{
	mStateStartTime = mRobot->mCurTime;
	std::cout << "Starting returning to base routine (" << mStateStartTime <<")\n";
}

void ReturnToBase::Think()
{
	using namespace Eigen;
	double progress = mRobot->mCurTime - mStateStartTime;

	if(progress < 10.0)
	{
		//Just getting off center somewhere
		mRobot->mMotion.mLeftWheel->SetForce(47.0);
		mRobot->mMotion.mRightWheel->SetForce(50.0);
		return;
	}

	//Get ourselves in front of the base station
	if(mStage == ReturnToBaseStage::PositionAfar)
	{
		/*
		static double nextThink = 0.0;

		if(progress < nextThink)
		{
			return;
		}
		nextThink = progress + 1.5;
		*/

		//Run straight to our desired location
		static const Vector2d goal(2.5, 0);

		Vector2d current = mRobot->mSensors.mTRS->GetPosition().head<2>();
		Vector2d diff = goal - current;

		if(diff.norm() < 0.1)
		{
			//Go to next stage
			mStage = ReturnToBaseStage::Alignmenet;
			return;
		}

		double targetAngle = std::atan2(diff.y(), diff.x());
		double currentAngle = mRobot->mSensors.mTRS->GetOrientation();

		Complex targetAngle2 = rotationToCompex(targetAngle);
		Complex currentAngle2 = rotationToCompex(currentAngle);

		Complex diff2 = targetAngle2 * std::conj(currentAngle2);
		double diffAngle = std::atan2(diff2.imag(), diff2.real()); //A value between pi and -pi

		//std::cout << "Current: " << current.transpose() << "\t" << goal.transpose() << "\n";
		//std::cout << "Diff: " << currentAngle << "\t" << targetAngle << "\t" << diffAngle << "\n";

		//If we are not alligned to go towards the goal
		if(std::abs(diffAngle) > (5*M_PI/180))
		{
			if(diffAngle < 0)
			{
				mRobot->mMotion.mLeftWheel->SetForce(30.0);
				mRobot->mMotion.mRightWheel->SetForce(-30.0);
			}
			else
			{
				mRobot->mMotion.mLeftWheel->SetForce(-30.0);
				mRobot->mMotion.mRightWheel->SetForce(30.0);
			}

			return;
		}

		mRobot->mMotion.mLeftWheel->SetForce(30.0);
		mRobot->mMotion.mRightWheel->SetForce(30.0);

		
	}

	if( mStage == ReturnToBaseStage::Alignmenet )
	{
		Complex targetAngle2(1, 0); //Pointing outwards
		Complex currentAngle2 = rotationToCompex(mRobot->mSensors.mTRS->GetOrientation());
		Complex diff2 = targetAngle2 * std::conj(currentAngle2);
		double diffAngle = std::atan2(diff2.imag(), diff2.real()); 

		if(std::abs(diffAngle) > (5*M_PI/180))
		{
			if(diffAngle < 0)
			{
				mRobot->mMotion.mLeftWheel->SetForce(30.0);
				mRobot->mMotion.mRightWheel->SetForce(-30.0);
			}
			else
			{
				mRobot->mMotion.mLeftWheel->SetForce(-30.0);
				mRobot->mMotion.mRightWheel->SetForce(30.0);
			}

			return;
		}
		else 
		{
			mStage = ReturnToBaseStage::BackingUp;
			return;
		}
	}


	if( mStage == ReturnToBaseStage::BackingUp )
	{
		static const Vector2d goal2(-0.5, 0);
		Vector2d current = mRobot->mSensors.mTRS->GetPosition().head<2>();
		Vector2d diff = goal2 - current;

		if(std::abs(diff.x()) > 0.05)
		{
			mRobot->mMotion.mLeftWheel->SetForce(-20.0);
			mRobot->mMotion.mRightWheel->SetForce(-20.0);
		}

		//std::cout << "Diff " << diff.transpose() << "\n";

	}

}








void Explore::Initialize()
{

}

void Explore::Think()
{


	mRobot->mMotion.mLeftWheel->SetForce(48.0);
	mRobot->mMotion.mRightWheel->SetForce(50.0);
}







void MoveToWaypoint::Initialize()
{

}

void MoveToWaypoint::Think()
{
	Eigen::Vector3d pos = mRobot->mSensors.mTRS->GetPosition();
	double angle = mRobot->mSensors.mTRS->GetOrientation();

	if(mRobot->mCurTime - lastReplan > 30.0 )
	{
		//Replan();
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
		
		double factor = 40.0;

		if(angleDiff < 0.01)
		{
			mRobot->mMotion.mLeftWheel->SetForce(0.2 * factor);
			mRobot->mMotion.mRightWheel->SetForce(0.4 * factor);
		}
		else if(angleDiff > 0.01)
		{
			mRobot->mMotion.mLeftWheel->SetForce(0.4 * factor);
			mRobot->mMotion.mRightWheel->SetForce(0.2 * factor);
		} 
		else 
		{
			mRobot->mMotion.mLeftWheel->SetForce(0.4 * factor);
			mRobot->mMotion.mRightWheel->SetForce(0.4 * factor);
		}

		if(diff2.norm() < 1.0)
		{
			mCurrentPoint++;
			std::cout << " New target id: " << mCurrentPoint << "\n"; 

			//GetTelemetry();
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
		//mPlanner->pack(stream);	
		stream.pack_array(3);
		stream.pack(mPlanner->mGoal);
		stream.pack(mPlanner->rootNode->mPoint);
		stream.pack(mPlanner->rootNode->mRotation);

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

	std::shared_ptr<TrajectoryPlanner> planner;
	planner.reset(new TrajectoryPlanner(pos.head<2>(), std::polar(angle, 1.0), {1.0,6.0}));
	planner->run(2000);

	auto newResult = planner->getResult();

	if(newResult.size() > 0)
	{
		mPlanner = planner;
		mPath = newResult;
		mCurrentPoint = 0;
	}

	std::cout << "Replan with: " << mPath.size() << " points\n";

	GetTelemetry();
	
}

} //State
} //Robot