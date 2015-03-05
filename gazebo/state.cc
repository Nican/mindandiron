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
	std::cout << "Starting returning to base routine\n";

	mStateStartTime = mRobot->mCurTime;
}

void ReturnToBase::Think()
{
	double progress = mRobot->mCurTime - mStateStartTime;

	if(progress < 8.0)
	{
		//std::cout << "Moving forward\n";
		mRobot->mMotion.mLeftWheel->SetForce(50.0);
		mRobot->mMotion.mRightWheel->SetForce(47.0);
		return;
	}

	if(progress < 1600.0)
	{
		//std::cout << "Solution: " << mRobot->mBaseStation->mSolution.transpose() << "\n";
		return;
	}

	if(mStage == ReturnToBaseStage::PositionAfar)
	{
		static double nextThink = 0.0;

		if(progress < nextThink)
		{
			return;
		}
		nextThink = progress + 1.5;



		static const Eigen::Vector3d goal(3.0, 0, 0);

		auto baseStationTrans = mRobot->mBaseStation->mBaseTransformation;

		auto goal2 = baseStationTrans.linear() * goal;
		double angle = mRobot->mSensors.mTRS->GetOrientation();

		Eigen::Vector2d diff2 = goal2.head<2>() - baseStationTrans.translation().head<2>();
		double desiredAngle = std::atan2(diff2.y(), diff2.x());
		double angleDiff = fmod(desiredAngle - angle, M_PI / 2.0);

		if(diff2.norm() < 0.5)
		{
		//	return;
		}

		auto inverse = baseStationTrans.inverse();

		auto q1 = baseStationTrans.linear() * Eigen::Vector3d(0,1,0);
		auto q2 = inverse.linear() * Eigen::Vector3d(0,1,0);

		//std::cout << "Robot: \n";
		//std::cout << "\t R: \t" << mRobot->mSensors.mTRS->GetPosition().transpose()  << " ("<< mRobot->mSensors.mTRS->GetOrientation() <<")\n";
		//std::cout << "\t T: \t" << baseStationTrans.translation().transpose() << " ("<< std::atan2(q1.y(), q1.x()) <<")\n";
		//std::cout << "\t I: \t" << inverse.translation().transpose()  << " ("<< std::atan2(q2.y(), q2.x()) <<")\n";
		//std::cout << "\t G: \t" << goal2.transpose() << "\n"; 

		/*
		double factor = 40.0;

		if(angleDiff < 0.01)
		{
			mRobot->mMotion.mLeftWheel->SetForce(36);
			mRobot->mMotion.mRightWheel->SetForce(40);
		}
		else if(angleDiff > 0.01)
		{
			mRobot->mMotion.mLeftWheel->SetForce(40);
			mRobot->mMotion.mRightWheel->SetForce(36);
		} 
		else 
		{
			mRobot->mMotion.mLeftWheel->SetForce(40);
			mRobot->mMotion.mRightWheel->SetForce(40);
		}
		*/



		//auto b2 = baseStationTrans 


		//auto diff = baseStationTrans.inverse() *  goal - baseStationTrans.translation();



		//goal = baseStationTrans.translation() + Vector3d();

		//std::cout << "Transformation: " << diff.transpose() << "\n\t" << baseStationTrans.transpose() << "\n" ;

	}

	/*
	else 
	{
		mRobot->mMotion.mLeftWheel->SetForce(0.0);
		mRobot->mMotion.mRightWheel->SetForce(0.0);
	}
	*/

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