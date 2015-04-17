#include "state.h"

using namespace Robot;


////////////////////
////	BaseState
////////////////////

bool BaseState::IsActive()
{
	return mRobot->mState == this;
}


////////////////////
////	MoveForwardState
////////////////////

void MoveForwardState::Start()
{
	mRobot->mWheelPID->Reset();

	mStartTime = QDateTime::currentDateTime();

	connect(mRobot->GetTeensy(), &Teensy::statusUpdate, this, &MoveForwardState::TeensyStatus);

	std::cout << "Start going forward\n";
}

void MoveForwardState::TeensyStatus(TeenseyStatus status)
{
	auto odometry = mRobot->GetOdometryTraveledSince(mStartTime);

	mRobot->mWheelPID->SetLeftDesiredVelocity(0.5);
	mRobot->mWheelPID->SetRightDesiredVelocity(0.5);

	std::cout << "\tDistance traveled " << odometry.mDistanceTraveled << "\n";

	if(std::abs(odometry.mDistanceTraveled) > std::abs(mDistance))
	{
		RotateState* newState = new RotateState(mRobot, M_PI / 2);
		mRobot->SetState(newState);
	}
}


////////////////////
////	RotateState
////////////////////

void RotateState::Start()
{
	mRobot->mWheelPID->Reset();

	mStartTime = QDateTime::currentDateTime();

	connect(mRobot->GetTeensy(), &Teensy::statusUpdate, this, &RotateState::TeensyStatus);

	std::cout << "Start Rotation\n";
}

void RotateState::TeensyStatus(TeenseyStatus status)
{
	auto odometry = mRobot->GetOdometryTraveledSince(mStartTime);

	mRobot->mWheelPID->SetLeftDesiredVelocity(0.5);
	mRobot->mWheelPID->SetRightDesiredVelocity(-0.5);

	std::cout << "\t Angletraveled " << odometry.mTheta << "\n";

	if(std::abs(odometry.mTheta) > std::abs(mRotation))
	{
		MoveForwardState* newState = new MoveForwardState(mRobot, 1.0);
		mRobot->SetState(newState);
	}
}
