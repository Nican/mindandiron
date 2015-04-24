#include "state.h"

using namespace Robot;


////////////////////
////	BaseState
////////////////////

bool BaseState::IsValid()
{
	return mRobot != nullptr;
}

////////////////////
////	RootState
////////////////////

RootState::RootState(Kratos2 *parent) : BaseState(parent)
{
	//MoveToNextState();
}

void RootState::SetState(ProgressState* nextState)
{
	auto oldState = mState;

	nextState->setParent(this);
 	mState = nextState;
 	connect(nextState, &ProgressState::Finished, this, &RootState::MoveToNextState);
 	nextState->Start();

 	if(oldState != nullptr)
 	{
 		oldState->End();
 		oldState->deleteLater();
 	}
}

void RootState::MoveToNextState()
{
	if(qobject_cast<MoveForwardState*>(mState) == nullptr)
	{
		ProgressState* newState = new MoveForwardState(mRobot, 2.0);
		SetState(newState);
	} 
	else
	{
		ProgressState* newState = new RotateState(mRobot, M_PI / 2);
		SetState(newState);
	}
}


////////////////////
////	ProgressState
////////////////////

bool ProgressState::IsFinished()
{
	return mIsFinished;
}

bool ProgressState::IsValid()
{
	return IsFinished() && BaseState::IsValid();
}

void ProgressState::SetFinished()
{
	mIsFinished = true;
	emit Finished();
}

////////////////////
////	MoveForwardState
////////////////////

void MoveTowardsGoalState::Start()
{
	mRobot->mWheelPID->Reset();

	connect(mRobot->GetTeensy(), &Teensy::statusUpdate, this, &MoveTowardsGoalState::TeensyStatus);
}

void MoveTowardsGoalState::TeensyStatus(TeenseyStatus status)
{
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
	if(IsFinished())
		return;

	auto odometry = mRobot->GetOdometryTraveledSince(mStartTime);

	mRobot->mWheelPID->SetLeftDesiredVelocity(0.5);
	mRobot->mWheelPID->SetRightDesiredVelocity(0.5);

	//std::cout << "\tDistance traveled " << odometry.mDistanceTraveled << "\n";

	if(std::abs(odometry.mDistanceTraveled) > std::abs(mDistance))
	{
		std::cout << "Move state finished with " << odometry << "\n";
		SetFinished();
		//RotateState* newState = new RotateState(mRobot, M_PI / 2);
		//mRobot->SetState(newState);
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

	std::cout << "Start Rotate State\n";
}

void RotateState::TeensyStatus(TeenseyStatus status)
{
	if(IsFinished())
		return;

	auto odometry = mRobot->GetOdometryTraveledSince(mStartTime);

	mRobot->mWheelPID->SetLeftDesiredVelocity(0.5);
	mRobot->mWheelPID->SetRightDesiredVelocity(-0.5);

	if(std::abs(odometry.mTheta) > std::abs(mRotation))
	{
		SetFinished();
		std::cout << "Rotate state finished with " << odometry << "\n";
		// MoveForwardState* newState = new MoveForwardState(mRobot, 1.0);
		// mRobot->SetState(newState);
	}
}