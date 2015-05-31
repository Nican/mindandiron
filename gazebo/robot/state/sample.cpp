#include "state.h"

using namespace Robot;
using namespace Eigen;

////////////////////
////	LeaveBaseStation
////////////////////

void LeaveBaseStation::Start()
{
	auto move = new MoveForwardState(this, 2.0);
	move->Start();

	connect(move, &ProgressState::Finished, this, &LeaveBaseStation::MoveToRotate);
}

void LeaveBaseStation::MoveToRotate()
{
	auto rotate = new RotateState(this, M_PI/2);
	rotate->Start();

	connect(rotate, &ProgressState::Finished, this, &LeaveBaseStation::MoveToNextState);
}

void LeaveBaseStation::MoveToNextState()
{
	connect(Robot(), &Kratos2::AprilLocationUpdate, this, &LeaveBaseStation::FoundAprilTag);
}

void LeaveBaseStation::FoundAprilTag(Eigen::Affine2d newLocation)
{
	Robot()->SetWheelVelocity(0.0, 0.0);

	if(mMoveInfront != nullptr)
		return;

	Rotation2Dd rotation2D(0);
	rotation2D.fromRotationMatrix(newLocation.linear());
	
	mMoveInfront = new MoveTowardsGoalState(this);
	mMoveInfront->mGoal = Vector2d(0.0, 3.0);
	mMoveInfront->mStartPos = newLocation.translation();
	mMoveInfront->mStartAngle = rotation2D.angle();
	mMoveInfront->mReverse = true;
	mMoveInfront->mAprilUpdates = true;
	mMoveInfront->Start();
}
