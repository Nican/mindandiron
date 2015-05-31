#include "state.h"
#include <QtConcurrent>

using namespace Robot;
using namespace Eigen;

////////////////////
////	LeaveBaseStation
////////////////////

void LeaveBaseStation::Start()
{
	// connect(Robot()->GetSampleDetection(), &SampleDetection::SampleDetected, this, &LeaveBaseStation::NavigateToSample);
	connect(Robot()->GetSampleDetection(), &SampleDetection::SampleDetected, this, &LeaveBaseStation::ProportionalSteerOverSample);
	finalApproach = 0;

	//auto move = new MoveForwardState(this, 2.0);
	//move->Start();

	//connect(move, &ProgressState::Finished, this, &LeaveBaseStation::MoveToRotate);
}

void LeaveBaseStation::MoveToRotate()
{
	//auto rotate = new RotateState(this, M_PI/2);
	//rotate->Start();

	//connect(rotate, &ProgressState::Finished, this, &LeaveBaseStation::MoveToNextState);
}

void LeaveBaseStation::MoveToNextState()
{
	//connect(Robot(), &Kratos2::AprilLocationUpdate, this, &LeaveBaseStation::FoundAprilTag);
}

void LeaveBaseStation::FoundAprilTag(Eigen::Affine2d newLocation)
{
	/*
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
	*/
}

void LeaveBaseStation::NavigateToSample(QList<DetectedSample> samples)
{
	if(samples.isEmpty())
		return;

	if(mMoveInfront != nullptr)
	{
		if(mMoveInfront->mStartTime.msecsTo(QDateTime::currentDateTime()) < 5000)
			return;

		//Replan!
		mMoveInfront->SetFinished();
		mMoveInfront->deleteLater();
		mMoveInfront = nullptr;
	}

	auto sample = samples[0];
	mMoveInfront = new MoveTowardsGoalState(this);
	mMoveInfront->mGoal = samples[0].location;
	mMoveInfront->Start();
}

void LeaveBaseStation::ProportionalSteerOverSample(QList<DetectedSample> samples)
{
	const double SIDE_VELOCITY_MAX_OFFSET = 0.15;
	const double P_FORWARD_VELOCITY = 0.12;
	const double MAX_FORWARD_VELOCITY = 0.25;
	const double MAX_ANGLE = 36.0*M_PI/180.0;  // The camera FOV is 70 degrees

	cout << "sample[0]: " << samples[0].location << "\n";
	if(samples.isEmpty())
		return;

	auto sample = samples[0];
	double angle = atan2(sample.location[1], sample.location[0]);  // The way atan2 works, we want to give data as (side-to-side, forward)
	double leftVelocity = P_FORWARD_VELOCITY - SIDE_VELOCITY_MAX_OFFSET * angle / MAX_ANGLE;
	double rightVelocity = P_FORWARD_VELOCITY + SIDE_VELOCITY_MAX_OFFSET * angle / MAX_ANGLE;

	// cout << "angle (deg): " << angle*180/M_PI << "\tleftVelocity: " << leftVelocity << "\trightVelocity: " << rightVelocity << "\n";
	// cout << "sample.location.norm(): " << sample.location.norm() << "\n";

	if (finalApproach) {
		return;
	} else {
		if (sample.location.norm() > 2.75) {
			// Do proportional control
			Robot()->SetWheelVelocity(leftVelocity, rightVelocity);
		} else if (abs(angle) > 0.05) {
			Robot()->SetWheelVelocity(leftVelocity - P_FORWARD_VELOCITY, rightVelocity - P_FORWARD_VELOCITY);
		} else {
			Robot()->SetWheelVelocity(MAX_FORWARD_VELOCITY, MAX_FORWARD_VELOCITY);
			Robot()->GetTeensy()->SetCollector(1);
			finalApproach = 1;
			QTimer::singleShot(7000, this, SLOT(FinishSampleCollection()));
		}
	}
}

void LeaveBaseStation::FinishSampleCollection()
{
	Robot()->SetWheelVelocity(0.0, 0.0);
	Robot()->GetTeensy()->SetCollector(0);
	finalApproach = 0;
	SetFinished();
}