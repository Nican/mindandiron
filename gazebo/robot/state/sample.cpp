#include "state.h"
#include <QtConcurrent>

using namespace Robot;
using namespace Eigen;




////////////////////
////	Level1State
////	1. Start at the base station
////	2. Leave base station, and turn 180 degrees
////	3. Travel behind the base station to find samples
////////////////////

void Level1State::Start()
{
	leaveBase = new LeaveBaseStation(this);
	connect(leaveBase, &ProgressState::Finished, this, &Level1State::StartToTravelBehind);
	leaveBase->Start();
}

void Level1State::StartToTravelBehind()
{
	leaveBase->deleteLater();
	leaveBase = nullptr;

	mMoveInfront = new MoveTowardsGoalState(this);
	mMoveInfront->mGoal = Vector2d(0.0, 4.0);
	mMoveInfront->mReverse = false;
	mMoveInfront->mAprilUpdates = true;
	
	connect(mMoveInfront, &ProgressState::Finished, this, &Level1State::MoveForwardBehind);
	mMoveInfront->Start();
}

void Level1State::MoveForwardBehind()
{
	auto estimate = Robot()->mLocation.GetEstimate();

	mMoveInfront->deleteLater();
	mMoveInfront = nullptr;

	Rotation2Dd rotation2D(0);
	rotation2D.fromRotationMatrix(estimate.linear());

	mMoveInfront = new MoveTowardsGoalState(this);
	mMoveInfront->mGoal = Vector2d(-6.0, 6.0);
	mMoveInfront->mStartPos = estimate.translation();
	mMoveInfront->mStartAngle = rotation2D.angle();
	mMoveInfront->mReverse = false;
	mMoveInfront->mAprilUpdates = true;
	
	connect(mMoveInfront, &ProgressState::Finished, this, &Level1State::StartExplore);
	//mMoveInfront->Start();
}

void Level1State::StartExplore()
{
	mMoveInfront->deleteLater();
	mMoveInfront = nullptr;

	mExplore = new ExploreState(this);
	//connect(mExplore, &ProgressState::Finished, this, &Level1State::EndExplore);
	mExplore->Start();

}


////////////////////
////	ExploreState
////////////////////


void ExploreState::Start()
{
	std::cout << "Starting explore state\n";
	StartNavigation();
}

void ExploreState::StartNavigation()
{
	if(mGoalMove != nullptr)
	{
		mGoalMove->deleteLater();
		mGoalMove = nullptr;
	}

	//Look at the current decawave value
	//Decide if we should keep moving, or head back

	if(Robot()->GetDecawave()->lastDistance > 80)
	{
		std::cout << "Ending explore. We are too far away from the decawave\n";
		SetFinished();
		return;
	}

	mGoalMove = new MoveTowardsGoalState(this);
	mGoalMove->mGoal = Vector2d(2.0, 0.0);
	mGoalMove->mReverse = false;
	mGoalMove->mAprilUpdates = false;

	connect(mGoalMove, &MoveTowardsGoalState::Failed, this, &ExploreState::FailedNavigation);
	connect(mGoalMove, &ProgressState::Finished, this, &ExploreState::MoveToNextState);
	mGoalMove->Start();

}

void ExploreState::MoveToNextState()
{
	if(Robot()->GetSampleDetection()->mLastDetection.size() > 0)
	{
		std::cout << "Navigating to sample\n";
		auto navigate = new NavigateToSample(this);
		connect(navigate, &ProgressState::Finished, this, &ExploreState::StartNavigation);
		navigate->Start();
	}
	else
	{
		std::cout << "Restarting navigation\n";
		StartNavigation();
	}
}


void ExploreState::FailedNavigation()
{
	auto estimate = Robot()->mLocation.GetEstimate();

	mGoalMove->deleteLater();
	mGoalMove = nullptr;

	Rotation2Dd rotation2D(0);
	rotation2D.fromRotationMatrix(estimate.linear());
	double rotateDirection = M_PI / 2;

	if((estimate.linear() * Vector2d(1.0, 0.0)).x() < 0)
	{
		rotateDirection *= -1;
	}

	std::cout << "Failed navigation. Rotating " << rotateDirection * 180 / M_PI << "degrees\n";

	ProgressState* newState = new RotateState(this, rotateDirection);
	connect(newState, &ProgressState::Finished, this, &ExploreState::StartNavigation);
	newState->Start();
}


////////////////////
////	NavigateToSample
////////////////////

void NavigateToSample::Start()
{
	connect(Robot()->GetSampleDetection(), &SampleDetection::SampleDetected, this, &NavigateToSample::ProportionalSteerOverSample);
	finalApproach = 0;
}

void NavigateToSample::ProportionalSteerOverSample(QList<DetectedSample> samples)
{
	if(IsFinished() || finalApproach)
		return;

	const double SIDE_VELOCITY_MAX_OFFSET = 0.15;
	const double P_FORWARD_VELOCITY = 0.12;
	const double MAX_FORWARD_VELOCITY = 0.25;
	const double MAX_ANGLE = 36.0*M_PI/180.0;  // The camera FOV is 70 degrees

	// cout << "sample[0]: " << samples[0].location.transpose() << "\n";
	if(samples.isEmpty())
		return;

	auto sample = samples[0];
	sample.location[1] = sample.location[1] - 0.2;
	double angle = atan2(sample.location[1], sample.location[0]);  // The way atan2 works, we want to give data as (side-to-side, forward)
	double leftVelocity = P_FORWARD_VELOCITY - SIDE_VELOCITY_MAX_OFFSET * angle / MAX_ANGLE;
	double rightVelocity = P_FORWARD_VELOCITY + SIDE_VELOCITY_MAX_OFFSET * angle / MAX_ANGLE;

	cout << "angle (deg): " << angle*180/M_PI << "\tleftVelocity: " << leftVelocity << "\trightVelocity: " << rightVelocity << "\n";
	cout << "sample.location.norm(): " << sample.location.norm() << "\n";

	if (sample.location.norm() > 1.85) {
		// Do proportional control
		Robot()->SetWheelVelocity(leftVelocity, rightVelocity);
	} else if (abs(angle) > 0.05) {
		Robot()->SetWheelVelocity(leftVelocity - P_FORWARD_VELOCITY, rightVelocity - P_FORWARD_VELOCITY);
	} else {
			Robot()->SetWheelVelocity(MAX_FORWARD_VELOCITY, MAX_FORWARD_VELOCITY);
			Robot()->GetTeensy()->SetCollector(1);
			finalApproach = 1;
			QTimer::singleShot(5000, this, SLOT(MomentarilyHaltRobot()));
	}
}

void NavigateToSample::MomentarilyHaltRobot()
{
	Robot()->SetWheelVelocity(0.0, 0.0);
	QTimer::singleShot(1000, this, SLOT(BackUpToCollectSample()));
}

void NavigateToSample::BackUpToCollectSample()
{
	Robot()->SetWheelVelocity(-0.2, -0.2);
	QTimer::singleShot(7000, this, SLOT(LongHaltForRobot()));
}

void NavigateToSample::LongHaltForRobot()
{
	Robot()->SetWheelVelocity(0.0, 0.0);
	QTimer::singleShot(20000, this, SLOT(FinishSampleCollection()));
}

void NavigateToSample::FinishSampleCollection()
{
	Robot()->SetWheelVelocity(0.0, 0.0);
	Robot()->GetTeensy()->SetCollector(0);
	finalApproach = 0;
	SetFinished();
}


////////////////////
////	LeaveBaseStation
////////////////////

void LeaveBaseStation::Start()
{
	auto move = new TravelToWayPoint(Vector2d(2.0, 0.0), this);
	move->Start();

	connect(move, &ProgressState::Finished, this, &LeaveBaseStation::MoveToRotate);
}

void LeaveBaseStation::MoveToRotate()
{
	// auto rotate = new RotateState(this, M_PI/2);
	// rotate->Start();
	auto move = new TravelToWayPoint(Vector2d(0.0, 4.0), this);
	move->Start();

	connect(move, &ProgressState::Finished, this, &LeaveBaseStation::MoveToNextState);
}

void LeaveBaseStation::MoveToNextState()
{
	SetFinished();
}
