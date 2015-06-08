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

	auto move = new TravelToWayPoint(Vector2d(-10.0, 10.0), this);
	move->Start();
	
	connect(move, &ProgressState::Finished, this, &Level1State::StartExplore);
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

	connect(Robot()->GetSampleDetection(), &SampleDetection::SampleDetected, this, &ExploreState::StartSampleCollection);
	connect(Robot()->mPlanner, &TrajectoryPlanner2::ObstacleMapUpdate, this, &ExploreState::ObstacleMapUpdate);
}

void ExploreState::StartNavigation()
{
	mExploreOut = new DecawaveMoveRadialState(0, this);
	//connect(mExploreOut, &ProgressState::Finished, this, &ExploreState::RestartNavigation);
	mExploreOut->Start();
}

void ExploreState::StartSampleCollection(QList<DetectedSample> samples)
{
	if(mExploreOut == nullptr)
	{
		//We are already trying to collect the sample
		return;
	}

	mExploreOut->SetFinished();
	mExploreOut->deleteLater();
	mExploreOut = nullptr;

	mSampleNavigation = new NavigateToSample(this);
	connect(mSampleNavigation, &ProgressState::Finished, this, &ExploreState::RestartNavigation);
	mSampleNavigation->Start();

}

void ExploreState::RestartNavigation()
{
	if(mSampleNavigation != nullptr)
	{
		mSampleNavigation->deleteLater();
		mSampleNavigation = nullptr;
	}

	StartNavigation();
}

void ExploreState::ObstacleMapUpdate(ObstacleMap obstacleMap)
{
	bool inRadius = false;

	for(auto& pt : obstacleMap.mObstacleList)
	{
		if(pt.norm() <= 3.0){
			inRadius = true;
			break;
		}
	}

	if(inRadius)
	{
		int left = 0;
		int right = 0;

		for(auto& pt : obstacleMap.mObstacleList)
		{
			if(pt.y() > 0)
				left++;
			else
				right++;
		}

		auto rotate = new RotateState(this, (left > right) ? (M_PI/2) : (-M_PI/2) );
		connect(rotate, &ProgressState::Finished, this, &ExploreState::FinishRotate);
		rotate->Start();
	}
}

void ExploreState::FinishRotate()
{

}


void ExploreState::FailedNavigation()
{
	/*
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
	*/
}


////////////////////
////	NavigateToSample
////////////////////

void NavigateToSample::Start()
{
	connect(Robot()->GetSampleDetection(), &SampleDetection::SampleDetected, this, &NavigateToSample::ProportionalSteerOverSample);
	connect(Robot()->GetTeensy(), &Teensy::statusUpdate, this, &NavigateToSample::TeensyStatus);
	finalApproach = 0;
	mLastSampleSeen = QDateTime::currentDateTime();
}

void NavigateToSample::TeensyStatus(TeenseyStatus status)
{
	if (!finalApproach) {
		if(std::abs(this->mLastSampleSeen.msecsTo(QDateTime::currentDateTime())) > 4000)
			FinishSampleCollection();
	}
}

void NavigateToSample::ProportionalSteerOverSample(QList<DetectedSample> samples)
{
	mLastSampleSeen = QDateTime::currentDateTime();

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
