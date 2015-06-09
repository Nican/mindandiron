#include "state.h"
#include <QtConcurrent>
#include <QSqlError>
#include <QSqlQuery>

using namespace Robot;
using namespace Eigen;


static const double SAMPLE_RANGE = 64.0; //+- 10m 
static const double SAMPLE_SEARCH_RANGE = 10.0;
static const double SAMPLE_ANGLE = 135.0 * M_PI / 180.0;

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
	//StartToTravelBehind();

	//RotateToTarget();
	//StartExplore();
}

void Level1State::StartToTravelBehind()
{
	if(leaveBase != nullptr)
	{
		leaveBase->deleteLater();
		leaveBase = nullptr;
	}

	auto move = new TravelToWayPoint(Vector2d(-5.0, 5.0), this);
	move->mTolerance = 2.0;
	move->Start();
	
	//connect(move, &ProgressState::Finished, this, &Level1State::RotateToTarget);
	connect(move, &ProgressState::Finished, this, &Level1State::StartExplore);
}

void Level1State::RotateToTarget()
{
	auto move = new AprilRotateState(this, SAMPLE_ANGLE); //163
	move->mTolerance = 10.0 * M_PI / 180.0;
	connect(move, &ProgressState::Finished, this, &Level1State::StartExplore);
	move->Start();
}

void Level1State::StartExplore()
{
	if(mMoveInfront != nullptr)
	{
		mMoveInfront->deleteLater();
		mMoveInfront = nullptr;
	}

	mExplore = new ExploreState(this);
	connect(mExplore, &ProgressState::Finished, this, &Level1State::EndExplore);
	mExplore->Start();
}

void Level1State::EndExplore()
{
	if(!mExplore->mSuccessCollect)
	{
		std::cout << "We failed to find the sample. :( \n";
		std::cout << "\t Heading back to try to search for the sample again\n";
		//Head back and ExploreAgain
		HeadBackAndExploreAgain* explore = new HeadBackAndExploreAgain(this);
		explore->Start();
	}
	else
	{
		SetFinished();
	}
}


////////////////////
////	ExploreState
////////////////////

ExploreState::ExploreState(QObject *parent) : 
	ProgressState(parent)
	,mExploreOut(nullptr)
	,mSampleNavigation(nullptr)
	,mNextExploreRadius(SAMPLE_RANGE-SAMPLE_SEARCH_RANGE)
{
}


void ExploreState::Start()
{
	std::cout << "Starting explore state\n";

	connect(Robot()->GetSampleDetection(), &SampleDetection::SampleDetected, this, &ExploreState::StartSampleCollection);
	connect(Robot()->mPlanner, &TrajectoryPlanner2::ObstacleMapUpdate, this, &ExploreState::ObstacleMapUpdate);
	connect(Robot()->GetDecawave(), &Decawave::statusUpdate, this, &ExploreState::DecawaveUpdate);
	connect(Robot(), &Kratos2::AprilLocationUpdate, this, &ExploreState::FoundAprilTag);
	connect(Robot(), &Kratos2::pauseUpdate, this, &ExploreState::pauseUpdate);


	mLastAprilTag = QDateTime::currentDateTime();
	mGoingOutUsingApril = true;
	mSuccessCollect = false;
	mLastRotation = M_PI/2;

	StartNavigation();

	QTimer* timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(CheckStuck()));
	timer->start(20 * 1000);

}

void ExploreState::FoundAprilTag(Eigen::Affine2d newLocation)
{
	mLastAprilTag = QDateTime::currentDateTime();
	//std::cout << QDateTime::currentDateTime().toString("hh:mm:ss.zzz").toStdString() <<  ": Got update position from april tag\n";
}

void ExploreState::CheckStuck()
{
	//We are going to check if we are stuck by looking at the last 4 minutes of decawave data
	//And checked if we have moved more than 1 meter

	QSqlQuery query(Robot()->mSensorLog->mDb);
	query.prepare("SELECT distance FROM decawaveLog WHERE timestamp > :startTime");
	query.bindValue(":startTime", (QDateTime::currentDateTime().addSecs(-60 * 4)).toMSecsSinceEpoch());

	if(!query.exec())
	{
		std::cout << "ExploreState::CheckStuck SQL= " <<  query.lastError().text().toStdString() << std::endl;
		return;
	}

	double min = 1000.0;
	double max = 0;

	while (query.next()) {
		double val = query.value(0).toDouble();

		if(val < min)
			min = val;

		if(val > max)
			max = val;
	}

	if(std::abs(max-min) <= 1.0)
	{
		std::cout << "Looks like we are stuck! Trying to move back.\n";

		//Fuck.. We do not seem to be moving
		ResetExplore();

		if(mSampleNavigation != nullptr)
			mSampleNavigation->SetFinished();

		mExploreOut = new MoveForwardState(this, -3.0);
		connect(mExploreOut, &ProgressState::Finished, this, &ExploreState::RotateBack);
		mExploreOut->Start();
	}
}

void ExploreState::pauseUpdate(bool paused)
{
	//GOD DAMN JUDGES - Stop playing with the pause switch

	if(paused)
	{
		ResetExplore();
	}
	else
	{
		StartNavigation();
	}
}

void ExploreState::DecawaveUpdate(double value)
{
	if(mGoingOutUsingApril == true && std::abs(mLastAprilTag.msecsTo(QDateTime::currentDateTime())) > 14000)
	{
		mGoingOutUsingApril = false;
		ResetExplore();
		StartNavigation();
	}

	//Do not switch states if we are already trying to grab a sample
	if(mExploreOut == nullptr)
		return;

	if(value >= mNextExploreRadius)
	{
		mNextExploreRadius = value + 5.0;
		ResetExplore();

		auto rotate = new RotateScanForSample(this);
		connect(rotate, &ProgressState::Finished, this, &ExploreState::StartNavigation);
		rotate->Start();
	}

	if(value >= (SAMPLE_RANGE + SAMPLE_SEARCH_RANGE))
	{
		SetFinished();
	}

}

void ExploreState::ResetExplore()
{
	if(mExploreOut == nullptr)
		return;

	mExploreOut->SetFinished();
	mExploreOut->deleteLater();
	mExploreOut = nullptr;
}

void ExploreState::StartNavigation()
{
	std::cout << "Starting navigation (Decawave: "<< Robot()->GetDecawave()->lastDistance <<"/"<<mGoingOutUsingApril<<")\n";

	if(mGoingOutUsingApril)
	{
		auto goal = Vector2d(std::cos(SAMPLE_ANGLE), std::sin(SAMPLE_ANGLE)) * (SAMPLE_RANGE+15.0);
		TravelToWayPoint* move = new TravelToWayPoint(goal, this);
		move->mTolerance = 5.0;
		mExploreOut = move;
		mExploreOut->Start();

		//Is this even possible?
		connect(mExploreOut, &ProgressState::Finished, this, &ExploreState::FinishedFarGoal);
	}
	else
	{
		ReturnRealignState* realign = new ReturnRealignState(this);
		realign->bGoingOut = true;
		connect(realign, &ProgressState::Finished, this, &ExploreState::StartDecawaveExplore);
		realign->Start();		
	}
}

void ExploreState::StartDecawaveExplore()
{
	//This state never finishes
	mExploreOut = new DecawaveMoveRadialState(0, this);
	mExploreOut->Start();
}

void ExploreState::FinishedFarGoal()
{
	std::cout << "Finishing reaching far goal!\n";
	ResetExplore();
	StartNavigation();
}

void ExploreState::StartSampleCollection(QList<DetectedSample> samples)
{
	if(IsFinished())
		return;

	if(mExploreOut == nullptr)
	{
		//`already trying to collect the sample
		return;
	}

	if(samples.isEmpty())
		return;

	std::cout << "Start to navigate to sample\n";
	mSampleNavigation = new NavigateToSample(this);
	connect(mSampleNavigation, &ProgressState::Finished, this, &ExploreState::FinishCollect);
	mSampleNavigation->Start();

	ResetExplore();
}

void ExploreState::FinishCollect()
{
	std::cout << "STATUS OF COLLECTING SAMPLE: ";
	std::cout << (mSampleNavigation->bSuccess ? "Success" : "-- FAILED --");

	if(mSampleNavigation->bSuccess)
	{
		mSuccessCollect = true;
	}

	if(mSampleNavigation != nullptr)
	{
		mSampleNavigation->deleteLater();
		mSampleNavigation = nullptr;
	}

	//Do one last scan before finishing
	ResetExplore();
	auto rotate = new RotateScanForSample(this);
	connect(rotate, &ProgressState::Finished, this, &ExploreState::FinishPostCollecting);
	rotate->Start();
}

void ExploreState::FinishPostCollecting()
{
	if(IsFinished())
		return;

	std::cout << "Finished post colleting (" << mSampleNavigation << ")\n";

	if(mSampleNavigation != nullptr)
		return;

	if(mSuccessCollect)
	{
		SetFinished();
	}
	else
	{
		StartNavigation();
	}
}

void ExploreState::ObstacleMapUpdate(ObstacleMap obstacleMap)
{
	if(IsFinished())
		return;

	if(mExploreOut == nullptr)
	{
		//We are already trying to collect the sample
		return;
	}
	
	bool inRadius = false;

	for(auto& pt : obstacleMap.mObstacleList)
	{
		if(pt.norm() <= 6.0){
			inRadius = true;
			break;
		}
	}

	if(inRadius)
	{
		std::cout << " !!!! !!!! Found obstacles !!!! !!!! :(\n";
		ResetExplore();

		int left = 0;
		int right = 0;

		for(auto& pt : obstacleMap.mObstacleList)
		{
			if(pt.y() > 0)
				left++;
			else
				right++;
		}

		std::cout << "Count: Left: " << left << "\tRight: " << right << " " << (left+right) << "\n";

		if((left+right) < 80)
			return;

		mLastRotation =  (left < right) ? (M_PI*0.45) : (-M_PI*0.45);
		auto rotate = new RotateState(this, mLastRotation);
		connect(rotate, &ProgressState::Finished, this, &ExploreState::FinishRotate);
		rotate->Start();
	}
}

void ExploreState::FinishRotate()
{
	if(IsFinished())
		return;

	std::cout << "Finished to rotate\n";
	mExploreOut = new MoveForwardState(this, 3.0);
	connect(mExploreOut, &ProgressState::Finished, this, &ExploreState::RotateBack);
	mExploreOut->Start();
}

void ExploreState::RotateBack()
{
	if(IsFinished())
		return;

	auto rotate = new RotateState(this, -mLastRotation);
	connect(rotate, &ProgressState::Finished, this, &ExploreState::StartNavigation);
	rotate->Start();
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
	bSuccess = false;
}

void NavigateToSample::TeensyStatus(TeenseyStatus status)
{
	if (!finalApproach) {
		auto timeDiff = std::abs(this->mLastSampleSeen.msecsTo(QDateTime::currentDateTime()));
		std::cout << "Time diff since last seen sample " << timeDiff << "\n";

		if(timeDiff > 2000)
		{
			bSuccess = false;
			FinishSampleCollection();
		}
	}
}

void NavigateToSample::ProportionalSteerOverSample(QList<DetectedSample> samples)
{
	if(samples.isEmpty())
		return;

	mLastSampleSeen = QDateTime::currentDateTime();

	if(IsFinished() || finalApproach)
		return;

	const double SIDE_VELOCITY_MAX_OFFSET = 0.15;
	const double P_FORWARD_VELOCITY = 0.12;
	const double MAX_FORWARD_VELOCITY = 0.25;
	const double MAX_ANGLE = 36.0*M_PI/180.0;  // The camera FOV is 70 degrees

	// cout << "sample[0]: " << samples[0].location.transpose() << "\n";

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
	bSuccess = true;
}

void NavigateToSample::FinishSampleCollection()
{
	std::cout << "Finished to collect sample - IsSuccess: " << bSuccess << "\n";
	Robot()->SetWheelVelocity(0.0, 0.0);
	Robot()->GetTeensy()->SetCollector(0);
	finalApproach = 0;
	SetFinished();
}

////////////////////
////	RotateScanForSample
////////////////////

void RotateScanForSample::Start()
{
	mRotate = new RotateState(this, M_PI/2);
	connect(mRotate, &ProgressState::Finished, this, &RotateScanForSample::FinishRotate1);
	mRotate->Start();
}

void RotateScanForSample::FinishRotate1()
{
	mRotate = new RotateState(this, -M_PI);
	connect(mRotate, &ProgressState::Finished, this, &RotateScanForSample::FinishRotate2);
	mRotate->Start();
}

void RotateScanForSample::FinishRotate2()
{
	mRotate = new RotateState(this, M_PI/2);
	connect(mRotate, &ProgressState::Finished, this, &RotateScanForSample::FinishRotate3);
	mRotate->Start();
}

void RotateScanForSample::FinishRotate3()
{
	SetFinished();
}


////////////////////
////	LeaveBaseStation
////////////////////

void LeaveBaseStation::Start()
{
	auto move = new TravelToWayPoint(Vector2d(8.0, 0.0), this);
	move->mTolerance = 1.0;
	move->Start();

	connect(move, &ProgressState::Finished, this, &LeaveBaseStation::MoveToRotate);
}

void LeaveBaseStation::MoveToRotate()
{
	// auto rotate = new RotateState(this, M_PI/2);
	// rotate->Start();
	auto move = new TravelToWayPoint(Vector2d(0.0, 5.0), this);
	move->mTolerance = 1.0;
	move->Start();

	connect(move, &ProgressState::Finished, this, &LeaveBaseStation::MoveToNextState);
}

void LeaveBaseStation::MoveToNextState()
{
	SetFinished();
}



////////////////////
////	HeadBackAndExploreAgain
////////////////////

void HeadBackAndExploreAgain::Start()
{
	std::cout << "WE GOT TOO FAR AWAY -- We are going to head back and search again.";

	auto realign = new ReturnRealignState(this);
	connect(realign, &ProgressState::Finished, this, &HeadBackAndExploreAgain::StartHeadingBack);
	realign->Start();
}

void HeadBackAndExploreAgain::StartHeadingBack()
{
	mMove = new DecawaveMoveRadialState(1, this);
	connect(Robot(), &Kratos2::AprilLocationUpdate, this, &HeadBackAndExploreAgain::FoundAprilTag);
	mMove->Start();
}


void HeadBackAndExploreAgain::FoundAprilTag(Eigen::Affine2d newLocation)
{
	if(IsFinished())
		return;

	if(mWaypoint != nullptr)
		return;

	if(newLocation.translation().norm() > 30.0)
		return;

	auto goal = Vector2d(std::cos(SAMPLE_ANGLE), std::sin(SAMPLE_ANGLE)) * (SAMPLE_RANGE/2);

	mWaypoint = new TravelToWayPoint(goal, this);
	//mWaypoint->mReverse = true;
	mWaypoint->mTolerance = 5.0;
	connect(mWaypoint, &ProgressState::Finished, this, &HeadBackAndExploreAgain::ExploreAgain);
	mWaypoint->Start();
}

void HeadBackAndExploreAgain::ExploreAgain()
{
	SetFinished();
}