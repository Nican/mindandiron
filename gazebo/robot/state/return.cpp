#include "state.h"
#include <QtConcurrent>

using namespace Robot;
using namespace Eigen;

////////////////////
////	ReturnToStationState
////////////////////

void ReturnToStationState::Start()
{
	mStartTime = QDateTime::currentDateTime();

	//Get the -30 seconds of odometry
	auto odometry = Robot()->GetOdometryTraveledSince(mStartTime.addSecs(-10));

	if(Robot()->GetDecawave()->lastDistance < 10.0)
	{
		SetState(new ReturnLocateAprilState(this));
	} 
	else if(odometry.mPosition.norm() < 1.0)
	{
		//If we moved less than 1 meter, we need to re-align
		SetState(new ReturnRealignState(this));
	}
	else
	{
		//TODO: Rotate to the desired orientation
		SetState(new ReturnRealignState(this));
	}
}


void ReturnToStationState::MoveToNextState()
{
	std::cout << "Finished state: " << mState->metaObject()->className() << "\n";

	if(qobject_cast<ReturnRealignState*>(mState) != nullptr)
	{
		//Move towards base station
		SetState(new ReturnToBaseStationDecawave(this));
		return;
	}

	if(qobject_cast<ReturnToBaseStationDecawave*>(mState) != nullptr)
	{
		//Locate april tag
		SetState(new ReturnLocateAprilState(this));
		return;
	}

	if(qobject_cast<ReturnLocateAprilState*>(mState) != nullptr)
	{
		//Back into the base station
		SetState(new BackIntoBaseStationState(this)); //Hehe, BIBSS
		return;
	}
}

void ReturnToStationState::SetState(ProgressState* nextState)
{
	std::cout << "Moving to state: " << nextState->metaObject()->className() << "\n";

	auto oldState = mState;

	nextState->setParent(this);
 	mState = nextState;
 	connect(nextState, &ProgressState::Finished, this, &ReturnToStationState::MoveToNextState);
 	nextState->Start();

 	if(oldState != nullptr)
 	{
 		oldState->End();
 		oldState->deleteLater();
 	}
}

////////////////////
////	ReturnToBaseStationDecawave
////////////////////

ReturnToBaseStationDecawave::ReturnToBaseStationDecawave(QObject *parent) :
	ProgressState(parent)
{
}

void ReturnToBaseStationDecawave::Start()
{
	mMove = new DecawaveMoveRadialState(1, this);

	connect(Robot(), &Kratos2::AprilLocationUpdate, this, &ReturnToBaseStationDecawave::FoundAprilTag);

	mMove->Start();
}

void ReturnToBaseStationDecawave::FoundAprilTag(Eigen::Affine2d newLocation)
{
	SetFinished();
}


////////////////////
////	BackIntoBaseStationState
////	We are in front of the april tag, and we must back into it
////////////////////

void BackIntoBaseStationState::Start()
{
	connect(Robot()->GetTeensy(), &Teensy::statusUpdate, this, &BackIntoBaseStationState::TeensyStatus);
	connect(Robot(), &Kratos2::AprilLocationUpdate, this, &BackIntoBaseStationState::FoundAprilTag);
}

void BackIntoBaseStationState::TeensyStatus(TeenseyStatus status)
{
	if(mMoveInfront == nullptr)
		return;

	if(Robot()->GetDecawave()->lastDistance < 0.7)
	{
		mMoveInfront->SetFinished();
		Robot()->SetWheelVelocity(0.0, 0.0);
	}
}

void BackIntoBaseStationState::FoundAprilTag(Eigen::Affine2d newLocation)
{
	if(mMoveInfront != nullptr)
		return;

	Rotation2Dd rotation2D(0);
	rotation2D.fromRotationMatrix(newLocation.linear());

	mMoveInfront = new MoveTowardsGoalState(this);
	mMoveInfront->mGoal = Vector2d(-0.8, 0.0);
	mMoveInfront->mStartPos = newLocation.translation();
	mMoveInfront->mStartAngle = rotation2D.angle();
	mMoveInfront->mReverse = true;
	mMoveInfront->mAprilUpdates = true;
	mMoveInfront->Start();

	connect(mMoveInfront, &ProgressState::Finished, this, &BackIntoBaseStationState::DriveInto);
}

void BackIntoBaseStationState::DriveInto()
{
	Robot()->SetWheelVelocity(0.0, 0.0);
}


////////////////////
////	ReturnLocateAprilState
////	After getting close enough, locate the april tag.
////////////////////

void ReturnLocateAprilState::Start()
{
	Robot()->SetWheelVelocity(0.0, 0.0);
	connect(Robot(), &Kratos2::AprilLocationUpdate, this, &ReturnLocateAprilState::FoundAprilTag);	
}


void ReturnLocateAprilState::FoundAprilTag(Eigen::Affine2d newLocation)
{
	disconnect(Robot(), &Kratos2::AprilLocationUpdate, this, &ReturnLocateAprilState::FoundAprilTag);	
	Vector2d loc = newLocation.translation();

	std::cout << "Starting: ReturnLocateAprilState; Robot at: " << loc.transpose() << "\n"; 
	
	if(loc.x() < 0.0)
	{
		Vector2d goal(0,0);

		if(loc.y() > 0.0)
		{
			goal = Vector2d(0.0, 4.0);
		}
		else
		{
			goal = Vector2d(0.0, -4.0);
		}

		std::cout << "Moving to the side of the base station at goal: " << goal << "\n";

		TravelToWayPoint* waypoint = new TravelToWayPoint(goal, this);
		connect(waypoint, &ProgressState::Finished, this, &ReturnLocateAprilState::TravelToFront);
		waypoint->Start();

		return;
	}

	TravelToFront();

}


void ReturnLocateAprilState::TravelToFront()
{
	std::cout << "Moving to the front of the base station\n";

	TravelToWayPoint* waypoint = new TravelToWayPoint(Vector2d(5, 0), this);
	connect(waypoint, &ProgressState::Finished, this, &ReturnLocateAprilState::RealignInFront);
	waypoint->Start();
}

void ReturnLocateAprilState::RealignInFront()
{
	std::cout << "Waiting 2 seconds rotating in front";
	QTimer::singleShot(2000, this, SLOT(StartRealignInFrontRotation()));
}

void ReturnLocateAprilState::StartRealignInFrontRotation()
{
	double angle = GetEuler(Robot()->mLocation.GetEstimate().linear());

	std::cout << "ReturnLocateAprilState::StartRealignInFrontRotation at angle: " << (angle*180/M_PI) << "\n";

	//Rotate to align itself with the base station
	RotateState* rotate = new RotateState(this, -angle);
	rotate->Start();

	connect(rotate, &ProgressState::Finished, this, &ReturnLocateAprilState::FinishedRealignRotation);
}

void ReturnLocateAprilState::FinishedRealignRotation()
{
	std::cout << "Finished 'realign in front' rotation\n";
	SetFinished();	
	// QTimer::singleShot(2000, this, SLOT(SendFinishedRealignRotationSignal()));
}


////////////////////
////	ReturnRealignState
////	We are going to first check if we have an obstacle in front of us
//// 	And move a bit to figure out which direction is the base station
////////////////////

void ReturnRealignState::Start()
{
	//connect(Robot()->mPlanner, SIGNAL(ObstacleMapUpdate(ObstacleMap)), this, SLOT(ReceiveObstcles(ObstacleMap)));
	connect(&mPathFutureWatcher, SIGNAL(finished()), this, SLOT(FinishedTrajectory()));
	FinishedTrajectory();

	Robot()->SetWheelVelocity(0.0, 0.0);


}

void ReturnRealignState::ReceiveObstcles(ObstacleMap obstacleMap)
{
	disconnect(Robot()->mPlanner, SIGNAL(ObstacleMapUpdate(ObstacleMap)), this, SLOT(ReceiveObstcles(ObstacleMap)));

	auto future = QtConcurrent::run([this, obstacleMap](){
		Vector2d goal(1.0, 0.0);

		std::vector<Vector2d> reducedObstacles;

		//Todo - Add obstacles from obstacleMap into the reducedObstacles

		auto planner = std::make_shared<TrajectorySearch>(reducedObstacles, Vector2d(0.0, 0.0), 0.0, goal);
		std::size_t i = 0;
		
		for(i = 0; i < 500 && !planner->foundSolution; i++)
		{
			planner->rootNode->explore();
		}

        return planner;
	});

	mPathFutureWatcher.setFuture(future);
}

void ReturnRealignState::FinishedTrajectory()
{
	// To test: Is 1.0 right? Could we do less? 
	mMoveForward = new MoveForwardState(this, 1.0);
	connect(mMoveForward, &ProgressState::Finished, this, &ReturnRealignState::Realign);

	mMoveForward->Start();

	startDecawaveValue = Robot()->GetDecawave()->lastDistance;
	std::cout << "Start Decawave: " << startDecawaveValue << "\n";
	std::cout << "Finished calculating trajectory\n";
}

void ReturnRealignState::Realign()
{
	//auto odometry = Robot()->GetOdometryTraveledSince(mMoveForward->mStartTime);
	//auto odometryDistance = odometry.mDistanceTraveled;

	double endDecawaveValue = Robot()->GetDecawave()->lastDistance;
	std::cout << "Start Decawave: " << startDecawaveValue << "\n";
	std::cout << "End Decawave: " << endDecawaveValue << "\n";
	double diff = startDecawaveValue - endDecawaveValue;

	std::cout << "Realigning with diff: " << diff << "\n";

	if(std::abs(diff) <= 0.33)
	{
		//We only moved 0.33 out of 1 meter in direction of the base station. 
		cout << "Decawave says we must walking somewhat along the circle\n";
		//Rotate 90 degrees
		RotateState* rotate = new RotateState(this, M_PI/2);
		connect(rotate, &ProgressState::Finished, this, &ReturnRealignState::FinishRotate);
		rotate->Start();

		return;
	}

	if(diff < 0)
	{
		cout << "Decawave says we are going completely in the wrong direction\n";
		RotateState* rotate = new RotateState(this, M_PI);
		connect(rotate, &ProgressState::Finished, this, &ReturnRealignState::FinishRotate);
		rotate->Start();

		return;
	}

	std::cout << "We are finished aligning the robot towards base with Decawave! :)\n";

	//We are going somewhat in the correct direction
	Robot()->SetWheelVelocity(0, 0);
	SetFinished();
}

void ReturnRealignState::FinishRotate()
{
	//connect(Robot()->mPlanner, SIGNAL(ObstacleMapUpdate(ObstacleMap)), this, SLOT(ReceiveObstcles(ObstacleMap)));
	FinishedTrajectory();
}