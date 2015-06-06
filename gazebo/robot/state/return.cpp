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

	if(Robot()->GetDecawave()->lastDistance < 10.0 || true)
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
		SetState(new ReturnMoveBackState(this));
		return;
	}

	if(qobject_cast<ReturnMoveBackState*>(mState) != nullptr)
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
	
}

void BackIntoBaseStationState::FoundAprilTag(Eigen::Affine2d newLocation)
{
	if(mMoveInfront != nullptr)
		return;

	Rotation2Dd rotation2D(0);
	rotation2D.fromRotationMatrix(newLocation.linear());

	mMoveInfront = new MoveTowardsGoalState(this);
	mMoveInfront->mGoal = Vector2d(-0.5, 0.0);
	mMoveInfront->mStartPos = newLocation.translation();
	mMoveInfront->mStartAngle = rotation2D.angle();
	mMoveInfront->mReverse = true;
	mMoveInfront->mAprilUpdates = true;
	mMoveInfront->Start();

	connect(mMoveInfront, &ProgressState::Finished, this, &BackIntoBaseStationState::DriveInto);
}

void BackIntoBaseStationState::DriveInto()
{

}


////////////////////
////	ReturnLocateAprilState
////	After getting close enough, locate the april tag.
////////////////////

void ReturnLocateAprilState::Start()
{
	mMoveInfront = nullptr;
	
	mRotate = new RotateState(this, M_PI);
	connect(mRotate, &ProgressState::Finished, this, &ReturnLocateAprilState::FinishRotate);
	// mRotate->Start();

	mFinishedRotatingTime = QDateTime::currentDateTime();

	mStartTime = QDateTime::currentDateTime();

	//Can we see the april tag?
	connect(Robot(), &Kratos2::AprilLocationUpdate, this, &ReturnLocateAprilState::FoundAprilTag);
}

void ReturnLocateAprilState::TeensyStatus(TeenseyStatus status)
{
	if(!mFinishedRotatingTime.isValid())
		return;

	//We are going to wait for a maximun of 30 seconds to find the april tags
	if(mStartTime.msecsTo(QDateTime::currentDateTime()) > 15 * 1000){
		//Move around a bit and try to find the april tag again
		return;
	}

}

void ReturnLocateAprilState::FinishRotate()
{
	//Robot()->SetWheelVelocity(0.0, 0.0);

	mFinishedRotatingTime = QDateTime::currentDateTime();
}

void ReturnLocateAprilState::FoundAprilTag(Eigen::Affine2d newLocation)
{
	Robot()->SetWheelVelocity(0.0, 0.0);

	if(mRotate->IsValid())
	{
		std::cout << "Rotating is still valid. Killing it!\n";
		mRotate->SetFinished();
	}

	//std::cout << "ReturnLocateAprilState::FoundAprilTag";

	if(mMoveInfront != nullptr)
		return;

	Rotation2Dd rotation2D(0);
	rotation2D.fromRotationMatrix(newLocation.linear());

	mMoveInfront = new MoveTowardsGoalState(this);
	mMoveInfront->mGoal = Vector2d(3.0, 0.0);
	mMoveInfront->mStartPos = newLocation.translation();
	mMoveInfront->mStartAngle = rotation2D.angle();
	mMoveInfront->mReverse = true;
	mMoveInfront->mAprilUpdates = true;
	mMoveInfront->Start();

	connect(mMoveInfront, &ProgressState::Finished, this, &ReturnLocateAprilState::RealignInFront);
}

void ReturnLocateAprilState::RealignInFront()
{
	QTimer::singleShot(2000, this, SLOT(StartRealignInFrontRotation()));
}

void ReturnLocateAprilState::StartRealignInFrontRotation()
{
	auto odometry = Robot()->GetOdometryTraveledSince(QDateTime::currentDateTime().addSecs(-2), QDateTime::currentDateTime(), true);
	cout << "Rotating angle: " << -odometry.mTheta << " rad\n";

	//Rotate to align itself with the base station
	RotateState* rotate = new RotateState(this, -odometry.mTheta);
	rotate->Start();

	connect(rotate, &ProgressState::Finished, this, &ReturnLocateAprilState::FinishedRealignRotation);
}

void ReturnLocateAprilState::FinishedRealignRotation()
{
	std::cout << "Finished 'realign in front' rotation\n";
	SetFinished();	
	// QTimer::singleShot(2000, this, SLOT(SendFinishedRealignRotationSignal()));
}

// void ReturnLocateAprilState::SendFinishedRealignRotationSignal()
// {
// 	std::cout << "Finished 'realign in front' rotation\n";
// 	SetFinished();
// }

////////////////////
////	ReturnMoveBackState
////	Move forward while attempting to minimize the distance between odometry and decawave distance traveled
////////////////////

void ReturnMoveBackState::Start()
{
	mLastPerformance = 0.0;
	returnType = ReturnMoveEnum::FORWARD;
	Robot()->SetWheelVelocity(0.2, 0.2);

	Reset();
	connect(Robot()->GetTeensy(), &Teensy::statusUpdate, this, &ReturnMoveBackState::TeensyStatus);
	connect(Robot()->GetDecawave(), &Decawave::statusUpdate, this, &ReturnMoveBackState::DecawaveUpdate);
}

void ReturnMoveBackState::DecawaveUpdate(double value)
{
	mLastReadings[lastReadingId] = value;

	if(lastReadingId == mLastReadings.size() - 1)
	{
		UpdateDirection(GetAverageDecawaveVelocity(), 1, 0, 0);
		mLastReadings[0] = mLastReadings[lastReadingId];
		lastReadingId = 1;
	}
	else
	{
		lastReadingId++;
	}

	// TODO: CHANGE TO SOMETHING LARGER - THIS IS FOR TESTING
	if(value < 5.0){
		Robot()->SetWheelVelocity(0.0, 0.0);
		SetFinished();
		return;
	}
}

// Should return average velocity in m/s
double ReturnMoveBackState::GetAverageDecawaveVelocity()
{
	double averageValue = 0.0;
	for(std::size_t i = 1; i < mLastReadings.size(); i++)
	{
		double diff = mLastReadings[i-1] - mLastReadings[i];
		// Decawave updates at about 0.8 seconds
		averageValue += diff / 0.8;  // THIS IS MORE ACCURATE THAN MEASURING TIME DIFF
									 // There's some weird data buffering going on
	}

	// cout << "Average m/s value is: " << averageValue / (double) mLastReadings.size() << "\n";
	return averageValue / (double) mLastReadings.size();
}

//////////////////////////////////////////////////////////////////////////
// Going In OR Going Out
//////////////////////////////////////////////////////////////////////////
void ReturnMoveBackState::UpdateDirection(double averageVelocity, int in, int out)
{
	if (!in && !out)  // Circle not implemented yet
		return;

	// Note to Henrique: if any velocities are positive/negative when they
	// shouldn't be, change state to realign!

	static double lastVelocity = 0.0;  // Stores last average velocity for comparison
	const double MAX_SIDE_VELOCITY = 0.075;
	const double MAX_FORWARD_VELOCITY = 0.275;
	static double maxSeenVelocity = MAX_FORWARD_VELOCITY;

	if ((out && averageVelocity < maxSeenVelocity) ||
	    (in  && averageVelocity > maxSeenVelocity))
	{
			maxSeenVelocity = averageVelocity;
	}
	// cout << "Average m/s value is: " << averageVelocity << "\n";
	// cout << "Last m/s value is: " << lastVelocity << "\n";

	// If we have to, we could make sideways motion move more slowly to not get off course
	double forwardVelocity = MAX_FORWARD_VELOCITY;

	// The "get average velocity" function overestimates a little, or the
	// Teensy overdoes the velocity. Hard to know. Test whether the multiplier
	// (currently 1.0) works on grass, it's good on concrete
	if ((out && averageVelocity <= (-1.0 * MAX_FORWARD_VELOCITY)) ||
		(in  && averageVelocity >= ( 1.0 * MAX_FORWARD_VELOCITY)))
	{
		std::cout << "\tDoing well, moving FORWARD\n";
		Robot()->SetWheelVelocity(forwardVelocity, forwardVelocity);
		lastVelocity = averageVelocity;  // For comparison to next computed velocity
		return;
	}

	if ((out && averageVelocity <= lastVelocity) ||
		(in  && averageVelocity >= lastVelocity))
	{
		cout << "State improved from last run, continuing on without DIRECTION change\n";
	} else if (returnType == ReturnMoveEnum::RIGHT) {
		std::cout << "\tMoving LEFT\n";
		returnType = ReturnMoveEnum::LEFT;
	} else {
		std::cout << "\tMoving RIGHT\n";
		returnType = ReturnMoveEnum::RIGHT;
	}

	double proportionalReaction = (1.15 - averageVelocity / maxSeenVelocity) * MAX_SIDE_VELOCITY;
	// cout << "forwardVelocity: " << forwardVelocity << "\n";
	// cout << "proportionalReaction: " << proportionalReaction << "\n";
	CommandVelocity(forwardVelocity, proportionalReaction)
	lastVelocity = averageVelocity;  // For comparison to next computed velocity
}


void ReturnMoveBackState::UpdateDirectionCircular(double averageVelocity)
{
	static double lastVelocity = 0.0;  // Stores last average velocity for comparison
	const double MAX_SIDE_VELOCITY = 0.075;
	const double MAX_FORWARD_VELOCITY = 0.275;
	// cout << "Average m/s value is: " << averageVelocity << "\n";
	// cout << "Last m/s value is: " << lastVelocity << "\n";

	// If we have to, we could make sideways motion move more slowly to not get off course
	double forwardVelocity = MAX_FORWARD_VELOCITY;

	if (abs(averageVelocity) <= (0.05 * MAX_FORWARD_VELOCITY))
	{
		std::cout << "\tDoing well, moving FORWARD\n";
		Robot()->SetWheelVelocity(forwardVelocity, forwardVelocity);
		lastVelocity = averageVelocity;  // For comparison to next computed velocity
		return;
	}

	if (((averageVelocity < 0 && lastVelocity < 0) ||
		 (averageVelocity > 0 && lastVelocity > 0)) &&
		abs(averageVelocity) < abs(lastVelocity))
	{
		cout << "State improved from last run, continuing on without DIRECTION change\n";
	} else if (returnType == ReturnMoveEnum::RIGHT) {
		std::cout << "\tMoving LEFT\n";
		returnType = ReturnMoveEnum::LEFT;
	} else {
		std::cout << "\tMoving RIGHT\n";
		returnType = ReturnMoveEnum::RIGHT;
	}

	double proportionalReaction = averageVelocity / MAX_FORWARD_VELOCITY * MAX_SIDE_VELOCITY;
	// cout << "forwardVelocity: " << forwardVelocity << "\n";
	// cout << "proportionalReaction: " << proportionalReaction << "\n";
	CommandVelocity(forwardVelocity, proportionalReaction)
	lastVelocity = averageVelocity;  // For comparison to next computed velocity
}

void ReturnMoveBackState::CommandVelocity(double forwardVelocity, double proportionalReaction)
{
	switch(returnType) {
		case ReturnMoveEnum::FORWARD:
			Robot()->SetWheelVelocity(forwardVelocity, forwardVelocity);
			break;
		case ReturnMoveEnum::RIGHT: 
			Robot()->SetWheelVelocity(forwardVelocity + proportionalReaction,
									  forwardVelocity - proportionalReaction);
			break;
		case ReturnMoveEnum::LEFT: 
			Robot()->SetWheelVelocity(forwardVelocity - proportionalReaction,
									  forwardVelocity + proportionalReaction);
			break;
	}
}

void ReturnMoveBackState::TeensyStatus(TeenseyStatus status)
{
	/*
	if(mStartTime.msecsTo(QDateTime::currentDateTime()) < 4000)
		return;

	auto odometry = Robot()->GetOdometryTraveledSince(mStartTime);
	auto newDecawave = Robot()->GetDecawave()->lastDistance;

	double decawaveDiff = startDecawaveValue - newDecawave;
	double diff = std::abs(decawaveDiff - odometry.mDistanceTraveled) / odometry.mDistanceTraveled;

	if(decawaveDiff > 0.0)
	{
		//TODO: WTF: Wa are getting farther away?
	}

	std::cout << "Returning to base station: \n";
	std::cout << "\tDecawave diff: " << decawaveDiff << "\t" << " Odometry: " << odometry << "\n";
	std::cout << "\tTotal diff: " << diff << "\n";

	//TODO: Remove 0.1 constant, and add a value that is relative to odometry.mDistanceTraveled
	if(std::abs(diff) < 0.1)
	{
		//We are going in the right direction
		Robot()->SetWheelVelocity(0.3, 0.3);
		returnType = ReturnMoveEnum::FORWARD;
		std::cout << "\tMoving forward\n";
	}
	else
	{
		if(mLastPerformance != 0.0 && mLastPerformance < diff)
		{
			//We did not have an improvement since the last run
			switch(returnType) {
				case ReturnMoveEnum::FORWARD:
				case ReturnMoveEnum::RIGHT: 
					std::cout << "\tMoving left\n";
					Robot()->SetWheelVelocity(0.2, 0.3);
					returnType = ReturnMoveEnum::LEFT;
					break;
				case ReturnMoveEnum::LEFT: 
					std::cout << "\tMoving right\n";
					Robot()->SetWheelVelocity(0.3, 0.2);
					returnType = ReturnMoveEnum::RIGHT;
					break;
			}		
		}
	}

	mLastPerformance = diff;

	Reset();

	if(newDecawave < 10.0){
		Robot()->SetWheelVelocity(0.0, 0.0);
		SetFinished();
		return;
	}
	*/
}

void ReturnMoveBackState::Reset()
{
	mStartTime = QDateTime::currentDateTime();
	//startDecawaveValue = Robot()->GetDecawave()->lastDistance;
	//std::cout << "Reseting the moving back (" << startDecawaveValue << ")\n";
}


////////////////////
////	ReturnRealignState
////	We are going to first check if we have an obstacle in front of us
//// 	And move a bit to figure out which direction is the base station
////////////////////

void ReturnRealignState::Start()
{
	connect(Robot()->mPlanner, SIGNAL(ObstacleMapUpdate(ObstacleMap)), this, SLOT(ReceiveObstcles(ObstacleMap)));
	connect(&mPathFutureWatcher, SIGNAL(finished()), this, SLOT(FinishedTrajectory()));

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

	std::cout << "Finished calculating trajectory\n";
}

void ReturnRealignState::Realign()
{
	//auto odometry = Robot()->GetOdometryTraveledSince(mMoveForward->mStartTime);
	//auto odometryDistance = odometry.mDistanceTraveled;

	double endDecawaveValue = Robot()->GetDecawave()->lastDistance;
	cout << "Decawave: " << endDecawaveValue << "\n";
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
	connect(Robot()->mPlanner, SIGNAL(ObstacleMapUpdate(ObstacleMap)), this, SLOT(ReceiveObstcles(ObstacleMap)));
}