#include "state.h"
#include "trajectory2.h"
#include <QtConcurrent>

using namespace Robot;
using namespace Eigen;


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

RootState::RootState(Kratos2 *parent) : BaseState(parent), mState(nullptr)
{
	//MoveToNextState();
	//SetState(new MoveTowardsGoalState(mRobot));

	//QTimer::singleShot(1000, this, SLOT(MoveToNextState));

	//SetState(new MoveForwardState(mRobot, 10.0));
	SetState(new ReturnToStationState(mRobot));
	//SetState(new RotateState(mRobot, M_PI));
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
	/*
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
	*/
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
////	ReturnToStationState
////////////////////

void ReturnToStationState::Start()
{
	mStartTime = QDateTime::currentDateTime();

	//Get the -30 seconds of odometry
	auto odometry = mRobot->GetOdometryTraveledSince(mStartTime.addSecs(-30));

	if(mRobot->GetDecawave()->lastDistance < 10.0 || true)
	{
		SetState(new ReturnLocateAprilState(mRobot));
	} 
	else if(odometry.mPosition.norm() < 1.0)
	{
		//If we moved less than 1 meter, we need to re-align
		SetState(new ReturnRealignState(mRobot));
	}
	else
	{
		//TODO: Rotate to the desired orientation
	}
}


void ReturnToStationState::MoveToNextState()
{
	if(qobject_cast<ReturnRealignState*>(mState) != nullptr)
	{
		//Move towards base station
		SetState(new ReturnMoveBackState(mRobot));
		return;
	}

	if(qobject_cast<ReturnMoveBackState*>(mState) != nullptr)
	{
		//Locate april tag
		SetState(new ReturnLocateAprilState(mRobot));
		return;
	}

	if(qobject_cast<ReturnLocateAprilState*>(mState) != nullptr)
	{
		//Locate april tag
		SetState(new BackIntoBaseStationState(mRobot)); //Hehe, BIBSS
		return;
	}
}

void ReturnToStationState::SetState(ProgressState* nextState)
{
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

}

void BackIntoBaseStationState::TeensyStatus(TeenseyStatus status)
{
	
}

void BackIntoBaseStationState::FoundAprilTag(Eigen::Affine2d newLocation)
{
	
}


////////////////////
////	ReturnLocateAprilState
////	After getting close enough, locate the april tag.
////////////////////

void ReturnLocateAprilState::Start()
{
	mMoveInfront = nullptr;
	connect(mRobot, &Kratos2::AprilLocationUpdate, this, &ReturnLocateAprilState::FoundAprilTag);

	//RotateState* rotate = new RotateState(mRobot, M_PI/2);
	//connect(rotate, &ProgressState::Finished, this, &ReturnLocateAprilState::FinishRotate);
	//rotate->Start();

	mFinishedRotatingTime = QDateTime::currentDateTime();

	mStartTime = QDateTime::currentDateTime();
}

void ReturnLocateAprilState::TeensyStatus(TeenseyStatus status)
{
	if(!mFinishedRotatingTime.isValid())
		return;

	//We are going to wait for a maximun of 30 seconds to find the april tags
	if(mStartTime.msecsTo(QDateTime::currentDateTime()) > 30 * 1000){
		//Move around a bit and try to find the april tag again
		return;
	}



}

void ReturnLocateAprilState::FinishRotate()
{
	mRobot->SetWheelVelocity(0.0, 0.0);
	//Can we see the april tag?

	mFinishedRotatingTime = QDateTime::currentDateTime();
}

void ReturnLocateAprilState::FoundAprilTag(Eigen::Affine2d newLocation)
{
	if(mMoveInfront != nullptr)
		return;

	Rotation2Dd rotation2D(0);
	rotation2D.fromRotationMatrix(newLocation.linear());

	mMoveInfront = new MoveTowardsGoalState(mRobot);
	mMoveInfront->mGoal = Vector2d(3.0, 0.0);
	mMoveInfront->mStartPos = newLocation.translation();
	mMoveInfront->mStartAngle = rotation2D.angle();
	mMoveInfront->mReverse = true;
	mMoveInfront->Start();

	connect(mMoveInfront, &ProgressState::Finished, this, &ReturnLocateAprilState::RealignInFront);
}

void ReturnLocateAprilState::RealignInFront()
{
	auto odometry = mRobot->GetOdometryTraveledSince(mMoveInfront->mStartTime);
	odometry.mPosition += mMoveInfront->mStartPos;
	odometry.mTheta += mMoveInfront->mStartAngle;

	//double desiredTheta = 0.0;

	RotateState* rotate = new RotateState(mRobot, -odometry.mTheta);
	rotate->Start();

}

////////////////////
////	ReturnMoveBackState
////	Move forward while attempting to minimize the distance between odometry and decawave distance traveled
////////////////////

void ReturnMoveBackState::Start()
{
	mLastPerformance = 0.0;
	returnType = ReturnMoveEnum::FORWARD;
	Reset();
	connect(mRobot->GetTeensy(), &Teensy::statusUpdate, this, &ReturnMoveBackState::TeensyStatus);

	mRobot->SetWheelVelocity(0.2, 0.2);
}

void ReturnMoveBackState::TeensyStatus(TeenseyStatus status)
{
	if(mStartTime.msecsTo(QDateTime::currentDateTime()) < 4000)
		return;

	auto odometry = mRobot->GetOdometryTraveledSince(mStartTime);
	auto newDecawave = mRobot->GetDecawave()->lastDistance;

	double decawaveDiff = startDecawaveValue - newDecawave;
	double diff = (decawaveDiff - odometry.mDistanceTraveled) / odometry.mDistanceTraveled;

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
		mRobot->SetWheelVelocity(0.4, 0.4);
		returnType = ReturnMoveEnum::FORWARD;
	}
	else
	{
		if(mLastPerformance != 0.0 && mLastPerformance < diff)
		{
			//We did not have an improvement since the last run
			switch(returnType) {
				case ReturnMoveEnum::FORWARD:
				case ReturnMoveEnum::LEFT: 
					std::cout << "Moving left\n";
					mRobot->SetWheelVelocity(0.4, 0.44);
					returnType = ReturnMoveEnum::LEFT;
					break;
				case ReturnMoveEnum::RIGHT: 
					std::cout << "Moving right\n";
					mRobot->SetWheelVelocity(0.44, 0.4);
					returnType = ReturnMoveEnum::RIGHT;
					break;
			}		
		}
	}

	mLastPerformance = diff;

	Reset();

	if(newDecawave < 10.0){
		SetFinished();
		return;
	}
}

void ReturnMoveBackState::Reset()
{
	mStartTime = QDateTime::currentDateTime();
	startDecawaveValue = mRobot->GetDecawave()->lastDistance;
}


////////////////////
////	ReturnRealignState
////	We are going to first check if we have an obstacle in front of us
//// 	And move a bit to figure out which direction is the base station
////////////////////

void ReturnRealignState::Start()
{
	connect(mRobot->mPlanner, SIGNAL(ObstacleMapUpdate(ObstacleMap)), this, SLOT(ReceiveObstcles(ObstacleMap)));
	connect(&mPathFutureWatcher, SIGNAL(finished()), this, SLOT(FinishedTrajectory()));

	mRobot->SetWheelVelocity(0.0, 0.0);
}

void ReturnRealignState::ReceiveObstcles(ObstacleMap obstacleMap)
{
	disconnect(mRobot->mPlanner, SIGNAL(ObstacleMapUpdate(ObstacleMap)), this, SLOT(ReceiveObstcles(ObstacleMap)));

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
	mMoveForward = new MoveForwardState(mRobot, 1.0);
	connect(mMoveForward, &ProgressState::Finished, this, &ReturnRealignState::Realign);

	mMoveForward->Start();

	startDecawaveValue = mRobot->GetDecawave()->lastDistance;

	std::cout << "Finished calculating trajectory\n";
}

void ReturnRealignState::Realign()
{
	//auto odometry = mRobot->GetOdometryTraveledSince(mMoveForward->mStartTime);
	//auto odometryDistance = odometry.mDistanceTraveled;

	double endDecawaveValue = mRobot->GetDecawave()->lastDistance;
	double diff = startDecawaveValue - endDecawaveValue;

	std::cout << "Realigning with diff: " << diff << "\n";

	if(std::abs(diff) <= 0.33)
	{
		//We only moved 0.33 out of 1 meter in direction of the base station. 
		//We must walking somewhat along the circle
		//Rotate 90 degrees
		RotateState* rotate = new RotateState(mRobot, M_PI/4);
		connect(rotate, &ProgressState::Finished, this, &ReturnRealignState::FinishRotate);
		rotate->Start();

		return;
	}

	if(diff < 0)
	{
		//We are going completly in the wrong direction
		RotateState* rotate = new RotateState(mRobot, M_PI/2);
		connect(rotate, &ProgressState::Finished, this, &ReturnRealignState::FinishRotate);
		rotate->Start();

		return;
	}

	//We are going somewhat in the correct direction
	mRobot->SetWheelVelocity(0, 0);
	SetFinished();
}

void ReturnRealignState::FinishRotate()
{
	connect(mRobot->mPlanner, SIGNAL(ObstacleMapUpdate(ObstacleMap)), this, SLOT(ReceiveObstcles(ObstacleMap)));
}

////////////////////
////	Explore State
////////////////////

void ExploreState::Start()
{
	mGoalMove = new MoveTowardsGoalState(mRobot);
	mGoalMove->mGoal = Vector2d(10.0, 10.0);

	
}


////////////////////
////	MoveTowardsGoalState
////////////////////

MoveTowardsGoalState::MoveTowardsGoalState(Kratos2 *parent) : 
	ProgressState(parent), 
	mLastResult(nullptr), 
	mGoal(10.0, 0.0),
	mStartPos(0, 0),
	mStartAngle(0),
	mReverse(false)
{
}

void MoveTowardsGoalState::Start()
{
	mStartTime = QDateTime::currentDateTime();
	//mRobot->mWheelPID->Reset();

	connect(mRobot->GetTeensy(), &Teensy::statusUpdate, this, &MoveTowardsGoalState::TeensyStatus);
	connect(mRobot->mPlanner, SIGNAL(ObstacleMapUpdate(ObstacleMap)), this, SLOT(UpdateTrajectory(ObstacleMap)));

	connect(&mPathFutureWatcher, SIGNAL(finished()), this, SLOT(FinishedTrajectory()));
}

void MoveTowardsGoalState::TeensyStatus(TeenseyStatus status)
{
	if(IsFinished())
		return;
	
	if(mLastResult == nullptr || mLastResultPoints.size() == 0)
	{
		std::cout << "Do not have results and I do not know where I am going. " << mLastResultPoints.size() << "\n";
		//mRobot->SetWheelVelocity(0.2, 0.15);
		return;
	}

	//auto odometrySinceStart = mRobot->GetOdometryTraveledSince(mStartTime);
	auto odometry = mRobot->GetOdometryTraveledSince(mLastResult->mCreatedTime);
	odometry.mPosition += mStartPos;
	odometry.mTheta += mStartAngle;

	mRobot->mSensorLog->SetRobot(odometry.mPosition, odometry.mTheta);

	int index = 0;
	double closestDistance(0.0);

	for(std::size_t i = 0; i < mLastResultPoints.size(); i++)
	{
		double newDistance = (mLastResultPoints[i] - odometry.mPosition).norm();
		if(closestDistance == 0.0 || closestDistance > newDistance)
		{
			index = (int) i;
			closestDistance = newDistance;
		}
	}

	auto nextTargetIndex = std::max<int>(index-5, 0);

	//std::cout << "Closest index: " << index << "(" << mLastResultPoints[index].transpose() << ")" 
	//		  << "\t Next: " << nextTargetIndex<< "(" << mLastResultPoints[nextTargetIndex].transpose() << ")\n";

	{
		QByteArray buffer;
		QDataStream stream(&buffer, QIODevice::WriteOnly);
		stream << odometry.mPosition.x();
		stream << odometry.mPosition.y();
		stream << mLastResultPoints[nextTargetIndex].x();
		stream << mLastResultPoints[nextTargetIndex].y();

		QList<QByteArray> msg;
		msg += QByteArray("\x13");
		msg += buffer;
		mRobot->mSensorLog->mSocket->sendMessage(msg);
	}

	//if(index <= 0)
	//{
	//	mRobot->SetWheelVelocity(0.0, 0.0);
	//	return;
	//}

	DriveTowards(odometry, mLastResultPoints[nextTargetIndex]);	
}

void MoveTowardsGoalState::DriveTowards(Odometry odometry, Eigen::Vector2d goal)
{
	if(mReverse)
		odometry.mTheta += M_PI;

	Eigen::Vector2d diff(goal - odometry.mPosition);
	double desiredAngle = std::atan2(diff.y(), diff.x());

	//Message to Mali: Complex numbers are way better. 
	double angleDiff = std::arg(std::polar(1.0, desiredAngle) * std::polar(1.0, -odometry.mTheta));
	//double angleDiff = fmod(desiredAngle - odometry.mTheta, M_PI);

	//std::cout << "\tOdometry: " << odometry << "\n";
	//std::cout << "\tDriving towards: " << (desiredAngle*180.0/M_PI) << "\t" <<  diff.transpose() << "("<< (angleDiff*180.0/M_PI) << ")\n";

	if(diff.norm() < 0.1){
		std::cout << "Stopping robot. Diff norm is too small\n";
		mRobot->SetWheelVelocity(0.0, 0.0);
		SetFinished();
		return;
	}

	if(angleDiff < -0.05)
	{
		if(mReverse)
			mRobot->SetWheelVelocity(-0.2, -0.4);
		else
			mRobot->SetWheelVelocity(0.4, 0.2);
	}
	else if(angleDiff > 0.05)
	{
		if(mReverse)
			mRobot->SetWheelVelocity(-0.4, -0.2);
		else
			mRobot->SetWheelVelocity(0.2, 0.4);
	} 
	else 
	{
		if(mReverse)
			mRobot->SetWheelVelocity(-0.3, -0.3);
		else
			mRobot->SetWheelVelocity(0.3, 0.3);
	}
}

void MoveTowardsGoalState::UpdateTrajectory(ObstacleMap obstacleMap)
{
	if(IsFinished())
		return;

	using namespace Eigen;
	auto odometry = mRobot->GetOdometryTraveledSince(mStartTime, obstacleMap.mCreatedTime);
	auto odometry2 = mRobot->GetOdometryTraveledSince(mStartTime);
	odometry2.mPosition += mStartPos;
	odometry2.mTheta += mStartAngle;

	if(mReverse)
		odometry2.mTheta += M_PI;

	ObstacleHistory historyItem;
	historyItem.mMap = obstacleMap;
	historyItem.mPosition = odometry.mPosition;
	historyItem.mRotation = odometry.mTheta;



	mObstacleHistory.enqueue(historyItem);

	auto currentTime = QDateTime::currentDateTime();
	auto validRange = currentTime.addSecs(-25);

	while(mObstacleHistory.size() > 0 && mObstacleHistory.head().mMap.mCreatedTime < validRange)
		mObstacleHistory.dequeue();
	
	if(mPathFutureWatcher.future().isRunning())
	{
		std::cout << "Trajectory is still processing. not starting a new one\n";
		return;
	}

	//Build obstacle map
	std::vector<Vector2d> buildObstacleMap;

	for(const auto& item : mObstacleHistory)
	{
		auto rotationTf = Rotation2Dd(item.mRotation);
		auto tf = Translation2d(item.mPosition);

		for(const auto pt : item.mMap.mObstacleList){
			Eigen::Vector2d pt2(tf * rotationTf * pt);
			//pt2.y() *= -1;
			buildObstacleMap.push_back(pt2);
		}
	}

	
	mRobot->mSensorLog->SendObstacles(buildObstacleMap);

	auto future = QtConcurrent::run([this, buildObstacleMap, odometry2](){
		//std::cout << "Started search from: " << odometry2 << "\n";

		std::vector<Vector2d> reducedObstacles;

		for(const auto pt : buildObstacleMap){
			bool hasNeighbor = false;
			for(const auto pt2 : reducedObstacles){
				if((pt - pt2).norm() < 0.1)
				{
					hasNeighbor = true;
					break;
				}
			}

			if(!hasNeighbor)
				reducedObstacles.emplace_back(pt);
		}

		auto planner = std::make_shared<TrajectorySearch>(reducedObstacles, odometry2.mPosition, odometry2.mTheta, this->mGoal);
		std::size_t i = 0;
		
		for(i = 0; i < 500 && !planner->foundSolution; i++)
		{
			planner->rootNode->explore();
		}

		//std::cout << "Finished " << planner->foundSolution << " ("<< i <<")\n";

        return planner;
	});

	mPathFutureWatcher.setFuture(future);
}

void MoveTowardsGoalState::FinishedTrajectory()
{
	if(mLastResult != nullptr)
		return;

	auto planner = mPathFutureWatcher.future().result();

	std::vector<Eigen::Vector2d> points;
	if(planner->GetResult(points) && points.size() > 0)
	{
		points[0] = this->mGoal;

		mLastResult = planner;
		mLastResultPoints = points;
		std::cout << "Path has " << points.size() << " points\n";
	}
	else
	{
		std::cout << "Found no soultion for the next path. :(\n";
	}

	mRobot->mSensorLog->ReceivePath(points);
}



////////////////////
////	MoveForwardState
////////////////////

void MoveForwardState::Start()
{
	mStartTime = QDateTime::currentDateTime();

	connect(mRobot->GetTeensy(), &Teensy::statusUpdate, this, &MoveForwardState::TeensyStatus);

	std::cout << "Start going forward\n";
}

void MoveForwardState::TeensyStatus(TeenseyStatus status)
{
	if(IsFinished())
		return;

	auto odometry = mRobot->GetOdometryTraveledSince(mStartTime);

	mRobot->SetWheelVelocity(0.65, 0.65);

	//std::cout << "\tDistance traveled " << odometry.mDistanceTraveled << "\n";

	if(std::abs(odometry.mDistanceTraveled) > std::abs(mDistance))
	{
		mRobot->SetWheelVelocity(0.0, 0.0);

		std::cout << "Move state finished with " << odometry << "\n";
		SetFinished();
	}
}


////////////////////
////	RotateState
////////////////////

void RotateState::Start()
{
	mStartTime = QDateTime::currentDateTime();

	connect(mRobot->GetTeensy(), &Teensy::statusUpdate, this, &RotateState::TeensyStatus);

	std::cout << "Start Rotate State\n";
}

void RotateState::TeensyStatus(TeenseyStatus status)
{
	if(IsFinished())
		return;

	auto odometry = mRobot->GetOdometryTraveledSince(mStartTime);

	mRobot->SetWheelVelocity(0.1, -0.1);

	static int counter = 0;

	if(counter++ % 20 == 0)
		std::cout << "Rotating: " << odometry << "\n";

	if(std::abs(odometry.mTheta) > std::abs(mRotation))
	{
		mRobot->SetWheelVelocity(0.0, 0.0);

		SetFinished();
		std::cout << "Rotate state finished with " << odometry << "\n";
	}
}