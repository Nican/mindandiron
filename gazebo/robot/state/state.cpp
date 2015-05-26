#include "state/state.h"
#include "trajectory2.h"
#include <QtConcurrent>

using namespace Robot;
using namespace Eigen;


////////////////////
////	BaseState
////////////////////

bool BaseState::IsValid()
{
	return Robot() != nullptr;
}

Kratos2* BaseState::Robot()
{
	QObject* parent = this->parent();

	Kratos2* robot = qobject_cast<Kratos2*>(parent);
	if(robot != nullptr)
		return robot;

	BaseState* base = qobject_cast<BaseState*>(parent);
	if(base != nullptr)
		return base->Robot();

	std::cerr << "Attempting to get parent state, while it does not exist.";
	return nullptr;
}

////////////////////
////	RootState
////////////////////

RootState::RootState(QObject *parent) : BaseState(parent), mState(nullptr)
{
	//MoveToNextState();
	//SetState(new MoveTowardsGoalState(this));

	//QTimer::singleShot(1000, this, SLOT(MoveToNextState));

	//SetState(new MoveForwardState(this, 10.0));
	SetState(new ReturnToStationState(this));
	//SetState(new RotateState(this, M_PI));
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
		ProgressState* newState = new MoveForwardState(this, 2.0);
		SetState(newState);
	} 
	else
	{
		ProgressState* newState = new RotateState(this, M_PI / 2);
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
	return !IsFinished() && BaseState::IsValid();
}

void ProgressState::SetFinished()
{
	mIsFinished = true;
	emit Finished();
}



////////////////////
////	Explore State
////////////////////

void ExploreState::Start()
{
	mGoalMove = new MoveTowardsGoalState(this);
	mGoalMove->mGoal = Vector2d(10.0, 10.0);

	
}


////////////////////
////	MoveTowardsGoalState
////////////////////

MoveTowardsGoalState::MoveTowardsGoalState(QObject *parent) : 
	ProgressState(parent), 
	mLastResult(nullptr), 
	mGoal(10.0, 0.0),
	mStartPos(0, 0),
	mStartAngle(0),
	mReverse(false),
	mAprilUpdates(false)
{
}

void MoveTowardsGoalState::Start()
{
	mStartTime = QDateTime::currentDateTime();

	connect(Robot()->GetTeensy(), &Teensy::statusUpdate, this, &MoveTowardsGoalState::TeensyStatus);
	connect(Robot()->mPlanner, SIGNAL(ObstacleMapUpdate(ObstacleMap)), this, SLOT(UpdateTrajectory(ObstacleMap)));
	connect(Robot(), &Kratos2::AprilLocationUpdate, this, &MoveTowardsGoalState::FoundAprilTag);

	connect(&mPathFutureWatcher, SIGNAL(finished()), this, SLOT(FinishedTrajectory()));
}

void MoveTowardsGoalState::FoundAprilTag(Eigen::Affine2d newLocation)
{
	mLastAprilUpdate = QDateTime::currentDateTime();
	mLastApril = newLocation;
}

void MoveTowardsGoalState::TeensyStatus(TeenseyStatus status)
{
	if(IsFinished())
		return;

	if(mLastResult == nullptr || mLastResultPoints.size() == 0)
	{
		std::cout << "Do not have results and I do not know where I am going. " << mLastResultPoints.size() << "\n";
		//Robot()->SetWheelVelocity(0.2, 0.15);
		return;
	}

	//auto odometrySinceStart = Robot()->GetOdometryTraveledSince(mStartTime);
	Odometry odometry; 

	if(mAprilUpdates && mLastAprilUpdate.isValid())
	{
		Rotation2Dd rotation2D(0);
		rotation2D.fromRotationMatrix(mLastApril.linear());

		odometry = Robot()->GetOdometryTraveledSince(mLastAprilUpdate);
		odometry.mPosition = Rotation2Dd(mStartAngle) * odometry.mPosition;
		odometry.mPosition += mLastApril.translation();
		odometry.mTheta += rotation2D.angle();
	}
	else
	{	
		odometry = Robot()->GetOdometryTraveledSince(mLastResult->mCreatedTime);
		odometry.mPosition = Rotation2Dd(mStartAngle) * odometry.mPosition;
		odometry.mPosition += mStartPos;
		odometry.mTheta += mStartAngle;
	}

	Robot()->mSensorLog->SetRobot(odometry.mPosition, odometry.mTheta);

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
		Robot()->mSensorLog->mSocket->sendMessage(msg);
	}

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
		Robot()->SetWheelVelocity(0.0, 0.0);
		SetFinished();
		return;
	}

	if(angleDiff < -0.05)
	{
		if(mReverse)
			Robot()->SetWheelVelocity(-0.2, -0.4);
		else
			Robot()->SetWheelVelocity(0.4, 0.2);
	}
	else if(angleDiff > 0.05)
	{
		if(mReverse)
			Robot()->SetWheelVelocity(-0.4, -0.2);
		else
			Robot()->SetWheelVelocity(0.2, 0.4);
	} 
	else 
	{
		if(mReverse)
			Robot()->SetWheelVelocity(-0.3, -0.3);
		else
			Robot()->SetWheelVelocity(0.3, 0.3);
	}
}

void MoveTowardsGoalState::UpdateTrajectory(ObstacleMap obstacleMap)
{
	if(IsFinished())
		return;

	using namespace Eigen;
	auto odometry = Robot()->GetOdometryTraveledSince(mStartTime, obstacleMap.mCreatedTime);
	auto odometry2 = Robot()->GetOdometryTraveledSince(mStartTime);
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

	
	Robot()->mSensorLog->SendObstacles(buildObstacleMap);

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

			//if(!hasNeighbor)
			//	reducedObstacles.emplace_back(pt);
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

	Robot()->mSensorLog->ReceivePath(points);
}



////////////////////
////	MoveForwardState
////////////////////

void MoveForwardState::Start()
{
	mStartTime = QDateTime::currentDateTime();

	connect(Robot()->GetTeensy(), &Teensy::statusUpdate, this, &MoveForwardState::TeensyStatus);

	std::cout << "Start going forward\n";
}

void MoveForwardState::TeensyStatus(TeenseyStatus status)
{
	if(IsFinished())
		return;

	auto odometry = Robot()->GetOdometryTraveledSince(mStartTime);

	Robot()->SetWheelVelocity(0.65, 0.65);

	//std::cout << "\tDistance traveled " << odometry.mDistanceTraveled << "\n";

	if(std::abs(odometry.mDistanceTraveled) > std::abs(mDistance))
	{
		Robot()->SetWheelVelocity(0.0, 0.0);

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

	connect(Robot()->GetTeensy(), &Teensy::statusUpdate, this, &RotateState::TeensyStatus);

	std::cout << "Start Rotate State\n";
}

void RotateState::TeensyStatus(TeenseyStatus status)
{
	if(IsFinished())
		return;

	auto odometry = Robot()->GetOdometryTraveledSince(mStartTime);

	if(mLastRotation != 0.0)
	{
		mTotalRotation += std::abs(odometry.mTheta - mLastRotation);
	}

	mLastRotation = odometry.mTheta;

	if(mRotation < 0.0)
		Robot()->SetWheelVelocity(0.1, -0.1);
	else
		Robot()->SetWheelVelocity(-0.1, 0.1);

	if(mTotalRotation >= std::abs(mRotation))
	{
		Robot()->SetWheelVelocity(0.0, 0.0);

		SetFinished();
		std::cout << "Rotate state finished with " << odometry << " ("<< (mRotation*180.0/M_PI) << ")\n";
	}
}