#include "state.h"
#include "trajectory2.h"
#include <QtConcurrent>

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

RootState::RootState(Kratos2 *parent) : BaseState(parent), mState(nullptr)
{
	//MoveToNextState();
	SetState(new MoveTowardsGoalState(mRobot));

	//QTimer::singleShot(1000, this, SLOT(MoveToNextState));

	//SetState(new MoveForwardState(mRobot, 10.0));
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
////	MoveTowardsGoalState
////////////////////

MoveTowardsGoalState::MoveTowardsGoalState(Kratos2 *parent) : 
	ProgressState(parent), 
	mLastResult(nullptr), 
	mGoal(10.0, 0.0)
{
}

void MoveTowardsGoalState::Start()
{
	mStartTime = QDateTime::currentDateTime();
	//mRobot->mWheelPID->Reset();

	connect(mRobot->GetTeensy(), &Teensy::statusUpdate, this, &MoveTowardsGoalState::TeensyStatus);
	connect(mRobot->mPlanner, SIGNAL(ObstacleMapUpdate(std::vector<Eigen::Vector2d>)), this, SLOT(UpdateTrajectory(std::vector<Eigen::Vector2d>)));

	connect(&mPathFutureWatcher, SIGNAL(finished()), this, SLOT(FinishedTrajectory()));
}

void MoveTowardsGoalState::TeensyStatus(TeenseyStatus status)
{
	if(mLastResult == nullptr || mLastResultPoints.size() == 0)
		return;

	//auto odometrySinceStart = mRobot->GetOdometryTraveledSince(mStartTime);
	auto odometry = mRobot->GetOdometryTraveledSince(mLastResult->mCreatedTime);
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

	auto nextTargetIndex = std::max<int>(index-3, 0);

	std::cout << "Closest index: " << index << "(" << mLastResultPoints[index].transpose() << ")" 
			  << "\t Next: " << nextTargetIndex<< "(" << mLastResultPoints[nextTargetIndex].transpose() << ")\n";

	DriveTowards(odometry, mLastResultPoints[nextTargetIndex]);	
}

void MoveTowardsGoalState::DriveTowards(Odometry odometry, Eigen::Vector2d goal)
{
	Eigen::Vector2d diff(goal - odometry.mPosition);
	double desiredAngle = std::atan2(diff.y(), diff.x());
	double angleDiff = fmod(desiredAngle - odometry.mTheta, M_PI / 2.0);

	//std::cout << "Driving towards: " << diff.transpose() << "("<< (angleDiff*180.0/M_PI) << ")\n";

	if(diff.norm() < 0.1)
		return;

	if(angleDiff < -0.02)
	{
		mRobot->SetWheelVelocity(0.2, 0.1);
	}
	else if(angleDiff > 0.02)
	{
		mRobot->SetWheelVelocity(0.1, 0.2);
	} 
	else 
	{
		mRobot->SetWheelVelocity(0.15, 0.15);
	}
}

void MoveTowardsGoalState::UpdateTrajectory(std::vector<Eigen::Vector2d> obstacleList)
{
	using namespace Eigen;
	if(mPathFutureWatcher.future().isRunning())
	{
		std::cout << "Trajectory is still processing. not starting a new one\n";
		return;
	}

	auto odometry = mRobot->GetOdometryTraveledSince(mStartTime);

	//Affine2d robotTransform;
	//robotTransform.prerotate(Rotation2Dd(M_PI/2));
	//robotTransform.pretranslate({5,2});

	auto rot = Rotation2Dd(odometry.mTheta);

	Vector2d goal(rot*mGoal - rot*odometry.mPosition);
	//goal = Rotation2Dd(-odometry.mTheta) * goal;

	std::cout << "Recalcualting trajectroy: \n";
	std::cout << "\tOdometry: " << odometry << " \n";
	std::cout << "\tNew goal: " << goal.transpose() << " \n";

	auto future = QtConcurrent::run([this, obstacleList, goal]() -> std::shared_ptr<TrajectorySearch>{
		auto planner = std::make_shared<TrajectorySearch>(obstacleList, goal);
		std::size_t i = 0;
		
		for(i = 0; i < 500 && !planner->foundSolution; i ++)
		{
			planner->rootNode->explore();
		}

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

	mRobot->SetWheelVelocity(0.2, 0.23);

	//std::cout << "\tDistance traveled " << odometry.mDistanceTraveled << "\n";

	if(std::abs(odometry.mDistanceTraveled) > std::abs(mDistance))
	{
		mRobot->SetWheelVelocity(0.0, 0.0);

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
	mStartTime = QDateTime::currentDateTime();

	connect(mRobot->GetTeensy(), &Teensy::statusUpdate, this, &RotateState::TeensyStatus);

	std::cout << "Start Rotate State\n";
}

void RotateState::TeensyStatus(TeenseyStatus status)
{
	if(IsFinished())
		return;

	auto odometry = mRobot->GetOdometryTraveledSince(mStartTime);

	mRobot->SetWheelVelocity(0.2, -0.2);

	if(std::abs(odometry.mTheta) > std::abs(mRotation))
	{
		SetFinished();
		std::cout << "Rotate state finished with " << odometry << "\n";
		// MoveForwardState* newState = new MoveForwardState(mRobot, 1.0);
		// mRobot->SetState(newState);
	}
}