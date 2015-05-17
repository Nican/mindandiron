#pragma once

#include "robot.h"
#include "trajectory2.h"
#include <QDateTime>
#include <QQueue>

namespace Robot
{

class TrajectorySearch;
class MoveForwardState;
class MoveTowardsGoalState;

class BaseState : public QObject
{
	Q_OBJECT
public:

	Kratos2* mRobot;

	BaseState(Kratos2 *parent) : QObject(parent), mRobot(parent)
	{
	}

	virtual bool IsValid();
	virtual void Start(){};
	virtual void End(){};
};

class ProgressState;

class RootState : public BaseState
{
	Q_OBJECT

	ProgressState *mState;

public:
	RootState(Kratos2 *parent);

	void SetState(ProgressState* nextState);

public slots:
	void MoveToNextState();

};


class ProgressState : public BaseState
{
	Q_OBJECT

private:
	bool mIsFinished;

public:
	ProgressState(Kratos2 *parent) : BaseState(parent), mIsFinished(false)
	{
	}

	void SetFinished();
	bool IsFinished();
	virtual bool IsValid() override;

signals:
	void Finished();
};


class ReturnToStationState : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	ProgressState *mState;

	ReturnToStationState(Kratos2 *parent) : ProgressState(parent), mState(nullptr)
	{
	}

	virtual void Start() override;
	void SetState(ProgressState* nextState);

public slots:
	void MoveToNextState();
};


class BackIntoBaseStationState : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;

	BackIntoBaseStationState(Kratos2 *parent) : ProgressState(parent)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
	void FoundAprilTag(Eigen::Affine2d newLocation);

};


class ReturnLocateAprilState : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	QDateTime mFinishedRotatingTime;

	MoveTowardsGoalState* mMoveInfront;

	ReturnLocateAprilState(Kratos2 *parent) : ProgressState(parent)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
	void FoundAprilTag(Eigen::Affine2d newLocation);
	void FinishRotate();
	void RealignInFront();
};

enum class ReturnMoveEnum 
{
	FORWARD,
	LEFT,
	RIGHT
};

class ReturnMoveBackState : public ProgressState
{
	Q_OBJECT
public:

	ReturnMoveEnum returnType;
	QDateTime mStartTime;
	double startDecawaveValue;
	double mLastPerformance;

	ReturnMoveBackState(Kratos2 *parent) : ProgressState(parent)
	{
	}

	virtual void Start() override;
	void Reset();

public slots:
	void TeensyStatus(TeenseyStatus status);
};



class ReturnRealignState : public ProgressState
{
	Q_OBJECT
public:

	QFutureWatcher<std::shared_ptr<TrajectorySearch>> mPathFutureWatcher;
	QDateTime mStartTime;
	MoveForwardState* mMoveForward;
	double startDecawaveValue;

	ReturnRealignState(Kratos2 *parent) : ProgressState(parent)
	{
	}

	virtual void Start() override;

public slots:
	void ReceiveObstcles(ObstacleMap obstacleMap);
	void FinishedTrajectory();
	void Realign();
	void FinishRotate();
};


class ExploreState : public ProgressState
{
	Q_OBJECT
public:

	MoveTowardsGoalState* mGoalMove;
	QDateTime mStartTime;

	ExploreState(Kratos2 *parent) : ProgressState(parent)
	{
	}

	virtual void Start() override;
};



class ObstacleHistory
{
public:
	ObstacleMap mMap;
	Eigen::Vector2d mPosition;
	double mRotation;
};

class MoveTowardsGoalState : public ProgressState
{
	Q_OBJECT
public:

	QQueue<ObstacleHistory> mObstacleHistory;

	QFutureWatcher<std::shared_ptr<TrajectorySearch>> mPathFutureWatcher;
	std::shared_ptr<TrajectorySearch> mLastResult;
	std::vector<Eigen::Vector2d> mLastResultPoints;

	Eigen::Vector2d mGoal;
	QDateTime mStartTime;
	Eigen::Vector2d mStartPos;
	double mStartAngle;
	bool mReverse;

	MoveTowardsGoalState(Kratos2 *parent);

	virtual void Start() override;
	void DriveTowards(Odometry odometry, Eigen::Vector2d goal);

	

public slots:
	void UpdateTrajectory(ObstacleMap);
	void TeensyStatus(TeenseyStatus status);
	void FinishedTrajectory();
};



class MoveForwardState : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	double mDistance;

	MoveForwardState(Kratos2 *parent, double distance) : ProgressState(parent), mDistance(distance)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
};


class RotateState : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	double mRotation;

	RotateState(Kratos2 *parent, double rotation) : ProgressState(parent), mRotation(rotation)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
};

}