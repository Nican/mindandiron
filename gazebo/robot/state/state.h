#pragma once

#include "../robot.h"
#include "../trajectory2.h"
#include <QDateTime>
#include <QQueue>

namespace Robot
{

class TrajectorySearch;
class MoveForwardState;
class MoveTowardsGoalState;
class RotateState;

class BaseState : public QObject
{
	Q_OBJECT
public:

	BaseState(QObject *parent) : QObject(parent)
	{
	}

	virtual bool IsValid();
	virtual void Start(){};
	virtual void End(){};

	virtual Kratos2* Robot();
};

class ProgressState;

class RootState : public BaseState
{
	Q_OBJECT

	ProgressState *mState;

public:
	RootState(QObject *parent);

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
	ProgressState(QObject *parent) : BaseState(parent), mIsFinished(false)
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

	ReturnToStationState(QObject *parent) : ProgressState(parent), mState(nullptr)
	{
	}

	virtual void Start() override;
	void SetState(ProgressState* nextState);

public slots:
	void MoveToNextState();
};


class LeaveBaseStation : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	MoveTowardsGoalState* mMoveInfront;

	LeaveBaseStation(QObject *parent) : ProgressState(parent), mMoveInfront(nullptr)
	{
	}

	virtual void Start() override;

public slots:
	void MoveToRotate();
	void MoveToNextState();
	void FoundAprilTag(Eigen::Affine2d newLocation);
};



class BackIntoBaseStationState : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	MoveTowardsGoalState* mMoveInfront;

	BackIntoBaseStationState(QObject *parent) : ProgressState(parent), mMoveInfront(nullptr)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
	void FoundAprilTag(Eigen::Affine2d newLocation);
	void DriveInto();
};


class ReturnLocateAprilState : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	QDateTime mFinishedRotatingTime;

	MoveTowardsGoalState* mMoveInfront;
	RotateState* mRotate;

	ReturnLocateAprilState(QObject *parent) : ProgressState(parent), mMoveInfront(nullptr)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
	void FoundAprilTag(Eigen::Affine2d newLocation);
	void FinishRotate();
	void RealignInFront();
	void FinishedFinalRotation();
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

	std::array<double, 5> mLastReadings;
	std::size_t lastReadingId;

	ReturnMoveBackState(QObject *parent) : ProgressState(parent), lastReadingId(0)
	{
	}

	virtual void Start() override;
	void Reset();
	void UpdateDirection();

public slots:
	void TeensyStatus(TeenseyStatus status);
	void DecawaveUpdate(double value);
};



class ReturnRealignState : public ProgressState
{
	Q_OBJECT
public:

	QFutureWatcher<std::shared_ptr<TrajectorySearch>> mPathFutureWatcher;
	QDateTime mStartTime;
	MoveForwardState* mMoveForward;
	double startDecawaveValue;

	ReturnRealignState(QObject *parent) : ProgressState(parent)
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

	ExploreState(QObject *parent) : ProgressState(parent)
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
	bool mAprilUpdates;

	QDateTime mLastAprilUpdate;
	Eigen::Affine2d mLastApril;

	MoveTowardsGoalState(QObject *parent);

	virtual void Start() override;
	void DriveTowards(Odometry odometry, Eigen::Vector2d goal);



public slots:
	void UpdateTrajectory(ObstacleMap);
	void TeensyStatus(TeenseyStatus status);
	void FoundAprilTag(Eigen::Affine2d newLocation);
	void FinishedTrajectory();
};



class MoveForwardState : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	double mDistance;

	MoveForwardState(QObject *parent, double distance) : ProgressState(parent), mDistance(distance)
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
	double mTotalRotation;
	double mLastRotation;

	RotateState(QObject *parent, double rotation) : 
		ProgressState(parent), 
		mRotation(rotation), 
		mTotalRotation(0.0),
		mLastRotation(0.0)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
};

}