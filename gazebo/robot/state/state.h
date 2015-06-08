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
class ExploreState;

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
	virtual void Start() override;

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
	int finalApproach;

	LeaveBaseStation(QObject *parent) : ProgressState(parent), mMoveInfront(nullptr)
	{
	}

	virtual void Start() override;

public slots:
	void MoveToRotate();
	void MoveToNextState();
};


class Level1State : public ProgressState
{
	Q_OBJECT
public:

	LeaveBaseStation* leaveBase;
	MoveTowardsGoalState* mMoveInfront;
	ExploreState* mExplore;

	Level1State(QObject *parent) : ProgressState(parent), leaveBase(nullptr), mMoveInfront(nullptr)
	{
	}

	virtual void Start() override;

public slots:
	void StartToTravelBehind();
	void MoveForwardBehind();
	void StartExplore();
};


class NavigateToSample : public ProgressState
{
	Q_OBJECT
public:
	int finalApproach;

	NavigateToSample(QObject *parent) : ProgressState(parent)
	{
	}

	virtual void Start() override;

public slots:
	void ProportionalSteerOverSample(QList<DetectedSample> samples);
	void MomentarilyHaltRobot();
	void BackUpToCollectSample();
	void FinishSampleCollection();
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
	void StartRealignInFrontRotation();
	void FinishedRealignRotation();
	// void SendFinishedRealignRotationSignal();
};

enum class ReturnMoveEnum 
{
	FORWARD,
	LEFT,
	RIGHT
};

class DecawaveMoveRadialState : public ProgressState
{
	Q_OBJECT
public:
	ReturnMoveEnum returnType;
	QDateTime mStartTime;
	double startDecawaveValue;
	double mLastPerformance;

	int in;  // Should the robot travel in?
	std::array<double, 4> mLastReadings;
	std::size_t lastReadingId;

	DecawaveMoveRadialState(int goIn, QObject *parent) : ProgressState(parent), lastReadingId(0)
	{
		in = goIn;
	}

	virtual void Start() override;
	double GetAverageDecawaveVelocity();
	virtual void UpdateDirection(double averageVelocity, int in);
	void CommandVelocity(double forwardVelocity, double proportionalReaction);

public slots:
	void DecawaveUpdate(double value);
};


class DecawaveMoveTangentialState : public DecawaveMoveRadialState
{
	Q_OBJECT
public:
	DecawaveMoveTangentialState(QObject *parent) : DecawaveMoveRadialState(0, parent)
	{
	}
	virtual void UpdateDirection(double averageVelocity, int in) override;
public slots:
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

	ExploreState(QObject *parent) : ProgressState(parent), mGoalMove(nullptr)
	{
	}

	virtual void Start() override;

public slots:
	void StartNavigation();
	void FailedNavigation();
	void MoveToNextState();
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

signals:
	void Failed();

public slots:
	void UpdateTrajectory(ObstacleMap);
	void TeensyStatus(TeenseyStatus status);
	void FoundAprilTag(Eigen::Affine2d newLocation);
	void FinishedTrajectory();
};




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class TravelToWayPoint : public ProgressState
{
	Q_OBJECT
public:

	Eigen::Vector2d mTargetPosition;
	bool mReverse;

	TravelToWayPoint(Eigen::Vector2d position, QObject *parent);
	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
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