#pragma once

#include "../robot.h"
#include "../trajectory2.h"
#include <QDateTime>
#include <QQueue>


double GetEuler(Eigen::Matrix2d matrix);

namespace Robot
{

class TrajectorySearch;
class MoveForwardState;
class MoveTowardsGoalState;
class RotateState;
class ExploreState;
class TravelToWayPoint;

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










class BackIntoBaseStationState : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	//MoveTowardsGoalState* mMoveInfront;
	TravelToWayPoint* mWaypoint;

	BackIntoBaseStationState(QObject *parent) : ProgressState(parent), mWaypoint(nullptr)
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
	void FoundAprilTag(Eigen::Affine2d newLocation);
	void TravelToFront();
	void RealignInFront();
	void StartRealignInFrontRotation();
	void FinishedRealignRotation();
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
	bool bGoingOut;

	ReturnRealignState(QObject *parent) : ProgressState(parent), bGoingOut(false)
	{
	}

	virtual void Start() override;

public slots:
	void ReceiveObstcles(ObstacleMap obstacleMap);
	void FinishedTrajectory();
	void Realign();
	void FinishRotate();
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


class ReturnToBaseStationDecawave : public ProgressState
{
	Q_OBJECT

public:
	DecawaveMoveRadialState* mMove;

	ReturnToBaseStationDecawave(QObject *parent);

	virtual void Start() override;

public slots:
	void FoundAprilTag(Eigen::Affine2d newLocation);
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class RotateScanForSample : public ProgressState
{
	Q_OBJECT
public:
	RotateState* mRotate;

	RotateScanForSample(QObject *parent) : ProgressState(parent), mRotate(nullptr)
	{
	}

	virtual void Start() override;

public slots:
	virtual void FinishRotate1();
	virtual void FinishRotate2();
	virtual void FinishRotate3();
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
	void RotateToTarget();
	void StartExplore();
	void EndExplore();
};

class NavigateToSample : public ProgressState
{
	Q_OBJECT
public:
	int finalApproach;
	QDateTime mLastSampleSeen;
	bool bSuccess;

	NavigateToSample(QObject *parent) : ProgressState(parent)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
	void ProportionalSteerOverSample(QList<DetectedSample> samples);
	void MomentarilyHaltRobot();
	void BackUpToCollectSample();
	void LongHaltForRobot();
	void FinishSampleCollection();
};

class ExploreState : public ProgressState
{
	Q_OBJECT
public:

	ProgressState* mExploreOut;
	NavigateToSample* mSampleNavigation;
	QDateTime mLastAprilTag;
	
	double mLastRotation;
	double mNextExploreRadius;
	bool mGoingOutUsingApril;
	bool mSuccessCollect;

	ExploreState(QObject *parent);

	virtual void Start() override;
	void ResetExplore();

public slots:
	void StartNavigation();
	void FinishCollect();
	void FinishPostCollecting();
	void StartDecawaveExplore();

	void FinishRotate();
	void RotateBack();

	void FinishedFarGoal();

	void CheckStuck();

	void StartSampleCollection(QList<DetectedSample> samples);
	void ObstacleMapUpdate(ObstacleMap obstacleMap);
	void DecawaveUpdate(double value);
	void FoundAprilTag(Eigen::Affine2d newLocation);
	void pauseUpdate(bool);
};

class HeadBackAndExploreAgain : public ProgressState
{
	Q_OBJECT
public:

	DecawaveMoveRadialState* mMove;
	TravelToWayPoint* mWaypoint;

	HeadBackAndExploreAgain(QObject *parent) : ProgressState(parent)
	{
	}

	virtual void Start() override;

public slots:
	void StartHeadingBack();
	void FoundAprilTag(Eigen::Affine2d newLocation);
	void ExploreAgain();
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class TravelToWayPoint : public ProgressState
{
	Q_OBJECT
public:

	Eigen::Vector2d mTargetPosition;
	double mTolerance;
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


class AprilRotateState : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	QDateTime mLastAprilTag;
	double mTargetRotation;
	double mTolerance;

	AprilRotateState(QObject *parent, double targetRotation) : 
		ProgressState(parent), 
		mTargetRotation(targetRotation),
		mTolerance(5.0 * M_PI / 180.0)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
	void FoundAprilTag(Eigen::Affine2d newLocation);
};


}