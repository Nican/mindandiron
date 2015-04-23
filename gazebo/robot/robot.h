#pragma once

#include <QObject>
#include <QSqlDatabase>
#include <QFutureWatcher>
#include <QDateTime>
#include "../nzmqt/nzmqt.hpp"
#include "../pointcloud.h"
#include "../teensy.h"
#include "trajectory2.h"

namespace Robot{

class BaseState;
class RootState;

class Decawave : public QObject
{
	Q_OBJECT
	
public:
	Decawave(QObject* parent) : QObject(parent)
	{
	}

signals:
    void statusUpdate(double distance);
};

class Kinect : public QObject
{
	Q_OBJECT
	
	public:
	Kinect(QObject* parent) : QObject(parent)
	{
	}

	virtual void requestColorFrame() = 0;
	virtual void requestDepthFrame() = 0;

signals:
    void receiveColorImage(ImgData mat);
    void receiveDepthImage(DepthImgData mat);
    
private:
    Q_DISABLE_COPY(Kinect)
};


class Teensy : public QObject
{
    Q_OBJECT

public:
    Teensy(QObject* parent) : QObject(parent)
	{
	}

signals:
    void statusUpdate(TeenseyStatus);
};


class SensorLog : public QObject
{
	Q_OBJECT
public:
	QSqlDatabase mDb;
	nzmqt::ZMQSocket* mSocket;

	SensorLog(QObject* parent, nzmqt::ZMQContext* context);

	void receiveSegmentedPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);
	void forceUpdated(double leftForce, double rightForce);

public slots:
	void receiveDepthImage(DepthImgData mat);
	void teensyStatus(TeenseyStatus status);
	void decawaveUpdate(double distance);
	void SendObstacles(std::vector<Eigen::Vector2i>);
};

class WheelPID : public QObject
{
	Q_OBJECT
public:
	double mLeftDesiredVelocity; //in rads/sec
	double mRightDesiredVelocity; //in rads/sec

	TeenseyStatus mLastStatus;
	QDateTime mLastStatusTime;

	double mLeftVelocity;
	double mRightVelocity;

	double mLeftForce;
	double mRightForce;

	WheelPID(QObject* parent = 0);

	void SetLeftDesiredAngularVelocity(double speed); //In radians per second
	void SetLeftDesiredVelocity(double speed); // In m/s

	void SetRightDesiredAngularVelocity(double speed); //In radians per second
	void SetRightDesiredVelocity(double speed); // In m/s

	void Reset();

public slots:
	void teensyStatus(TeenseyStatus status);

signals:
    void forceUpdated();
};


class Kratos2 : public QObject
{
	Q_OBJECT

public:
	nzmqt::ZMQContext* mContext;

	SensorLog* mSensorLog;
	QFutureWatcher<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> mFutureWatcher;

	TrajectoryPlanner2* mPlanner;
	WheelPID* mWheelPID;

	RootState *mState;

	bool mIsPaused;

	Kratos2(QObject* parent);

	void Initialize();

	//The state is only set on the next frame
	//void SetState(BaseState* nextState); 

	virtual Kinect* GetKinect() = 0;
	virtual Teensy* GetTeensy() = 0;
	virtual Decawave* GetDecawave() = 0;

	virtual void SetLeftWheelPower(double power) = 0;
	virtual void SetRightWheelPower(double power) = 0;

	Odometry GetOdometryTraveledSince(QDateTime time);

public slots:
	void ProccessPointCloud(DepthImgData mat);
	void TeensyStatus(TeenseyStatus status);
	void FinishedPointCloud();
	void updateForces();

signals:
	void pauseUpdate(bool); //True when paused

};



}