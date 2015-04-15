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

	WheelPID(QObject* parent);

	void SetLeftDesiredAngularVelocity(double speed); //In radians per second
	void SetLeftDesiredVelocity(double speed); // In m/s

	void SetRightDesiredAngularVelocity(double speed); //In radians per second
	void SetRightDesiredVelocity(double speed); // In m/s

public slots:
	void teensyStatus(TeenseyStatus status);
};


class Kratos2 : public QObject
{
	Q_OBJECT

public:
	nzmqt::ZMQContext* mContext;

	SensorLog* mSensorLog;
	QFutureWatcher<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> mFutureWatcher;

	TrajectoryPlanner2* mPlanner;

	Kratos2(QObject* parent);

	void Initialize();

	virtual Kinect* GetKinect() = 0;
	virtual Teensy* GetTeensy() = 0;
	virtual Decawave* GetDecawave() = 0;

	virtual void SetLeftWheelPower(double power) = 0;
	virtual void SetLRightWheelPower(double power) = 0;

public slots:
	void ProccessPointCloud(DepthImgData mat);
	void FinishedPointCloud();


};



}