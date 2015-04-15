#pragma once

#include <QObject>
#include <QSqlDatabase>
#include <QFutureWatcher>
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