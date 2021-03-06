#pragma once

#include <QObject>
#include <QSqlDatabase>
#include <QFutureWatcher>
#include <QDateTime>
#include "../nzmqt/nzmqt.hpp"
#include "../pointcloud.h"
#include "../teensy.h"
#include "../odometry.h"
#include "../april.h"

#include <QImage>

namespace Robot{

class Kratos2;
class BaseState;
class RootState;
class TrajectoryPlanner2;

class SegmentedPointCloud 
{
public:
	QDateTime mTimestamp;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mPointCloud;
};

class Decawave : public QObject
{
	Q_OBJECT
	
public:
	double lastDistance;

	Decawave(QObject* parent) : QObject(parent), lastDistance(10000)
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
    void receiveColorImage(Robot::ImgData mat);
    void receiveDepthImage(Robot::DepthImgData mat);
    
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

	virtual void SetSorter(int slot) = 0;
	virtual void SetCollector(int col) = 0;

signals:
    void statusUpdate(TeenseyStatus);
};

class Teensy2 : public QObject
{
	Q_OBJECT
public:
	Teensy2Status lastStatus;

	Teensy2(QObject* parent) : QObject(parent)
	{
	}

	virtual void setAprilAngle(double angle)
	{
		sendRaw(static_cast<int>(angle * 180.0 / M_PI));
	}

	virtual void sendRaw(int intAngle) = 0;
	

signals:
    void statusUpdate(Teensy2Status);
};


class SampleDetection : public QObject
{
	Q_OBJECT

public:
	QList<DetectedSample> mLastDetection;

	SampleDetection(QObject* parent) : QObject(parent)
	{
	}

signals:
	//Returns x / y relative to camera position, in meters
	void SampleDetected(QList<DetectedSample> samples);
};


class SensorLog : public QObject
{
	Q_OBJECT
public:
	QSqlDatabase mDb;
	nzmqt::ZMQSocket* mSocket;
	QFutureWatcher<QByteArray> mAprilTagWatcher;
	Kratos2* mRobot;

	SensorLog(Kratos2* parent, nzmqt::ZMQContext* context);

	void ReceiveSegmentedPointcloud(SegmentedPointCloud pointCloud);
	void forceUpdated(double leftForce, double rightForce);
	void ReceivePath(const std::vector<Eigen::Vector2d> &points);

	void AprilLocationUpdate(QDateTime time, Eigen::Affine2d location);

	void SendObstacles(std::vector<Eigen::Vector2d>);
	void SetRobot(Eigen::Vector2d pos, double ang);

public slots:
	void receiveDepthImage(Robot::DepthImgData mat);
	void receiveKinectImage(Robot::ImgData mat);
	void teensyStatus(TeenseyStatus status);
	void teensy2Status(Teensy2Status status);
	void decawaveUpdate(double distance);
	void WheelVelocityUpdate(double left, double right);
	void ReceiveAprilTagImage(QImage image);
	void SendAprilTagInfo();
	void ReceiveAprilTags(QList<AprilTagDetectionItem> tags);
	void ReceiveAprilTags2(QList<AprilTagDetectionItem> tags);
	void SampleDetected(QList<DetectedSample> samples);
};

class LocationEstimation : public QObject
{
	Q_OBJECT

	Kratos2* robot;
	Odometry odometry;
	TeenseyStatus lastTeensyStatus;

	QVector<Eigen::Affine2d> mHistory;

public:
	LocationEstimation(Kratos2* parent);
	Eigen::Affine2d GetEstimate();

public slots:
	void teensyStatus(TeenseyStatus status);
	void AprilLocationUpdate(Eigen::Affine2d newLocation);

	void SendUpdate();
};

class Kratos2 : public QObject
{
	Q_OBJECT

	double mLeftWheelVelocity;
	double mRightWheelVelocity;

	double mTotalRunTime;
	QDateTime mLastUnpaused;

public:
	nzmqt::ZMQContext* mContext;
	LocationEstimation mLocation;

	SensorLog* mSensorLog;
	QFutureWatcher<SegmentedPointCloud> mFutureWatcher;

	TrajectoryPlanner2* mPlanner;

	RootState *mState;

	bool mIsPaused;

	int mLastAprilId;
	QDateTime mLastAprilDetection;
	Eigen::Affine2d mLastAprilLocation;

	Kratos2(QObject* parent);

	void Initialize();

	AprilTagCamera* mApril2;

	virtual Kinect* GetKinect() = 0;
	virtual Teensy* GetTeensy() = 0;
	virtual Teensy2* GetTeensy2() = 0;
	virtual Decawave* GetDecawave() = 0;
	virtual AprilTagCamera* GetApril() = 0;
	virtual SampleDetection* GetSampleDetection() = 0;
	virtual QString Name() = 0;

	//Set the velocities in m/s
	//Upper limit at ~0.66
	void SetWheelVelocity(double left, double right);
	double GetLeftVelocity();
	double GetRightVelocity();

	double GetTimeEstimate();

	Odometry GetOdometryTraveledSince(QDateTime startTime, QDateTime endTime = QDateTime::currentDateTime(), bool useApril = false);

public slots:
	void ProccessPointCloud(Robot::DepthImgData mat);
	void receiveKinectImage(Robot::ImgData mat);
	void TeensyStatus(TeenseyStatus status);
	void FinishedPointCloud();

	void AprilTagDetected(QList<AprilTagDetectionItem> detections);
	virtual void AprilTag2Detected(QList<AprilTagDetectionItem> detections);
	void AprilScanTimer();

	void StartStateMachine();
	
signals:
	void pauseUpdate(bool); //True when paused
	void WheelVelocityUpdate(double left, double right);
	void AprilLocationUpdate(Eigen::Affine2d newLocation);
};



}
