#pragma once
#include <QObject>
#include "robot.h"
#include "../gazebo2.h"

class QTimer;

namespace Robot{

class GazeboKratos;

class GazeboSampleDetection : public SampleDetection
{
	Q_OBJECT
public:
	RobotGazeboTickData gazeboData;

	GazeboSampleDetection(QObject* parent) : SampleDetection(parent)
	{
	}

public slots:
	void receiveUpdate(const RobotGazeboTickData &data);
	void TeensyStatus(TeenseyStatus status);
};

class GazeboAprilTag : public AprilTagCamera
{
	Q_OBJECT
public:
	GazeboKratos* robot;

	GazeboAprilTag(GazeboKratos* parent = 0);

	virtual void finishedProcessing(std::vector<AprilTags::TagDetection> detections) override;
};

class GazeboDevawave : public Decawave
{
	Q_OBJECT
public:
	QTimer* mUpdateTimer;
	RobotGazeboTickData mLastTick;

	GazeboDevawave(QObject* parent);

	void receiveUpdate(const RobotGazeboTickData &data);

protected slots:
	void fireUpdate();
};

class GazeboKinect : public Kinect
{
	Q_OBJECT
	
public:
	bool colorFrameRequested;
	bool depthFrameRequested;

	GazeboKinect(QObject* parent) : 
		Kinect(parent), 
		colorFrameRequested(false), 
		depthFrameRequested(false)
	{
	}

	virtual void requestDepthFrame() override;
    virtual void requestColorFrame() override;
};

class GazeboTeensey : public Teensy
{
	Q_OBJECT

public:
	QTimer* mUpdateTimer;
	RobotGazeboTickData mLastTick;

	GazeboTeensey(QObject* parent = 0);

	void receiveUpdate(const RobotGazeboTickData &data);

	virtual void SetSorter(int slot) override {}

	virtual void SetCollector(int col) override	{}

protected slots:
	void fireUpdate();
};

class GazeboTeensey2 : public Teensy2
{
	Q_OBJECT

public:
	QTimer* mUpdateTimer;
	RobotGazeboTickData mLastTick;
	int mServoAngle;

	GazeboTeensey2(QObject* parent = 0);

	void receiveUpdate(const RobotGazeboTickData &data);
	virtual void sendRaw(int intAngle) override;

	double GetAprilGazeboAngle();

protected slots:
	void fireUpdate();
};



class GazeboKratos : public Kratos2
{
	Q_OBJECT

public:
	nzmqt::ZMQSocket* mSubSocket;
	nzmqt::ZMQSocket* mPubSocket;

	GazeboTeensey* mTeensey;
	GazeboKinect* mKinect;
	GazeboDevawave* mDecaWave;
	GazeboTeensey2* mTeensy2;
	GazeboAprilTag* mAprilTag;
	GazeboSampleDetection* mSampleDetection;

	QTimer* mSendControlTimer;

	GazeboKratos(QObject* parent = 0);

	virtual Kinect* GetKinect() override
	{
		return mKinect;
	}

	virtual Teensy* GetTeensy() override
	{
		return mTeensey;
	}

	virtual Teensy2* GetTeensy2() override
	{
		return mTeensy2;
	}

	virtual Decawave* GetDecawave() override
	{
		return mDecaWave;
	}

	virtual AprilTagCamera* GetApril() override
	{
		return mAprilTag;
	}

	virtual SampleDetection* GetSampleDetection() override
	{
		return mSampleDetection;
	}

	virtual QString Name() override
	{
		return QString("gazebo");
	}

	virtual void AprilTag2Detected(QList<AprilTagDetectionItem> detections) override;

protected slots:
	void fireControlUpdate();
    void messageReceived(const QList<QByteArray>& message);
};

}