#pragma once
#include <QObject>
#include "robot.h"
#include "../gazebo2.h"

class QTimer;

namespace Robot{

class GazeboKratos;

class GazeboAprilTag : public AprilTagCamera
{
	Q_OBJECT
public:
	bool mFrameRequested;
	GazeboKratos* robot;

	GazeboAprilTag(GazeboKratos* parent = 0);

	virtual void RequestFrame() override;
	virtual void finishedProcessing() override;
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

	virtual QString Name() override
	{
		return QString("gazebo");
	}

protected slots:
	void fireControlUpdate();
    void messageReceived(const QList<QByteArray>& message);
};

}