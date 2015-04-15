#pragma once
#include <QObject>
#include "robot.h"
#include "../gazebo2.h"

class QTimer;

namespace Robot{

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



class GazeboKratos : public Kratos2
{
	Q_OBJECT

public:
	nzmqt::ZMQSocket* mSubSocket;
	nzmqt::ZMQSocket* mPubSocket;

	GazeboTeensey* mTeensey;
	GazeboKinect* mKinect;
	GazeboDevawave* mDecaWave;

	QTimer* mSendControlTimer;

	double leftWheel;
	double rightWheel;

	GazeboKratos(QObject* parent = 0);

	virtual Kinect* GetKinect() override
	{
		return mKinect;
	}

	virtual Teensy* GetTeensy() override
	{
		return mTeensey;
	}

	virtual Decawave* GetDecawave() override
	{
		return mDecaWave;
	}

	virtual void SetLeftWheelPower(double power) override
	{
		leftWheel = power;
	}

	virtual void SetLRightWheelPower(double power) override
	{
		rightWheel = power;
	}

protected slots:
	void fireControlUpdate();
    void messageReceived(const QList<QByteArray>& message);
};

}