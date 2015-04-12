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
	nzmqt::ZMQSocket* mSocket;

	GazeboTeensey* mTeensey;
	GazeboKinect* mKinect;
	GazeboDevawave* mDecaWave;

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

protected slots:

    void messageReceived(const QList<QByteArray>& message);
};

}