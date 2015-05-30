#pragma once

#include "../odometry.h"
#include "../robot.h"
#include "robot.h"
#include "teensey.h"
#include "camera.h"
#include <iostream>
#include <mutex>
#include <QElapsedTimer>
#include <QObject>
#include <QDateTime>
#include <QTimer>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>

#include <opencv2/opencv.hpp>
#include "../AprilTags/TagDetector.h"

namespace Robot 
{

class KratosSampleDetection : public SampleDetection
{
	Q_OBJECT
public:
	nzmqt::ZMQSocket* mSubSocket;


	KratosSampleDetection(nzmqt::ZMQContext* context, QObject* parent);

public slots:
	void messageReceived(const QList<QByteArray>& messages);
	//void receiveUpdate(const RobotGazeboTickData &data);
};

class KratosAprilTag : public AprilTagCamera
{
	Q_OBJECT
public:
	QFutureWatcher<QImage> mImageWatcher;

	KratosCamera* mCamera;
	KratosAprilTag(QObject* parent = 0);

	void RequestFrame();

public slots:
	void ReceiveCameraFrame();
};



class KratosKinect : public Robot::Kinect, public libfreenect2::FrameListener
{
	Q_OBJECT

	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *mDev;

	std::mutex mRequestLock;
	bool depthRequested; 
	bool colorRequested;

public:
	
	KratosKinect(QObject* parent);

	virtual bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame) override;

	virtual void requestDepthFrame() override;
	virtual void requestColorFrame() override;
	
private:
	Q_DISABLE_COPY(KratosKinect)

};


class RealRobot : public Robot::Kratos2
{
	Q_OBJECT

	KratosKinect* mKinect;

	Robot::KratosTeensy* mTeensy;
	Robot::KratosTeensy2* mTeensy2;
	Robot::TeenseyStatus lastStatus;
	Robot::KratosDecawave* mDecawave;
	Robot::KratosAprilTag* mAprilTag;
	Robot::KratosSampleDetection* mSampleDetection;
	bool bFirstTeenseyMessage;

	KratosCamera* mFrontCamera;

public:
	RealRobot(QObject* parent = 0);

	virtual Kinect* GetKinect() override
	{
		return mKinect;
	}

	virtual Teensy* GetTeensy() override
	{
		return mTeensy;
	}

	virtual Decawave* GetDecawave() override
	{
		return mDecawave;
	}

	virtual Teensy2* GetTeensy2() override
	{
		return mTeensy2;
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
		return QString("robot");
	}

	void RequestFrontImage();

public slots:
	void updateForces();
};

}