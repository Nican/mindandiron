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

class KratosAprilTag : public QObject
{
	Q_OBJECT
public:

	KratosCamera* mCamera;
	std::shared_ptr<AprilTags::TagDetector> m_tagDetector;

	QFutureWatcher<std::vector<AprilTags::TagDetection>> mDetectionFutureWatcher;

	double mTagSize;
	double mFx;
	double mFy;
	double mPx;
	double mPy;

	KratosAprilTag(QObject* parent = 0);

	void readCamera();

public slots:
	void finishedProcessing();

signals:
	void tagsDetected(QList<AprilTagDetectionItem>);
};



class KratosKinect : public Robot::Kinect, public libfreenect2::FrameListener
{
	Q_OBJECT

	libfreenect2::Freenect2Device *mDev;

	std::mutex mRequestLock;
	bool depthRequested; 
	bool colorRequested;

public:
	
	KratosKinect(libfreenect2::Freenect2Device *dev, QObject* parent);

	virtual bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame) override;

	virtual void requestDepthFrame() override;
	virtual void requestColorFrame() override;
	
private:
	Q_DISABLE_COPY(KratosKinect)

};


class RealRobot : public Robot::Kratos2
{
	Q_OBJECT

	libfreenect2::Freenect2 freenect2;
	KratosKinect* mKinect;

	Robot::KratosTeensy* mTeensy;
	Robot::KratosTeensy2* mTeensy2;
	Robot::TeenseyStatus lastStatus;
	Robot::KratosDecawave* mDecawave;
	Robot::KratosAprilTag* mAprilTag;
	bool bFirstTeenseyMessage;

	QDateTime mLastAprilDetection;

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

	virtual QString Name() override
	{
		return "robot";
	}


public slots:
	void updateForces();
};

}