#pragma once

#include <QObject>
#include <QTimer>
#include <QElapsedTimer>
#include <iostream>
#include "../robot.h"
#include "robot.h"
#include "../odometry.h"
#include "teensey.h"
#include <mutex>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>

#include <opencv2/opencv.hpp>

namespace Robot 
{

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
    Robot::TeenseyStatus lastStatus;
    bool bFirstTeenseyMessage;
  	//libfreenect2::Freenect2Device *dev;

    //std::shared_ptr<Robot::Kratos> m_kratos;

public:
    RealRobot(QObject* parent = 0);

    QElapsedTimer mElapsedTimer;

    virtual Kinect* GetKinect() override
    {
        return mKinect;
    }

    virtual Teensy* GetTeensy() override
    {
        return mTeensy;
    }

public slots:
    //void showGPS();

    //void receiveColorImage2(Robot::ImgData mat);
   // void receiveDepthImage2(Robot::DepthImgData mat);

private:
    //QTimer *timer;
};

}