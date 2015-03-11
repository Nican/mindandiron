#pragma once

#include <QObject>
#include <QTimer>
#include <iostream>
#include "../robot.h"
#include <mutex>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>

#include <opencv2/opencv.hpp>


class KratosKinect : public QObject, public libfreenect2::FrameListener
{
    Q_OBJECT

    libfreenect2::Freenect2Device *mDev;

    std::mutex mRequestLock;
    bool depthRequested; 
    bool colorRequested;

public:
    
    KratosKinect(libfreenect2::Freenect2Device *dev, QObject* parent);

    virtual bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame) override;

    void requestDepthFrame();
    void requestColorFrame();

signals:
    void receiveColorImage(Robot::ImgData mat);
    void receiveDepthImage(Robot::DepthImgData mat);

};


class RealRobot : public QObject
{
    Q_OBJECT

    libfreenect2::Freenect2 freenect2;
    KratosKinect* mKinect;
  	//libfreenect2::Freenect2Device *dev;

    std::shared_ptr<Robot::Kratos> m_kratos;

public:
    RealRobot();

public slots:
    void showGPS();

    void receiveColorImage2(Robot::ImgData mat);
    void receiveDepthImage2(Robot::DepthImgData mat);

private:
    QTimer *timer;
};