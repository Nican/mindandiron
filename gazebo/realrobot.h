#pragma once

#include <QObject>
#include <QTimer>
#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>

#include <opencv2/opencv.hpp>


class RealRobot : public QObject, public libfreenect2::FrameListener
{
    Q_OBJECT

    libfreenect2::Freenect2 freenect2;
  	libfreenect2::Freenect2Device *dev;

public:
    RealRobot();

    virtual bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame) override;

signals:
	void receiveColorImage(cv::Mat mat);
    void receiveDepthImage(cv::Mat mat);

public slots:
    void showGPS();

    void receiveColorImage2(cv::Mat mat);

private:
    QTimer *timer;
};