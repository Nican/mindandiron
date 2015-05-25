#pragma once

#include <QObject>
#include <QThread>
#include <QImage>
#include "opencv2/opencv.hpp"

class KratosCamera : public QObject
{
	Q_OBJECT

public:

	//usb-046d_HD_Pro_Webcam_C920_2245793F-video-index0
	//usb-046d_HD_Webcam_C615_4E998DA0-video-index0
	//usb-Chicony_USB2.0_HD_UVC_WebCam-video-index0

	int mDeviceId;
	int mWidth;
	int mHeight;

	QString mDeviceName; 
	cv::VideoCapture mCapture;

	KratosCamera(QString name, int width, int height, QObject* parent = 0);

	void openCamera();
	bool read(cv::Mat &image);

	//bool isValid();
	int getCameraByName(QString name);

signals:
	void CameraFrame(QImage image);

};

class KratosThreadCamera : public QThread
{
	Q_OBJECT

public:
	int mDeviceId;
	int mWidth;
	int mHeight;

	QString mDeviceName; 
	cv::VideoCapture mCapture;

	KratosThreadCamera(QString name, int width, int height, QObject* parent = 0);

	void openCamera();
	void run() override;

	int getCameraByName(QString name);

signals:
	void CameraFrame(QImage image);
};
