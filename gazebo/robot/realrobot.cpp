#include "realrobot.h"

#include <QMetaType>
#include <QThread>
#include <QtConcurrent>
#include <QJsonArray>
#include <QJsonDocument>

#include "../AprilTags/Tag25h9.h"

using namespace Robot;
using namespace Eigen;

//////////////////////////
////	KratosSampleDetection
//////////////////////////

KratosSampleDetection::KratosSampleDetection(nzmqt::ZMQContext* context, QObject* parent) : SampleDetection(parent)
{
	mSubSocket = context->createSocket(nzmqt::ZMQSocket::TYP_SUB, this);
	mSubSocket->setObjectName("Subscriber.Socket.socket(SUB)");
	mSubSocket->connectTo("tcp://127.0.0.1:5561");
	mSubSocket->setOption(nzmqt::ZMQSocket::OPT_SUBSCRIBE, "", 0);

	connect(mSubSocket, SIGNAL(messageReceived(const QList<QByteArray>&)), SLOT(messageReceived(const QList<QByteArray>&)));

}

void KratosSampleDetection::messageReceived(const QList<QByteArray>& messages)
{
	if(messages.size() == 0)
		return;

	QJsonParseError error;
	QList<DetectedSample> samples;
	QJsonDocument json(QJsonDocument::fromJson(messages[0], &error));

	if(error.error != 0)
	{
		std::cout << "Failed to parse JSON from sample detection: " << error.errorString().toStdString() << "\n";
		return;
	}

	QJsonArray locations = json.array();	

	for (int id = 0; id < locations.size(); ++id) {
		QJsonObject locationObject = locations[id].toObject();

	 	DetectedSample sample;
	 	sample.location = {
			(locationObject["x"].toDouble() / 100.0) + 1.0,  // Convert cm to m
			locationObject["y"].toDouble() / 100.0   // Convert cm to m
		};
		sample.name = locationObject["name"].toString();
		samples.append(sample);
	}

	mLastDetection = samples;
	emit SampleDetected(samples);
}

//////////////////////////
////	KratosAprilTag
//////////////////////////

KratosAprilTag::KratosAprilTag(QObject* parent) : 
	AprilTagCamera(1315, 1315, 1920, 1080, parent)
{
	mCamera = new KratosCamera("usb-046d_HD_Pro_Webcam_C920_2245793F-video-index0", mPx, mPy, this);
	// mCamera = new KratosCamera("usb-046d_HD_Pro_Webcam_C920_2245793F-video-index0", 1280, 720, this);
	//mCamera = new KratosCamera("usb-046d_HD_Pro_Webcam_C920_2245793F-video-index0", 960, 540, this);
	//usb-046d_HD_Pro_Webcam_C920_F19B696F-video-index0
	//usb-046d_HD_Pro_Webcam_C920_2245793F-video-index0 -- April camera

	connect(&mImageWatcher, SIGNAL(finished()), this, SLOT(ReceiveCameraFrame()));
	RequestFrame();
}

void KratosAprilTag::ReceiveCameraFrame()
{
	QImage frame = mImageWatcher.future().result();

	emit this->ReceiveFrame(frame);

	RequestFrame();
}

void KratosAprilTag::RequestFrame()
{
	QFuture<QImage> future = QtConcurrent::run([this](){
		QImage frame = this->mCamera->read();

		//static int counter = 0;
		//frame.save("a" + QString::number(counter++) + ".png");

		return frame;
	});

	mImageWatcher.setFuture(future);
}



//////////////////////////
////	KratosKinect
//////////////////////////

KratosKinect::KratosKinect(QObject* parent) : 
	Robot::Kinect(parent)
{	
	mDev = freenect2.openDefaultDevice();

	if(mDev == nullptr)
	{
		std::cout << "no device connected or failure opening the default one!" << std::endl;
	}
	else
	{
		mDev->start();
	}
}

void KratosKinect::requestDepthFrame()
{
	std::lock_guard<std::mutex> lock(mRequestLock);

	depthRequested = true;

	if(mDev != nullptr)
		mDev->setIrAndDepthFrameListener(this);

	//std::cout << "Depth frame requested\n";
}

void KratosKinect::requestColorFrame()
{
	std::lock_guard<std::mutex> lock(mRequestLock);

	colorRequested = true;

	if(mDev != nullptr)
		mDev->setColorFrameListener(this);
}


bool KratosKinect::onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame)
{
	std::lock_guard<std::mutex> lock(mRequestLock);

	//std::cout << "Received new frame\n";

	if(type == libfreenect2::Frame::Color)
	{
		if(colorRequested == false)
			return false;

		colorRequested = false;

		//cv::Mat mat(frame->height, frame->width, CV_8UC3, frame->data);
		Robot::ImgData imgData;
		imgData.width = frame->width;
		imgData.height = frame->height;
		imgData.data.resize(frame->width * frame->height * 3);
		memcpy(imgData.data.data(), frame->data, imgData.data.size());

		emit receiveColorImage(imgData);

	}

	if(type == libfreenect2::Frame::Depth)
	{
		if(depthRequested == false)
			return false;

		depthRequested = false;

		//cv::Mat mat(frame->height, frame->width, CV_32FC1, frame->data);
		//emit receiveDepthImage(mat.clone());
		auto pointCount = frame->width * frame->height;
		float* floatData = (float*)((void*) frame->data);

		Robot::DepthImgData imgData;
		imgData.width = frame->width;
		imgData.height = frame->height;
		imgData.hfov = 70.6 / 180.0 * M_PI;
		imgData.data.resize(pointCount);

		for(std::size_t i = 0; i < pointCount; i++)
			imgData.data[i] = floatData[i] / 1000.0; //Convert cm to m.

		mDev->setIrAndDepthFrameListener(nullptr);

		//std::cout << "Got depth frame\n";

		emit receiveDepthImage(imgData);
	}

	//Return false so that the library re-uses the "frame" pointer
	return false;

};


//////////////////////////
////	RealRobot
//////////////////////////

RealRobot::RealRobot(QObject* parent) : 
	Robot::Kratos2(parent),
	mKinect(nullptr),
	bFirstTeenseyMessage(true)
{
	mLastAprilDetection = QDateTime::currentDateTime();

	mDecawave = new Robot::KratosDecawave(mContext, this);
	mTeensy = new Robot::KratosTeensy(this);
	mTeensy2 = new Robot::KratosTeensy2(this);
	mAprilTag = new Robot::KratosAprilTag(this);
	mSampleDetection = new Robot::KratosSampleDetection(mContext, this);

	QObject::connect(mTeensy, &Robot::KratosTeensy::statusUpdate, [this](Robot::TeenseyStatus status){
		bFirstTeenseyMessage = false;
		lastStatus = status;
	});

	mKinect = new KratosKinect(this);
	mKinect->requestDepthFrame();

	auto timer = new QTimer(this);
	timer->start(100); //time specified in ms
	connect(timer, &QTimer::timeout, this, &RealRobot::updateForces);


	//mFrontCamera = new KratosCamera("usb-046d_HD_Pro_Webcam_C920_F19B696F-video-index0", 1920, 1080, this);
	/*
	connect(mFrontCamera, &KratosCamera::CameraFrame, this, [this](QImage image)
	{
		QImage image2 = image;
		image2.bits();

		QtConcurrent::run([this, image2](){
			static int counter = 0;
			std::cout << "Saving front image\n";
			image2.save("front/" + QString::fromStdString(std::to_string(counter)) + ".png");
			counter++;
			this->RequestFrontImage();
		});
	});
	*/

	//RequestFrontImage();
}

void RealRobot::RequestFrontImage()
{
	/*
	QtConcurrent::run([this](){
		std::cout << "Reading image\n";
		static int counter = 0;
		//std::cout << "Requesting front image\n";
		cv::Mat image;
		this->mFrontCamera->read(image);

		//DebugImage(image);

		//imwrite("front/" + std::to_string(counter) + ".png", image );
		counter++;
		this->RequestFrontImage();
	});
	*/
}

void RealRobot::updateForces()
{
	mTeensy->SetVelocities(GetLeftVelocity(), GetRightVelocity());
}

