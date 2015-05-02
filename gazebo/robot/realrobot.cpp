#include "realrobot.h"

#include <QMetaType>
#include <QThread>
#include <QtConcurrent>

#include "../AprilTags/Tag25h9.h"

using namespace Robot;


//////////////////////////
////	KratosAprilTag
//////////////////////////

static void DebugImage(cv::Mat &input)
{
	cv::Mat output;
	cv::resize(input, output, {960, 540});

	cv::imshow("AA", output);
}

KratosAprilTag::KratosAprilTag(QObject* parent) : 
	AprilTagCamera(parent)
{
	mCamera = new KratosCamera("usb-046d_HD_Pro_Webcam_C920_2245793F-video-index0", 1920, 1080, this);
}

void KratosAprilTag::RequestFrame()
{
	QtConcurrent::run([this]()
	{
		cv::Mat image;

		if(!this->mCamera->read(image))
		{
			std::cout << "Unable to read april tag camera.\n";
			return;
		}

		//DebugImage(image);
		
		cv::Mat temp; // make the same cv::Mat
		cvtColor(image, temp, CV_BGR2RGB); // cvtColor Makes a copt, that what i need
		QImage dest(reinterpret_cast<const uchar*>(temp.data), temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
		dest.bits(); // enforce deep copy, see documentation of QImage::QImage ( const uchar * data, int width, int height, Format format )

		emit this->ReceiveFrame(dest);
	});


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

		emit receiveDepthImage(imgData);
	}

	//Return false so that the library re-uses the "frame" pointer
	return false;

};


//////////////////////////
////	RealRobot
//////////////////////////

inline double clamp(double x, double a, double b)
{
	return x < a ? a : (x > b ? b : x);
}

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

	QObject::connect(mTeensy, &Robot::KratosTeensy::statusUpdate, [this](Robot::TeenseyStatus status){
		bFirstTeenseyMessage = false;
		lastStatus = status;
	});

	mKinect = new KratosKinect(this);
	mKinect->requestDepthFrame();

	auto timer = new QTimer(this);
	timer->start(100); //time specified in ms
	connect(timer, &QTimer::timeout, this, &RealRobot::updateForces);

	auto timer4 = new QTimer(this);
	timer4->start(3000);
	QObject::connect(timer4, &QTimer::timeout, this, [this](){
		static QList<int> scanModeValues = {-88, -45, 0, 45, 88, 45, 0, -45};
		static int lastScanValue = 0;

		auto currentTime = QDateTime::currentDateTime();
		if(std::abs(this->mLastAprilDetection.msecsTo(currentTime)) > 4000)
		{
			std::cout << "Setting it to SCAN MODE ("<< scanModeValues[lastScanValue] <<")\n";	

			//this->mTeensy2->sendRaw(255);
			this->mTeensy2->sendRaw(scanModeValues[lastScanValue]);
			lastScanValue = (lastScanValue + 1) % scanModeValues.size();	
		}
	});

	connect(mAprilTag, &KratosAprilTag::tagsDetected, this, [this](QList<AprilTagDetectionItem> detections){

		double readValue = this->mTeensy2->lastStatus.servoAngle;

		for(auto& tag : detections)
		{
			if(tag.detection.id != 0)
				continue;

			this->mLastAprilDetection = QDateTime::currentDateTime();

			double rot2 = std::atan2(tag.translation.y(), tag.translation.x());

			if(std::abs(rot2) < (10.0 * M_PI / 180.0))
				continue;

			double target = clamp(rot2 + readValue, -M_PI/2, M_PI/2);

			// std::cout << "Moving camera:\n";
			// std::cout << "\t Teensy2 read value: " << readValue*180/M_PI << "\n";
			// std::cout << "\t :Angle of the tag: " << rot2*180/M_PI << "\n";
			// std::cout << "\t :New target: " << target*180/M_PI << "\n";

			this->mTeensy2->setAprilAngle(target);
		}

	});


	auto timer3 = new QTimer(this);
	timer3->start(1000); //time specified in ms
	QObject::connect(timer3, &QTimer::timeout, this, [this](){
		if(this->mKinect != nullptr)
			this->mKinect->requestColorFrame();
	});
}

void RealRobot::updateForces()
{
	mTeensy->SetVelocities(GetLeftVelocity(), GetRightVelocity());
}

