#include "realrobot.h"

#include <QMetaType>
#include <QThread>
#include <QtConcurrent>

#include "../AprilTags/Tag25h9.h"

using namespace Robot;


//////////////////////////
////	KratosAprilTag
//////////////////////////

KratosAprilTag::KratosAprilTag(QObject* parent) : 
	QObject(parent), 
	mTagSize(0.829),
	mFx(1315), mFy(1315),
	mPx(1920/2), mPy(1080/2)
{
	mCamera = new KratosCamera("usb-046d_HD_Pro_Webcam_C920_2245793F-video-index0", 1920, 1080, this);
	m_tagDetector.reset(new AprilTags::TagDetector(AprilTags::tagCodes25h9));

	connect(&mDetectionFutureWatcher, SIGNAL(finished()), this, SLOT(finishedProcessing()));
}

void KratosAprilTag::readCamera()
{
	if(mDetectionFutureWatcher.future().isRunning())
	{
		std::cout << "April tags is still processing. not starting a new one\n";
		return;
	}

	auto future = QtConcurrent::run([this](){
		cv::Mat image;
		cv::Mat image_gray;
		std::vector<AprilTags::TagDetection> detections;

		if(!this->mCamera->read(image))
		{
			std::cout << "Unable to read april tag camera.\n";
			return detections;
		}

		cv::cvtColor(image, image_gray, CV_BGR2GRAY);
		//cv::imshow("AA", image_gray);

		detections = m_tagDetector->extractTags(image_gray);

		return detections;
	});

	mDetectionFutureWatcher.setFuture(future);

}

inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+M_PI, M_PI * 2) - M_PI;
  } else {
    t = fmod(t-M_PI, M_PI * -2) + M_PI;
  }
  return t;
}

static void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

void KratosAprilTag::finishedProcessing()
{
	auto detections = mDetectionFutureWatcher.future().result();
	QList<AprilTagDetectionItem> detectionsItems;

	for(auto &tag : detections)
	{
		AprilTagDetectionItem item;

		tag.getRelativeTranslationRotation(mTagSize, mFx, mFy, mPx, mPy, item.translation, item.rotation);
		item.detection = tag;

		Eigen::Matrix3d F;
		F <<
		1, 0,  0,
		0,  -1,  0,
		0,  0,  1;
		item.rotation = F * item.rotation;

		double yaw, pitch, roll;
   		wRo_to_euler(item.rotation, yaw, pitch, roll);

		item.euler = {yaw, pitch, roll};
			
		std::cout << "\tTag id " << item.detection.id << "\n"; 
		std::cout << "\t\tT " << item.translation.transpose() << " ("<< item.translation.norm() <<")\n"; 
		std::cout << "\t\tR " << (item.euler / M_PI * 180.0).transpose() << "\n"; 

		detectionsItems.append(item);
	}

	emit tagsDetected(detectionsItems);
}

//////////////////////////
////	KratosKinect
//////////////////////////

KratosKinect::KratosKinect(libfreenect2::Freenect2Device *dev, QObject* parent) : 
	Robot::Kinect(parent), 
	mDev(dev)
{	
	mDev->start();
}

void KratosKinect::requestDepthFrame()
{
	std::lock_guard<std::mutex> lock(mRequestLock);

	depthRequested = true;
	mDev->setIrAndDepthFrameListener(this);
}

void KratosKinect::requestColorFrame()
{
	std::lock_guard<std::mutex> lock(mRequestLock);

	colorRequested = true;
	mDev->setColorFrameListener(this);
}


bool KratosKinect::onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame)
{
	std::lock_guard<std::mutex> lock(mRequestLock);

	std::cout << "Received new frame\n";

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

RealRobot::RealRobot(QObject* parent) : 
	Robot::Kratos2(parent),
	mKinect(nullptr),
	bFirstTeenseyMessage(true)
{

	mDecawave = new Robot::KratosDecawave(mContext, this);
	mTeensy = new Robot::KratosTeensy(this);
	mAprilTag = new Robot::KratosAprilTag(this);

	QObject::connect(mTeensy, &Robot::KratosTeensy::statusUpdate, [this](Robot::TeenseyStatus status){
		bFirstTeenseyMessage = false;
		lastStatus = status;
	});


	auto timer = new QTimer(this);
	timer->start(100); //time specified in ms
	QObject::connect(timer, SIGNAL(timeout()), this, SLOT(updateForces()));

	auto dev = freenect2.openDefaultDevice();

	if(dev == nullptr)
	{
		std::cout << "no device connected or failure opening the default one!" << std::endl;
	}
	else
	{
		mKinect = new KratosKinect(dev, this);
		mKinect->requestDepthFrame();
	}

	connect(mAprilTag->mCamera, &KratosCamera::CameraFrame, mSensorLog, &SensorLog::ReceiveAprilTagImage);

	auto timer2 = new QTimer(this);
	timer2->start(1000/3); //time specified in ms
	QObject::connect(timer, &QTimer::timeout, this, [this](){
		this->mAprilTag->readCamera();
	});

	// connect(mAprilTag, &KratosAprilTag::tagsDetected, this, [](QList<AprilTagDetectionItem> detections){
	// 	std::cout << "Found " << detections.size() << " april tag entries\n";
	// 	for(auto& tag : detections)
	// 	{
	// 		auto euler = tag.rotation.eulerAngles(2,0,2) * 180.0 / M_PI;
	// 		std::cout << "\tTag id " << tag.detection.id << "\n"; 
	// 		std::cout << "\t\tT " << tag.translation.transpose() << "\n"; 
	// 		std::cout << "\t\tR " << euler.transpose() << "\n"; 
	// 	}
	// });

}




void RealRobot::updateForces()
{
	mTeensy->SetVelocities(GetLeftVelocity(), GetRightVelocity());
}

