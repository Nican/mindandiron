#include "realrobot.h"
#include <QCoreApplication>
#include <QThread>
#include <QMetaType>


class AprilTagCameraMotorReal : public Robot::AprilTagServo
{
public:

  AprilTagCameraMotorReal()
  {
  }

  virtual double GetPosition() const override
  {
    return 0.0;
  }

  virtual void SetPosition(double radians) override
  {
  }
};

class WheelJointReal : public Robot::Wheel
{
public:
  WheelJointReal()
  {
  }

  virtual double GetRotationVelocity() const override
  {
    return 0.0;
  }

  virtual void SetForce(double force) override
  {
  }

};

class TRSReal : public Robot::TotalRoboticStation 
{
public:

  TRSReal()
  {
  }

  virtual Eigen::Vector3d GetPosition() const override
  {
    return Eigen::Vector3d(0, 0, 0);
  }

  virtual double GetOrientation() const override
  {
    return 0.0;
  }
};

KratosKinect::KratosKinect(libfreenect2::Freenect2Device *dev, QObject* parent) : QObject(parent), mDev(dev)
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

RealRobot::RealRobot() : mKinect(nullptr)
{


	Robot::RobotMotion motion;
	motion.mAprilServo = std::make_shared<AprilTagCameraMotorReal>();
	motion.mLeftWheel = std::make_shared<WheelJointReal>();
	motion.mRightWheel = std::make_shared<WheelJointReal>();

	Robot::RobotSensors sensors;
	sensors.mTRS = std::make_shared<TRSReal>();

	m_kratos = std::make_shared<Robot::Kratos>(motion, sensors);

	mTeensey = new Robot::Teensey(this);
	QObject::connect(mTeensey, &Robot::Teensey::statusUpdate, [](Robot::TeenseyStatus){
		//std::cout << "Got update!\n";
	});


    timer = new QTimer(this);
    timer->start(1000); //time specified in ms

    auto dev = freenect2.openDefaultDevice();

    if(dev == nullptr)
	{
		std::cout << "no device connected or failure opening the default one!" << std::endl;
	}
	else
	{
		mKinect = new KratosKinect(dev, this);
		mKinect->requestDepthFrame();
		
		QObject::connect(timer, SIGNAL(timeout()), this, SLOT(showGPS()));
		QObject::connect(mKinect, SIGNAL(receiveColorImage(Robot::ImgData)), this, SLOT(receiveColorImage2(Robot::ImgData)));
		QObject::connect(mKinect, SIGNAL(receiveDepthImage(Robot::DepthImgData)), this, SLOT(receiveDepthImage2(Robot::DepthImgData)));
	}

}



void RealRobot::receiveColorImage2(Robot::ImgData mat)
{
	//std::cout << "Received color image2\t" << QThread::currentThreadId() << "\n";
}

void RealRobot::receiveDepthImage2(Robot::DepthImgData image)
{
	std::cout << "Sending frame out!\n";
	//std::cout << "Received depth image2\t" << QThread::currentThreadId() << "\n";	

	if(m_kratos != nullptr)
		m_kratos->ReceiveDepth(image);
}




void RealRobot::showGPS()
{
	if(mKinect != nullptr)
		mKinect->requestDepthFrame();
    //qDebug()<<Q_FUNC_INFO;
    //std::cout << "AAAA\t" << QThread::currentThreadId() << "\n";
}


int main(int argc, char* argv[])
{
	//QCoreApplication does not have a GUI
	QCoreApplication app(argc, argv);
	qRegisterMetaType<Robot::DepthImgData>("Robot::DepthImgData");	
	qRegisterMetaType<Robot::ImgData>("Robot::ImgData");
	qRegisterMetaType<Robot::TeenseyStatus>("Robot::TeenseyStatus");

	RealRobot myWidget;

	// ...
	return app.exec();
}


