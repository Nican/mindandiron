#include "gazeborobot.h"
#include <QDebug>
#include <iostream>
#include <QTimer>

using namespace Robot;
using namespace Eigen;


void GazeboSampleDetection::receiveUpdate(const RobotGazeboTickData &data)
{
	static const Vector2d sampleLocation(12, 12);

	Vector2d relative = sampleLocation - data.robotPosition.head<2>();
	//double angle = std::atan2(relative.y(), relative.x());

	//std::cout << "Robot angle: " << data.robotOrientation << "\n";
	//std::cout << "\tangle: " << angle << "\n";

	//double relativeAngle = angle - data.robotOrientation;

	Vector2d relative2 = Rotation2Dd(data.robotOrientation) * relative;

	//Assume that the detector can see objects at most 10m away
	if(relative2.norm() > 10.0)
		return;

	emit SampleDetected(relative2.x(), relative2.y());
}

GazeboAprilTag::GazeboAprilTag(GazeboKratos* parent) 
	: AprilTagCamera(parent), mFrameRequested(false), robot(parent)
{
}

void GazeboAprilTag::RequestFrame()
{
	mFrameRequested = true;
}

void GazeboAprilTag::finishedProcessing()
{
	auto detections = mDetectionFutureWatcher.future().result();
	QList<AprilTagDetectionItem> detectionsItems;

	RobotGazeboTickData tick = robot->mTeensey->mLastTick;
	double robotCameraAngle = M_PI - (robot->mTeensy2->mServoAngle * M_PI / 180.0);

	//std::cout << "Robot rotation " << tick.robotOrientation << "\t" << robotCameraAngle << "\n";

	for(auto &tag : detections)
	{
		AprilTagDetectionItem item;
		item.detection = tag;
		item.time = lastFrameTime.toMSecsSinceEpoch();

		Affine3d affine;
		Vector3d normal(0.9936, 0.1121, 0); 
		Vector3d forward(1, 0, 0);

		if(item.detection.id == 0)
		{
			affine.linear() = Quaterniond::FromTwoVectors(normal, forward).toRotationMatrix();
			affine.translation() = Vector3d(-1.0565, 0.5, 1.068);
		}
		else if (item.detection.id == 1)
		{
			normal.y() *= -1;
			affine.linear() = Quaterniond::FromTwoVectors(normal, forward).toRotationMatrix();
			affine.translation() = Vector3d(-1.0565, -0.5, 1.068);
		}
		else
		{
			std::cout << "Reading unkown tag id: " << item.detection.id << "\n";
			continue;
		}

		auto rotation = AngleAxisd(-tick.robotOrientation + robotCameraAngle, Vector3d::UnitZ());

		affine.translation() -= tick.robotPosition;
		affine.translation().z() -= 0.6;

		//std::cout << "Found sample: " << item.detection.id << "(" << affine.translation().transpose() << ")\n";

		affine.translation() = rotation * affine.translation();
		affine.linear() = AngleAxisd(tick.robotOrientation - robotCameraAngle + M_PI, Vector3d::UnitZ()) * affine.linear();

		//std::cout << "\tAfter Rotation: " << affine.translation().transpose() << "\n";
		//std::cout << "\tAngle " << (affine.linear().eulerAngles(1,2,0) * 180 / M_PI).transpose() << "\n";

		item.translation = affine.translation();
		item.rotation = affine.linear();
		item.euler = affine.linear().eulerAngles(1,2,0);
		//item.euler.y() = M_PI - item.euler.y();

		//30m limit range
		if(item.translation.norm() > 30.0)
			continue;

		detectionsItems.append(item);
	}

	emit tagsDetected(detectionsItems);
}


GazeboDevawave::GazeboDevawave(QObject* parent) 
	: Decawave(parent)
{
	mUpdateTimer = new QTimer(this);
	mUpdateTimer->setSingleShot(true);

	connect(mUpdateTimer, SIGNAL(timeout()), this, SLOT(fireUpdate()));
}

void GazeboDevawave::receiveUpdate(const RobotGazeboTickData &data)
{
	mLastTick = data;

	if(!mUpdateTimer->isActive())
		mUpdateTimer->start(1000);
}

void GazeboDevawave::fireUpdate()
{
	double distance = mLastTick.robotPosition.norm();
	emit statusUpdate(distance);
}



void GazeboKinect::requestDepthFrame()
{
	depthFrameRequested = true;
}

void GazeboKinect::requestColorFrame()
{
	colorFrameRequested = true;
}


GazeboTeensey::GazeboTeensey(QObject* parent) 
	: Teensy(parent)
{
	mUpdateTimer = new QTimer(this);
	mUpdateTimer->setSingleShot(true);

	connect(mUpdateTimer, SIGNAL(timeout()), this, SLOT(fireUpdate()));
}

void GazeboTeensey::receiveUpdate(const RobotGazeboTickData &data)
{
	mLastTick = data;

	if(!mUpdateTimer->isActive())
		mUpdateTimer->start(100);
}

void GazeboTeensey::fireUpdate()
{
	Robot::TeenseyStatus status;
	status.leftPosition = mLastTick.leftWheelTicks;
	status.rightPosition = mLastTick.rightWheelTicks;
	status.leftVelocity = mLastTick.leftWheelVelocity;
	status.rightVelocity = mLastTick.rightWheelVelocity;
	status.acceleration = mLastTick.linearAcceleration;
	status.autoFlag = false;

	emit statusUpdate(status);
}

GazeboTeensey2::GazeboTeensey2(QObject* parent) 
	: Teensy2(parent), mServoAngle(0)
{
	mUpdateTimer = new QTimer(this);
	mUpdateTimer->setSingleShot(true);

	connect(mUpdateTimer, SIGNAL(timeout()), this, SLOT(fireUpdate()));
}

void GazeboTeensey2::receiveUpdate(const RobotGazeboTickData &data)
{
	mLastTick = data;

	if(!mUpdateTimer->isActive())
		mUpdateTimer->start(100);
}

void GazeboTeensey2::fireUpdate()
{
	Robot::Teensy2Status status;
	status.servoAngle = mServoAngle / 180.0 * M_PI;
	status.current = 10.0;
	status.voltage = 10.0;
	status.isPaused = false;

	lastStatus = status;
	emit statusUpdate(status);
}

void GazeboTeensey2::sendRaw(int intAngle)
{
	mServoAngle = intAngle;
}

double GazeboTeensey2::GetAprilGazeboAngle()
{
	return (static_cast<double>(mServoAngle) * M_PI / 180.0) - M_PI;
}


GazeboKratos::GazeboKratos(QObject* parent) 
	: Kratos2(parent)
{
	mSubSocket = mContext->createSocket(nzmqt::ZMQSocket::TYP_SUB, this);
	mSubSocket->setObjectName("Subscriber.Socket.socket(SUB)");
	mSubSocket->connectTo("tcp://127.0.0.1:5556");
	mSubSocket->setOption(nzmqt::ZMQSocket::OPT_SUBSCRIBE, "", 0);

	mPubSocket = mContext->createSocket(nzmqt::ZMQSocket::TYP_PUB, this);
	mPubSocket->setObjectName("Publisher.Socket.socket(PUB)");
	mPubSocket->bindTo("tcp://*:5557");

	mTeensey = new GazeboTeensey(this);
	mKinect = new GazeboKinect(this);
	mDecaWave = new GazeboDevawave(this);
	mTeensy2 = new GazeboTeensey2(this);
	mAprilTag = new GazeboAprilTag(this);
	mSampleDetection = new GazeboSampleDetection(this);

	mSendControlTimer = new QTimer(this);
	mSendControlTimer->start(1000/30);

	connect(mSendControlTimer, SIGNAL(timeout()), this, SLOT(fireControlUpdate()));
	connect(mSubSocket, SIGNAL(messageReceived(const QList<QByteArray>&)), SLOT(messageReceived(const QList<QByteArray>&)));
}

void GazeboKratos::fireControlUpdate()
{
	RobotGazeboControl control;
	control.leftVelocity = GetLeftVelocity();
	control.rightVelocity = GetRightVelocity();
	control.aprilAngle = mTeensy2->GetAprilGazeboAngle();

	char id = 0;

	msgpack::sbuffer sbuf;
	sbuf.write(&id, sizeof(id));
	msgpack::pack(sbuf, control);

	QList<QByteArray> msg;
	msg += QByteArray(sbuf.data(), sbuf.size());
	mPubSocket->sendMessage(msg);
}

void GazeboKratos::messageReceived(const QList<QByteArray>& messages)
{
	//std::cout << "Message size: " << message.size () << "\n";

	//Why is this a list of arrays?
	for(auto& byteArray : messages)
	{
		auto id = byteArray[0];

		//Read result from the network
		msgpack::unpacked result;
		try { 
			//Skip the first byte since that is the ID
			msgpack::unpack(result, byteArray.data()+1, byteArray.size() - 1);
		}
		catch(std::exception &e)
		{
			std::cerr << "Failed to parse: " << e.what() << "\n";
			return;
		}

		if(id == 0)
		{
			RobotGazeboTickData tickData;
			result.get().convert(&tickData);

			mTeensey->receiveUpdate(tickData);
			mTeensy2->receiveUpdate(tickData);
			mDecaWave->receiveUpdate(tickData);
			mSampleDetection->receiveUpdate(tickData);
		}
		else if(id == 1)
		{
			Robot::ImgData data;
			result.get().convert(&data);

			if(mKinect->depthFrameRequested)
			{
				emit mKinect->receiveColorImage(data);
				mKinect->depthFrameRequested = false;
			}
		}
		else if(id == 2)
		{
			Robot::DepthImgData data;
			result.get().convert(&data);

			if(mKinect->depthFrameRequested)
			{
				emit mKinect->receiveDepthImage(data);
				mKinect->depthFrameRequested = false;
			}
			
		}
		else if(id == 3)
		{
			Robot::ImgData data;
			result.get().convert(&data);

			//std::cout << "Robot data image april size: ("<< data.data.size() <<")\n";

			if(mAprilTag->mFrameRequested)
			{
				QImage image(data.data.data(), data.width, data.height, QImage::Format_RGB888);

				emit mAprilTag->ReceiveFrame(image);
				mAprilTag->mFrameRequested = false;
			}
		}
		else
		{
			std::cout << "Received unkown message from gazebo with id " << id << "\n";
		}
	}
}
