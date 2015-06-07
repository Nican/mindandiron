#include "gazeborobot.h"
#include "../msgpack.h"
#include <QDebug>
#include <iostream>
#include <QTimer>
#include <QProcess>

using namespace Robot;
using namespace Eigen;


void GazeboSampleDetection::receiveUpdate(const RobotGazeboTickData &data)
{
	gazeboData = data;
}

void GazeboSampleDetection::TeensyStatus(TeenseyStatus status)
{
	static const Vector2d sampleLocation(-20, 20);
	QList<DetectedSample> detections;

	Vector2d relative = sampleLocation - gazeboData.robotPosition.head<2>();
	Vector2d relative2 = Rotation2Dd(-gazeboData.robotOrientation) * relative;

	//Assume that the detector can see objects at most 10m away
	if(relative2.norm() <= 10.0)
	{
		//Or more than 90 degrees from the view of view
		if(abs(atan2(relative2.y(), relative2.x())) <= (45.0 * M_PI / 180.0))
		{
			DetectedSample sample;
		 	sample.location = relative2;
			sample.name = "test";

			detections += sample;
		}
	}

	mLastDetection = detections;
	emit SampleDetected(detections);
}

GazeboAprilTag::GazeboAprilTag(GazeboKratos* parent) 
	: AprilTagCamera(1, 1, 1920, 1080, parent), robot(parent)
{
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

		item.translation = affine.translation();
		item.rotation = affine.linear();
		item.euler = affine.linear().eulerAngles(1,2,0);
		//item.euler.y() = M_PI - item.euler.y();

		//30m limit range
		if(item.translation.norm() > 15.0)
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
	lastDistance = distance;
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
	status.autoFlag = true;

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

	connect(mTeensey, &Teensy::statusUpdate, mSampleDetection, &GazeboSampleDetection::TeensyStatus);

	QString program = "gazebo";
    QStringList arguments;
    arguments << "/home/kratos/projects/mindandiron/models/kratos_flat.world";

    QProcess *myProcess = new QProcess();
    myProcess->start(program, arguments);
}

void GazeboKratos::fireControlUpdate()
{
	RobotGazeboControl control;
	control.leftVelocity = GetLeftVelocity() * 2.5;
	control.rightVelocity = GetRightVelocity() * 2.5;
	control.aprilAngle = mTeensy2->GetAprilGazeboAngle();

	QByteArray buffer;
	QDataStream stream(&buffer, QIODevice::WriteOnly);
	stream << control;

	QList<QByteArray> msg;
	msg += buffer; //QByteArray(sbuf.data(), sbuf.size());
	mPubSocket->sendMessage(msg);
}

void GazeboKratos::messageReceived(const QList<QByteArray>& messages)
{
	//std::cout << "Message size: " << message.size () << "\n";

	//Why is this a list of arrays?
	for(auto& byteArray : messages)
	{
		QDataStream stream(byteArray);

		qint8 id;
		stream >> id;

		if(id == 0)
		{
			RobotGazeboTickData tickData;
			stream >> tickData;

			mTeensey->receiveUpdate(tickData);
			mTeensy2->receiveUpdate(tickData);
			mDecaWave->receiveUpdate(tickData);
			mSampleDetection->receiveUpdate(tickData);
		}
		else if(id == 1)
		{
			Robot::ImgData data;
			stream >> data;

			if(mKinect->colorFrameRequested)
			{
				emit mKinect->receiveColorImage(data);
				mKinect->colorFrameRequested = false;
			}
		}
		else if(id == 2)
		{
			Robot::DepthImgData data;
			stream >> data;

			if(mKinect->depthFrameRequested)
			{
				emit mKinect->receiveDepthImage(data);
				mKinect->depthFrameRequested = false;
			}
			
		}
		else if(id == 3)
		{
			Robot::ImgData data;
			stream >> data;

			QImage image(data.data.data(), data.width, data.height, QImage::Format_RGB888);
			emit mAprilTag->ReceiveFrame(image);
		}
		else
		{
			std::cout << "Received unkown message from gazebo with id " << id << "\n";
		}

		/*
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
				mAprilTag->mFrameRequested = false;

				emit mAprilTag->ReceiveFrame(image);
			}
		}
		else
		{
			std::cout << "Received unkown message from gazebo with id " << id << "\n";
		}
		*/
	}
}
