#include "gazeborobot.h"
#include <QDebug>
#include <iostream>
#include <QTimer>

using namespace Robot;

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
	status.acceleration = mLastTick.linearAcceleration;
	status.autoFlag = false;

	//std::cout << "Emmiting status update " << mLastTick.linearAcceleration << "\n";

	emit statusUpdate(status);
}

GazeboKratos::GazeboKratos(QObject* parent) 
	: Kratos2(parent)
{
	mSocket = mContext->createSocket(nzmqt::ZMQSocket::TYP_SUB, this);
	mSocket->setObjectName("Subscriber.Socket.socket(SUB)");
	mSocket->connectTo("tcp://127.0.0.1:5556");
	mSocket->setOption(nzmqt::ZMQSocket::OPT_SUBSCRIBE, "", 0);

	mTeensey = new GazeboTeensey(this);
	mKinect = new GazeboKinect(this);
	mDecaWave = new GazeboDevawave(this);

	connect(mSocket, SIGNAL(messageReceived(const QList<QByteArray>&)), SLOT(messageReceived(const QList<QByteArray>&)));
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
	    	mDecaWave->receiveUpdate(tickData);
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
	    	
	    }

	}

}
