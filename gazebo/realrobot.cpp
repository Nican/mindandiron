#include "realrobot.h"
#include <QCoreApplication>
#include <QThread>
#include <QMetaType>



RealRobot::RealRobot()
{
	

    timer = new QTimer(this);
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(showGPS()));
    timer->start(1000); //time specified in ms

    this->dev = freenect2.openDefaultDevice();

    if(this->dev == 0)
	{
		std::cout << "no device connected or failure opening the default one!" << std::endl;
	    return;
	}

	dev->setColorFrameListener(this);
	dev->setIrAndDepthFrameListener(this);
	dev->start();

	QObject::connect(this, SIGNAL(receiveColorImage(cv::Mat)), this, SLOT(receiveColorImage2(cv::Mat)));

}

bool RealRobot::onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame)
{
	
	if(type == libfreenect2::Frame::Color)
	{
		cv::Mat mat(frame->height, frame->width, CV_8UC3, frame->data);
		emit receiveColorImage(mat.clone());

	}

	if(type == libfreenect2::Frame::Depth)
	{
		cv::Mat mat(frame->height, frame->width, CV_32FC1, frame->data);
		emit receiveDepthImage(mat.clone());
	}

	//Who owns this pointer anyway?
	//delete frame;

};
/*
void RealRobot::receiveColorImage(cv::Mat mat)
{
	std::cout << "Received color image\t" << QThread::currentThreadId() << "\n";
}

void RealRobot::receiveDepthImage(cv::Mat mat)
{
	std::cout << "Received depth image\t" << QThread::currentThreadId() << "\n";
}
*/

void RealRobot::receiveColorImage2(cv::Mat mat)
{
	std::cout << "Received color image2\t" << QThread::currentThreadId() << "\n";

	//cv::imshow("rgb", mat);
}



void RealRobot::showGPS()
{
    //qDebug()<<Q_FUNC_INFO;
    std::cout << "AAAA\t" << QThread::currentThreadId() << "\n";
}


int main(int argc, char* argv[])
{
	//QCoreApplication does not have a GUI
	QCoreApplication app(argc, argv);
	qRegisterMetaType< cv::Mat >("cv::Mat");	

	RealRobot myWidget;

	// ...
	return app.exec();
}


