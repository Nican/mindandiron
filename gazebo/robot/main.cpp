#include "realrobot.h"
#include "gazeborobot.h"
#include <QCoreApplication>

int main(int argc, char* argv[])
{
	//QCoreApplication does not have a GUI
	QCoreApplication app(argc, argv);
	//qRegisterMetaType<Robot::DepthImgData>("");	
	//qRegisterMetaType<Robot::ImgData>("Robot::ImgData");
	//qRegisterMetaType<Robot::TeenseyStatus>("Robot::TeenseyStatus");


	Robot::RealRobot robot;
	//Robot::GazeboKratos robot;
	robot.Initialize();

	// ...
	return app.exec();
}


