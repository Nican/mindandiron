#include "realrobot.h"
#include "gazeborobot.h"
#include <QCoreApplication>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <iostream>
#include <QFile>
#include "camera.h"

int main(int argc, char* argv[])
{
	using namespace Eigen;
	//QCoreApplication does not have a GUI
	QCoreApplication app(argc, argv);

	qRegisterMetaType<Robot::DepthImgData>("Robot::DepthImgData");	
	qRegisterMetaType<Robot::ImgData>("Robot::ImgData");
	qRegisterMetaType<Robot::TeenseyStatus>("Robot::TeenseyStatus");

	// QSqlDatabase mDb(QSqlDatabase::addDatabase("QSQLITE"));
	// mDb.setHostName("localhost");
	// mDb.setDatabaseName("/home/kratos/projects/mindandiron/gazebo/build/data/real_robot/2015_04_25_14_35_26.db");

	// if (!mDb.open())
	// {
	// 	std::cout << "Unable to open database! :(\n";
	// }

	// Eigen::Vector2d finalGoal(10,0);
	// Odometry odometry(0.69);
	// odometry.mPosition = { 0.375877, 0.0697082};
	// odometry.mTheta = 21.1342 / 180.0 * M_PI;


	// auto goal = Rotation2Dd(odometry.mTheta) * (finalGoal - odometry.mPosition);
	// std::cout << goal << "\n";


	//Expected output

	/*
	QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
	for (const QCameraInfo &cameraInfo : cameras) {
	    std::cout << "Camera: " << cameraInfo.deviceName().toStdString() << "\n";
	}
	*/

	//QFile file("/dev/v4l/by-id/usb-Chicony_USB2.0_HD_UVC_WebCam-video-index0");
	//std::cout << "Camera: " << file.symLinkTarget().toStdString() << "\n";

	//KratosCamera camera("usb-Chicony_USB2.0_HD_UVC_WebCam-video-index0", 1920, 1080);


	Robot::RealRobot robot;
	//Robot::GazeboKratos robot;
	robot.Initialize();

	return app.exec();
}


