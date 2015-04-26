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
	// mDb.setDatabaseName("/home/kratos/projects/mindandiron/gazebo/build/data/real_robot/2015_04_26_16_09_30.db");

	// if (!mDb.open())
	// {
	// 	std::cout << "Unable to open database! :(\n";
	// }

	// QSqlQuery query(mDb);
	// query.prepare("SELECT timestamp, data FROM aprilTagImageLog");

	// if(!query.exec())
	// 	std::cout << "error SQL= " <<  query.lastError().text().toStdString() << std::endl;

	// std::cout << "Reading results\n";

	// while (query.next()) 
	// {
	// 	QImage image;
	// 	auto buffer = query.value(1).toByteArray();

	// 	QDataStream stream(buffer);
	// 	stream >> image;

	// 	cv::Mat mat(image.height(), image.width(), format, image.bits(), image.bytesPerLine());

	// 	cv::imwrite ("april.png", mat);

	// 	break;
	// }


	Robot::RealRobot robot;
	//Robot::GazeboKratos robot;
	robot.Initialize();

	return app.exec();
}


