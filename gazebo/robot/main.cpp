#include "realrobot.h"
#include "gazeborobot.h"
#include <QCoreApplication>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>

int main(int argc, char* argv[])
{
	//QCoreApplication does not have a GUI
	QCoreApplication app(argc, argv);

	// QSqlDatabase mDb(QSqlDatabase::addDatabase("QSQLITE"));
	// mDb.setHostName("localhost");
	// mDb.setDatabaseName("/home/kratos/projects/mindandiron/gazebo/build/data/real_robot/2015_04_25_14_35_26.db");

	// if (!mDb.open())
	// {
	// 	std::cout << "Unable to open database! :(\n";
	// }

	Robot::RealRobot robot;
	//Robot::GazeboKratos robot;
	robot.Initialize();

	return app.exec();
}


