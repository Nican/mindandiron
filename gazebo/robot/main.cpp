#include "realrobot.h"
#include "gazeborobot.h"
#include <QCoreApplication>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <iostream>
#include <QFile>
#include "camera.h"

using namespace Eigen;

struct TestItem
{
	double rot; 
	Vector3d value;
	Vector3d expected;
};

int main(int argc, char* argv[])
{
	//QCoreApplication does not have a GUI
	QCoreApplication app(argc, argv);

	qRegisterMetaType<Robot::DepthImgData>("Robot::DepthImgData");
	qRegisterMetaType<Robot::ImgData>("Robot::ImgData");
	qRegisterMetaType<Robot::TeenseyStatus>("Robot::TeenseyStatus");

	/*
	QSqlDatabase mDb(QSqlDatabase::addDatabase("QSQLITE"));
	mDb.setHostName("localhost");
	mDb.setDatabaseName("/home/nican/2015_04_26_16_09_30.db");

	if (!mDb.open())
	{
		std::cout << "Unable to open database! :(" << mDb.lastError().text().toStdString()  << "\n";
		return -1;
	}

	QSqlQuery query(mDb);
	query.prepare("SELECT timestamp, data FROM aprilTagImageLog");

	if(!query.exec()){
		std::cout << "error SQL= " <<  query.lastError().text().toStdString() << std::endl;
		return -1;
	}

	std::cout << "Reading results\n";

	while (query.next())
	{
		QImage image;
		auto buffer = query.value(1).toByteArray();

		QDataStream stream(buffer);
		stream >> image;

		image.save("april2.png");

		cv::Mat mat(image.height(), image.width(), CV_8UC3, image.bits(), image.bytesPerLine());
		cv::Mat result;
		cvtColor(mat, result, CV_BGR2RGB);

		cv::imwrite ("april.png", mat);

		break;
	}
	*/
 
 /*
	std::vector<TestItem> vecs = {
		{M_PI / 2, {-3, 5.5, 0}, {5.5, 3.0, 0}},
		{M_PI, {-3, 0.5, 0}, {3, -0.5, 0}},
		{M_PI, {-3, -0.5, 0}, {3, 0.5, 0}},
		{-M_PI / 2, {-3, -5.5, 0}, {5.5, -3.0, 0}}
	};

	double cameraRot = M_PI;

	for(auto& item : vecs)
	{
		auto pos = AngleAxisd(-item.rot, Vector3d::UnitZ()) * (item.value);

		auto rotation = AngleAxisd(M_PI - cameraRot, Vector3d::UnitZ());
		auto pos2 = rotation * pos;

		std::cout << pos2.transpose() << "\n";
		std::cout << item.expected.transpose() << "\n";
		std::cout << "\n";
	}


	return 0;
	*/

	//Robot::RealRobot robot;
	Robot::GazeboKratos robot;
	robot.Initialize();

	return app.exec();
}


