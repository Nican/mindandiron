#include "realrobot.h"
#include "gazeborobot.h"
#include <QCoreApplication>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <iostream>
#include <QFile>
#include <QtConcurrent>
#include "camera.h"

#include "../april.h"
#include "../AprilTags/Tag25h9.h"

using namespace Eigen;

int main(int argc, char* argv[])
{
	//QCoreApplication does not have a GUI
	QCoreApplication app(argc, argv);

	qRegisterMetaType<Robot::DepthImgData>("Robot::DepthImgData");
	qRegisterMetaType<Robot::ImgData>("Robot::ImgData");
	qRegisterMetaType<Robot::TeenseyStatus>("Robot::TeenseyStatus");
	qRegisterMetaType<Robot::AprilTagDetectionItem>("Robot::AprilTagDetectionItem");
	qRegisterMetaType<QList<Robot::AprilTagDetectionItem>>("QList<AprilTagDetectionItem>");

	/*
	QSqlDatabase mDb(QSqlDatabase::addDatabase("QSQLITE"));
	mDb.setHostName("localhost");
	mDb.setDatabaseName("/home/kratos/projects/mindandiron/gazebo/build/data/robot_2015_06_08_16_13_03.db");

	if (!mDb.open())
	{
		std::cout << "Unable to open database! :(" << mDb.lastError().text().toStdString()  << "\n";
		return -1;
	}

	QSqlQuery query(mDb);
	query.prepare("SELECT timestamp, data FROM aprilTagImageLog ORDER BY timestamp");

	if(!query.exec()){
		std::cout << "error SQL= " <<  query.lastError().text().toStdString() << std::endl;
		return -1;
	}

	std::cout << "Reading results\n";
	int lastId = 0;

	while (query.next())
	{
		auto id = query.value(0).toInt();
		auto buffer = query.value(1).toByteArray();

		//std::cout << "Diff\t" << (id-lastId) << "\n";

		lastId = id;

		QtConcurrent::run([buffer, id](){
			QDataStream stream(buffer);
			QImage image;

			stream >> image;

			auto start = std::chrono::high_resolution_clock::now();
			cv::Mat tmp(image.height(), image.width(), CV_8UC4, (uchar*)image.bits(), image.bytesPerLine());
			cv::Mat cvImage(tmp.rows, tmp.cols, CV_8UC3 );

			int from_to[] = { 0,0,  1,1,  2,2 };
    		cv::mixChannels( &tmp, 1, &cvImage, 1, from_to, 3 );

			AprilTags::TagDetector detector(AprilTags::tagCodes25h9);

			cv::Mat image_gray;
			std::vector<AprilTags::TagDetection> detections;
			
			cv::cvtColor(cvImage, image_gray, CV_BGR2GRAY);

			detections = detector.extractTags(image_gray);

			QString prefix(QString::number(detections.size()));
			image.save("/home/kratos/april2/" + prefix + "_" + QString::fromStdString(std::to_string(id)) + ".png");
		
			// cv::imwrite ("/home/nican/april2/" + prefix.toStdString() + "_" + std::to_string(id) + ".png", tmp);
			auto end = std::chrono::high_resolution_clock::now();
        	std::chrono::duration<double> diff = end-start;


			std::cout << "Saved image " << id << "("<< diff.count() <<" s)\n";
		});		
	}
	*/

	/*
	QSqlQuery query(mDb);
	query.prepare("SELECT timestamp, data FROM kinectImageLog");

	if(!query.exec()){
		std::cout << "error SQL= " <<  query.lastError().text().toStdString() << std::endl;
		return -1;
	}

	while (query.next())
	{
		auto id = query.value(0).toInt();
		auto buffer = query.value(1).toByteArray();

		QtConcurrent::run([buffer, id](){
			msgpack::unpacked result;
			msgpack::unpack(result, buffer.data(), buffer.size());

			Robot::ImgData imgData;

			result.get().convert(&imgData);


			//QDataStream stream(buffer);
			QImage image(imgData.data.data(), imgData.width, imgData.height, QImage::Format_RGB888);

			//stream >> image;

			image.save("/home/nican/kinect/" + QString::fromStdString(std::to_string(id)) + ".png");

			
			//cv::Mat mat(image.height(), image.width(), CV_8UC3, image.bits(), image.bytesPerLine());
			//cv::Mat result;
			//cvtColor(mat, result, CV_BGR2RGB);
			//cv::imwrite ("april.png", mat);
			
			std::cout << "Saved image " <<id << "\n";
		});		
	}
	*/

	Robot::RealRobot robot;
	// Robot::GazeboKratos robot;
	robot.Initialize();

	return app.exec();
}


