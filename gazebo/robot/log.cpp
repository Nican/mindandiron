#include "robot.h"
#include <QSqlError>
#include <QSqlQuery>
#include <QtConcurrent>
#include <eigen3/Eigen/Dense>
#include <QDir>

using namespace Robot;
using namespace nzmqt;

SensorLog::SensorLog(Kratos2* parent, nzmqt::ZMQContext* context) : 
QObject(parent),
mDb(QSqlDatabase::addDatabase("QSQLITE"))
{
	mSocket = context->createSocket(ZMQSocket::TYP_PUB, this);
	mSocket->setObjectName("Publisher.Socket.socket(PUB)");
	mSocket->bindTo("tcp://*:5555");

	QDir dataFolder("data");

	if(!dataFolder.exists())
		QDir().mkdir(dataFolder.absolutePath());

	QDateTime date = QDateTime::currentDateTime();
	QString dataFile = date.toString("yyyy_MM_dd_hh_mm_ss");

	mDb.setHostName("localhost");
	mDb.setDatabaseName(dataFolder.absolutePath() + "/" + parent->Name() + "_" + dataFile + ".db");

	if (!mDb.open())
	{
		std::cout << "Unable to open database! :(\n";
	}

	//Avoid Sqlite writing to disk all the time
	mDb.exec("PRAGMA synchronous=OFF");
	//Other optimzations suggestions
	//http://stackoverflow.com/questions/784173/what-are-the-performance-characteristics-of-sqlite-with-very-large-database-file
	//http://www.sqlite.org/pragma.html

	if (mDb.lastError().isValid())
    	qDebug() << mDb.lastError();

	mDb.exec("CREATE TABLE IF NOT EXISTS depthLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"data COLLATE BINARY"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS kinectImageLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"data COLLATE BINARY"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS teensyLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"leftWheel INTEGER, rightWheel INTEGER, accelX REAL, accelY REAL, accelZ REAL, autoFlag INTEGER"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS teensy2Log(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"servoAngle REAL, current REAL, voltage REAL, paused INTEGER"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS pointCloudLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER, imageTime INTEGER,"\
		"data COLLATE BINARY"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS decawaveLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"distance REAL"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS velocityLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"leftForce REAL, rightForce REAL"\
		")");

/*
	mDb.exec("CREATE TABLE IF NOT EXISTS frontCameraImageLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"data COLLATE BINARY"\
		")");
		*/

	mDb.exec("CREATE TABLE IF NOT EXISTS aprilTagImageLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"data COLLATE BINARY"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS aprilTagLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"tagId INTEGER, x REAL, y REAL, z REAL, rp REAL, ry REAL, rr REAL, data COLLATE BINARY"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS aprilLocationLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"x REAL, y REAL, rotation REAL"\
		")");

	if (mDb.lastError().isValid())
    	qDebug() << mDb.lastError();

	mDb.exec("CREATE INDEX IF NOT EXISTS depthLogTimestampIndex ON depthLog(timestamp)");
	mDb.exec("CREATE INDEX IF NOT EXISTS teensyLogTimestampIndex ON teensyLog(timestamp)");
	mDb.exec("CREATE INDEX IF NOT EXISTS teensy2LogTimestampIndex ON teensy2Log(timestamp)");
	mDb.exec("CREATE INDEX IF NOT EXISTS pointCloudTimestampIndex ON pointCloudLog(timestamp)");
	mDb.exec("CREATE INDEX IF NOT EXISTS decawaveLogTimestampIndex ON decawaveLog(timestamp)");

	connect(&mAprilTagWatcher, SIGNAL(finished()), this, SLOT(SendAprilTagInfo()));
}

void SensorLog::AprilLocationUpdate(QDateTime time, Eigen::Affine2d location)
{
	QByteArray data(reinterpret_cast<char*>(&location), sizeof(location));

	Eigen::Rotation2Dd rotation(0);
	rotation.fromRotationMatrix(location.linear());

	QSqlQuery query(mDb);
	query.prepare("INSERT INTO aprilLocationLog(timestamp, x, y, rotation) VALUES(:timestamp, :x, :y, :rotation)");
	query.bindValue(":timestamp", time.toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":x", location.translation().x(), QSql::In);
	query.bindValue(":y", location.translation().y(), QSql::In);
	query.bindValue(":rotation", rotation.angle(), QSql::In);
	query.exec();

	QList<QByteArray> msg;
	msg += QByteArray("\x12");
	msg += data;
	mSocket->sendMessage(msg);
}

void SensorLog::receiveDepthImage(DepthImgData mat)
{
	QByteArray buffer;
	QDataStream stream(&buffer, QIODevice::WriteOnly);
	stream << mat; //.scaledToWidth(1920/3);

	QSqlQuery query(mDb);
	query.prepare("INSERT INTO depthLog(timestamp, data) VALUES(:timestamp, :data)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":data", buffer, QSql::In | QSql::Binary);
	query.exec();

	QList<QByteArray> msg;
	msg += QByteArray("\x01");
	msg += buffer;
	mSocket->sendMessage(msg);
}

void SensorLog::receiveKinectImage(Robot::ImgData mat)
{
	QByteArray buffer;
	QDataStream stream(&buffer, QIODevice::WriteOnly);
	stream << mat;

	QSqlQuery query(mDb);
	query.prepare("INSERT INTO kinectImageLog(timestamp, data) VALUES(:timestamp, :data)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":data", buffer, QSql::In | QSql::Binary);
	query.exec();
}

void SensorLog::teensyStatus(TeenseyStatus status)
{
	QSqlQuery query(mDb);
	query.prepare("INSERT INTO teensyLog(timestamp, leftWheel, rightWheel, accelX, accelY, accelZ, autoFlag) VALUES "\
		"(:timestamp, :left, :right, :accelX, :accelY, :accelZ, :auto)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":left", status.leftPosition, QSql::In);
	query.bindValue(":right", status.rightPosition, QSql::In);
	query.bindValue(":accelX", status.acceleration.x(), QSql::In);
	query.bindValue(":accelY", status.acceleration.y(), QSql::In);
	query.bindValue(":accelZ", status.acceleration.z(), QSql::In);
	query.bindValue(":auto", status.autoFlag, QSql::In);
	query.exec();

	QByteArray buffer;
	QDataStream stream(&buffer, QIODevice::WriteOnly);
	stream << status;


	QList<QByteArray> msg;
	msg += QByteArray("\x02");
	msg += buffer;
	mSocket->sendMessage(msg);
}

void SensorLog::teensy2Status(Teensy2Status status)
{
	QSqlQuery query(mDb);
	query.prepare("INSERT INTO teensy2Log(timestamp, servoAngle, current, voltage, paused) VALUES "\
		"(:timestamp, :servo, :current, :voltage, :paused)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":servo", status.servoAngle, QSql::In);
	query.bindValue(":current", status.current, QSql::In);
	query.bindValue(":voltage", status.voltage, QSql::In);
	query.bindValue(":paused", status.isPaused, QSql::In);
	query.exec();

	QByteArray buffer;
	QDataStream stream(&buffer, QIODevice::WriteOnly);
	stream << status;

	QList<QByteArray> msg;
	msg += QByteArray("\x08");
	msg += buffer;
	mSocket->sendMessage(msg);
}


void SensorLog::ReceiveSegmentedPointcloud(SegmentedPointCloud pointCloud)
{
	QByteArray data(
		reinterpret_cast<char*>(pointCloud.mPointCloud->points.data()), 
		pointCloud.mPointCloud->points.size() * sizeof(pcl::PointXYZRGB)
		);

	QSqlQuery query(mDb);
	query.prepare("INSERT INTO pointCloudLog(timestamp, data) VALUES(:timestamp, :data)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":imageTime", pointCloud.mTimestamp.toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":data", data, QSql::In | QSql::Binary);
	query.exec();

	QList<QByteArray> msg;
	msg += QByteArray("\x03");
	msg += data;
	mSocket->sendMessage(msg);
}

void SensorLog::decawaveUpdate(double distance)
{
	QSqlQuery query(mDb);
	query.prepare("INSERT INTO decawaveLog(timestamp, distance) VALUES(:timestamp, :distance)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":distance", distance, QSql::In | QSql::Binary);
	query.exec();

	QList<QByteArray> msg;
	msg += QByteArray("\x04");
	msg += QByteArray(reinterpret_cast<char*>(&distance), sizeof(double));
	mSocket->sendMessage(msg);
}

void SensorLog::SendObstacles(std::vector<Eigen::Vector2d> points)
{
	QByteArray buffer;
	QDataStream stream(&buffer, QIODevice::WriteOnly);
	stream << QVector<Eigen::Vector2d>::fromStdVector(points);

	QList<QByteArray> msg;
	msg += QByteArray("\x05");
	msg += buffer; //QByteArray(sbuf.data(), sbuf.size());
	mSocket->sendMessage(msg);
}

void SensorLog::SetRobot(Eigen::Vector2d pos, double ang)
{

	QByteArray buffer;
	QDataStream stream(&buffer, QIODevice::WriteOnly);

	stream << pos;
	stream << ang;

	QList<QByteArray> msg;
	msg += QByteArray("\x09");
	msg += buffer; //QByteArray(sbuf2.data(), sbuf2.size());
	mSocket->sendMessage(msg);
}

void SensorLog::WheelVelocityUpdate(double left, double right)
{
	QSqlQuery query(mDb);
	query.prepare("INSERT INTO velocityLog(timestamp, leftForce, rightForce) VALUES "\
		"(:timestamp, :left, :right)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":left", left, QSql::In);
	query.bindValue(":right", right, QSql::In);
	query.exec();

	QByteArray buffer;
	QDataStream stream(&buffer, QIODevice::WriteOnly);
	stream << left;
	stream << right;


	QList<QByteArray> msg;
	msg += QByteArray("\x06");
	msg += buffer; //QByteArray(sbuf.data(), sbuf.size());
	mSocket->sendMessage(msg);
}

void SensorLog::ReceivePath(const std::vector<Eigen::Vector2d> &points)
{

	QByteArray buffer;
	QDataStream stream(&buffer, QIODevice::WriteOnly);
	stream << QVector<Eigen::Vector2d>::fromStdVector(points);

	QList<QByteArray> msg;
	msg += QByteArray("\x07");
	msg += buffer; //QByteArray(sbuf.data(), sbuf.size());
	mSocket->sendMessage(msg);
}

void SensorLog::ReceiveAprilTagImage(QImage image)
{
	if(mAprilTagWatcher.future().isRunning())
		return;

	auto future = QtConcurrent::run([image](){
		//std::cout << "Start compressing april tag\n";
		QByteArray buffer;
		QDataStream stream(&buffer, QIODevice::WriteOnly);
		stream.setVersion(QDataStream::Qt_4_8);
		stream << image; //.scaledToWidth(1920/3);

		return buffer;
	});

	mAprilTagWatcher.setFuture(future);
}

void SensorLog::SendAprilTagInfo()
{
	auto buffer = mAprilTagWatcher.result();

	QSqlQuery query(mDb);
	query.prepare("INSERT INTO aprilTagImageLog(timestamp, data) VALUES(:timestamp, :data)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":data", buffer, QSql::In | QSql::Binary);
	query.exec();

	QList<QByteArray> msg;
	msg += QByteArray("\x10");
	msg += buffer;
	mSocket->sendMessage(msg);
}

void SensorLog::ReceiveAprilTags(QList<AprilTagDetectionItem> tags)
{
	auto date = QDateTime::currentDateTime().toMSecsSinceEpoch();

	for(auto& tag : tags)
	{
		QByteArray buffer(reinterpret_cast<char*>(&tag), sizeof(AprilTags::TagDetection));

		QSqlQuery query(mDb);
		query.prepare("INSERT INTO aprilTagLog(timestamp, tagId, x, y, z, rp, ry, rr, data) VALUES(:timestamp, :id, :x, :y, :z, :rp, :ry, :rr, :data)");
		query.bindValue(":timestamp", date, QSql::In);

		//auto euler = tag.rotation.eulerAngles(2,0,2);

		query.bindValue(":id", tag.detection.id, QSql::In);
		query.bindValue(":x", tag.translation.x(), QSql::In);
		query.bindValue(":y", tag.translation.y(), QSql::In);
		query.bindValue(":z", tag.translation.z(), QSql::In);
		query.bindValue(":rp", tag.euler.x(), QSql::In);
		query.bindValue(":ry", tag.euler.y(), QSql::In);
		query.bindValue(":rr", tag.euler.z(), QSql::In);
		query.bindValue(":data", buffer, QSql::In | QSql::Binary);
		query.exec();
	}

	QByteArray buffer;

	{
		QDataStream stream(&buffer, QIODevice::WriteOnly);
		stream.setVersion(QDataStream::Qt_4_8);
		stream << tags;
	}

	QList<QByteArray> msg;
	msg += QByteArray("\x11");
	msg += buffer;
	mSocket->sendMessage(msg);	
}