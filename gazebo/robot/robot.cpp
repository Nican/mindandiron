#include "robot.h"
#include <QSqlQuery>
#include <QVariant>
#include <QDateTime>
#include <QtConcurrent>
#include <QDir>

using namespace Robot;
using namespace nzmqt;

SensorLog::SensorLog(QObject* parent, nzmqt::ZMQContext* context) : 
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
	mDb.setDatabaseName(dataFolder.absolutePath() + "/" + dataFile + ".db");

	if (!mDb.open())
	{
		std::cout << "Unable to open database! :(\n";
	}

	mDb.exec("CREATE TABLE IF NOT EXISTS depthLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,"\
		"data COLLATE BINARY"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS teensyLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,"\
		"leftWheel INTEGER, rightWheel INTEGER, accelX REAL, accelY REAL, accelZ REAL, autoFlag INTEGER"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS pointCloudLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,"\
		"data COLLATE BINARY"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS decawaveLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,"\
		"distance REAL"\
		")");

	mDb.exec("CREATE INDEX IF NOT EXISTS depthLogTimestampIndex ON depthLog(timestamp)");
	mDb.exec("CREATE INDEX IF NOT EXISTS teensyLogTimestampIndex ON teensyLog(timestamp)");
	mDb.exec("CREATE INDEX IF NOT EXISTS pointCloudTimestampIndex ON pointCloudLog(timestamp)");
}

void SensorLog::receiveDepthImage(DepthImgData mat)
{
	msgpack::sbuffer sbuf;
	msgpack::pack(sbuf, mat);

	QByteArray data = QByteArray(sbuf.data(), sbuf.size());

	QSqlQuery query(mDb);
	query.prepare("INSERT INTO depthLog(data) VALUES(:data)");
	query.bindValue(":data", data, QSql::In | QSql::Binary);
	query.exec();

	QList<QByteArray> msg;
	msg += QByteArray("\x01");
	msg += data;
	mSocket->sendMessage(msg);
}

void SensorLog::teensyStatus(TeenseyStatus status)
{
	QSqlQuery query(mDb);
	query.prepare("INSERT INTO teensyLog(leftWheel, rightWheel, accelX, accelY, accelZ, autoFlag) VALUES "\
		"(:left, :right, :accelX, :accelY, :accelZ, :auto)");
	query.bindValue(":left", status.leftPosition, QSql::In);
	query.bindValue(":right", status.rightPosition, QSql::In);
	query.bindValue(":accelX", status.acceleration.x(), QSql::In);
	query.bindValue(":accelY", status.acceleration.y(), QSql::In);
	query.bindValue(":accelZ", status.acceleration.z(), QSql::In);
	query.bindValue(":auto", status.autoFlag, QSql::In);
	query.exec();

	msgpack::sbuffer sbuf;
	msgpack::pack(sbuf, status);

	QList<QByteArray> msg;
	msg += QByteArray("\x02");
	msg += QByteArray(sbuf.data(), sbuf.size());
	mSocket->sendMessage(msg);
}

void SensorLog::receiveSegmentedPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
	QByteArray data(
		reinterpret_cast<char*>(pointCloud->points.data()), 
		pointCloud->points.size() * sizeof(pcl::PointXYZRGB)
		);

	QSqlQuery query(mDb);
	query.prepare("INSERT INTO pointCloudLog(data) VALUES(:data)");
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
	query.prepare("INSERT INTO decawaveLog(distance) VALUES(:distance)");
	query.bindValue(":distance", distance, QSql::In | QSql::Binary);
	query.exec();

	QList<QByteArray> msg;
	msg += QByteArray("\x04");
	msg += QByteArray(reinterpret_cast<char*>(&distance), sizeof(double));
	mSocket->sendMessage(msg);
}

Kratos2::Kratos2(QObject* parent) : QObject(parent)
{
	mContext = nzmqt::createDefaultContext(this);
	mContext->start();

	mSensorLog = new SensorLog(this, mContext);
}


void Kratos2::Initialize()
{
	connect(GetKinect(), &Kinect::receiveDepthImage, mSensorLog, &SensorLog::receiveDepthImage);
	connect(GetTeensy(), &Teensy::statusUpdate, mSensorLog, &SensorLog::teensyStatus);

	auto decawave = GetDecawave();

	if(decawave != nullptr)
	{
		connect(decawave, &Decawave::statusUpdate, mSensorLog, &SensorLog::decawaveUpdate);
	}


	connect(GetKinect(), &Kinect::receiveDepthImage, this, &Kratos2::ProccessPointCloud);

	connect(&mFutureWatcher, SIGNAL(finished()), this, SLOT(FinishedPointCloud()));

	GetKinect()->requestDepthFrame();
}

void Kratos2::ProccessPointCloud(DepthImgData mat)
{
	QFuture<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> future = QtConcurrent::run([mat]() -> pcl::PointCloud<pcl::PointXYZRGB>::Ptr{

		auto imgCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		//imgCloud->sensor_origin_ = Eigen::Vector4f(lastPointPos.x(), lastPointPos.y(), lastPointPos.z(), 0.0);
		//imgCloud->sensor_orientation_ = Eigen::AngleAxisf((float) lastPoint.mRotation, Eigen::Vector3f::UnitZ());

		UpdatePointCloud(mat, *imgCloud);

		RegionGrowingSegmenter growingRegion;
		return growingRegion.AsyncronousUpdate(imgCloud);
	});

	mFutureWatcher.setFuture(future);
}

void Kratos2::FinishedPointCloud()
{
	auto pointCloud = mFutureWatcher.future().result();
	mSensorLog->receiveSegmentedPointcloud(pointCloud);



	GetKinect()->requestDepthFrame();
}


Q_DECLARE_METATYPE(Robot::DepthImgData)
Q_DECLARE_METATYPE(Robot::ImgData)
Q_DECLARE_METATYPE(Robot::TeenseyStatus)


