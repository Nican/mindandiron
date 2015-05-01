#include "trajectory2.h"
#include "robot.h"
#include "state.h"
#include <QDateTime>
#include <QDir>
#include <QSqlError>
#include <QSqlQuery>
#include <QtConcurrent>
#include <QVariant>

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

	if (mDb.lastError().isValid())
    	qDebug() << mDb.lastError();

	mDb.exec("CREATE INDEX IF NOT EXISTS depthLogTimestampIndex ON depthLog(timestamp)");
	mDb.exec("CREATE INDEX IF NOT EXISTS teensyLogTimestampIndex ON teensyLog(timestamp)");
	mDb.exec("CREATE INDEX IF NOT EXISTS pointCloudTimestampIndex ON pointCloudLog(timestamp)");
	mDb.exec("CREATE INDEX IF NOT EXISTS decawaveLogTimestampIndex ON decawaveLog(timestamp)");
}

void SensorLog::receiveDepthImage(DepthImgData mat)
{
	msgpack::sbuffer sbuf;
	msgpack::pack(sbuf, mat);

	QByteArray data = QByteArray(sbuf.data(), sbuf.size());

	QSqlQuery query(mDb);
	query.prepare("INSERT INTO depthLog(timestamp, data) VALUES(:timestamp, :data)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":data", data, QSql::In | QSql::Binary);
	query.exec();

	QList<QByteArray> msg;
	msg += QByteArray("\x01");
	msg += data;
	mSocket->sendMessage(msg);
}

void SensorLog::receiveKinectImage(Robot::ImgData mat)
{
	std::cout << "Received kinect kinectImageLog\n";

	msgpack::sbuffer sbuf;
	msgpack::pack(sbuf, mat);

	QByteArray data = QByteArray(sbuf.data(), sbuf.size());

	QSqlQuery query(mDb);
	query.prepare("INSERT INTO kinectImageLog(timestamp, data) VALUES(:timestamp, :data)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":data", data, QSql::In | QSql::Binary);
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

	msgpack::sbuffer sbuf;
	msgpack::pack(sbuf, status);

	QList<QByteArray> msg;
	msg += QByteArray("\x02");
	msg += QByteArray(sbuf.data(), sbuf.size());
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

	msgpack::sbuffer sbuf;
	msgpack::pack(sbuf, status);

	QList<QByteArray> msg;
	msg += QByteArray("\x08");
	msg += QByteArray(sbuf.data(), sbuf.size());
	mSocket->sendMessage(msg);
}


void SensorLog::receiveSegmentedPointcloud(SegmentedPointCloud pointCloud)
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
	msgpack::sbuffer sbuf;
	msgpack::pack(sbuf, points);

	QList<QByteArray> msg;
	msg += QByteArray("\x05");
	msg += QByteArray(sbuf.data(), sbuf.size());
	mSocket->sendMessage(msg);
}

void SensorLog::SetRobot(Eigen::Vector2d pos, double ang)
{
	std::vector<double> arrVals = {pos.x(), pos.y(), ang};

	msgpack::sbuffer sbuf2;
	msgpack::pack(sbuf2, arrVals);

	QList<QByteArray> msg;
	msg += QByteArray("\x09");
	msg += QByteArray(sbuf2.data(), sbuf2.size());
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

	std::vector<double> velocities = {
		left,
		right
	};

	msgpack::sbuffer sbuf;
	msgpack::pack(sbuf, velocities);

	QList<QByteArray> msg;
	msg += QByteArray("\x06");
	msg += QByteArray(sbuf.data(), sbuf.size());
	mSocket->sendMessage(msg);
}

void SensorLog::ReceivePath(const std::vector<Eigen::Vector2d> &points)
{
	msgpack::sbuffer sbuf;
	msgpack::pack(sbuf, points);

	QList<QByteArray> msg;
	msg += QByteArray("\x07");
	msg += QByteArray(sbuf.data(), sbuf.size());
	mSocket->sendMessage(msg);
}

void SensorLog::ReceiveAprilTagImage(QImage image)
{
	QByteArray buffer;

	{
		QDataStream stream(&buffer, QIODevice::WriteOnly);
		stream << image;
	}

	QSqlQuery query(mDb);
	query.prepare("INSERT INTO aprilTagImageLog(timestamp, data) VALUES(:timestamp, :data)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":data", buffer, QSql::In | QSql::Binary);
	query.exec();

	QList<QByteArray> msg;
	msg += QByteArray("\x10");
	msg += QByteArray(buffer.data(), buffer.size());
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

	
}


//////////////////////////
/// Kratos2
//////////////////////////

Kratos2::Kratos2(QObject* parent) : 
	QObject(parent),
	mLeftWheelVelocity(0.0),
	mRightWheelVelocity(0.0),
	mState(nullptr), 
	mIsPaused(false)
{
	mContext = nzmqt::createDefaultContext(this);
	mContext->start();

	mSensorLog = new SensorLog(this, mContext);
	mPlanner = new TrajectoryPlanner2(this);
	//mWheelPID = new WheelPID(this);
}


void Kratos2::Initialize()
{
	auto kinect = GetKinect();

	if(kinect != nullptr)
	{
		connect(kinect, &Kinect::receiveDepthImage, mSensorLog, &SensorLog::receiveDepthImage);
		connect(kinect, &Kinect::receiveDepthImage, this, &Kratos2::ProccessPointCloud);

		connect(kinect, &Kinect::receiveColorImage, mSensorLog, &SensorLog::receiveKinectImage);
		kinect->requestDepthFrame();
	}

	connect(GetTeensy(), &Teensy::statusUpdate, mSensorLog, &SensorLog::teensyStatus);
	connect(GetTeensy2(), &Teensy2::statusUpdate, mSensorLog, &SensorLog::teensy2Status);
	//connect(GetTeensy(), &Teensy::statusUpdate, mWheelPID, &WheelPID::teensyStatus);
	connect(GetTeensy(), &Teensy::statusUpdate, this, &Kratos2::TeensyStatus);
	
	auto decawave = GetDecawave();

	if(decawave != nullptr)
	{
		connect(decawave, &Decawave::statusUpdate, mSensorLog, &SensorLog::decawaveUpdate);
	}	

	
	connect(&mFutureWatcher, SIGNAL(finished()), this, SLOT(FinishedPointCloud()));
	//connect(mPlanner, SIGNAL(ObstacleMapUpdate(std::vector<Eigen::Vector2d>)), mSensorLog, SLOT(SendObstacles(std::vector<Eigen::Vector2d>)));
	connect(this, &Kratos2::WheelVelocityUpdate, mSensorLog, &SensorLog::WheelVelocityUpdate);
	//connect(mWheelPID, &WheelPID::forceUpdated, this, &Kratos2::updateForces);

	mState = new RootState(this);
	mState->Start();
}

void Kratos2::TeensyStatus(TeenseyStatus status)
{
	if(mIsPaused != status.autoFlag)
	{
		mIsPaused = status.autoFlag;
		std::cout << "Robot changed to paused state: " << mIsPaused << "\n";
		emit pauseUpdate(mIsPaused);
	}
}

void Kratos2::SetWheelVelocity(double left, double right)
{
	mLeftWheelVelocity = left;
	mRightWheelVelocity = right;

	emit WheelVelocityUpdate(left, right);
}

double Kratos2::GetLeftVelocity()
{
	return mLeftWheelVelocity;
}

double Kratos2::GetRightVelocity()
{
	return mRightWheelVelocity;
}


void Kratos2::ProccessPointCloud(DepthImgData mat)
{
	if(mFutureWatcher.future().isRunning()){
		std::cout << "Point cloud is still processing. not starting a new one\n";
		return;
	}

	QDateTime currentTime = QDateTime::currentDateTime();

	QFuture<SegmentedPointCloud> future = QtConcurrent::run([mat, currentTime](){

		auto imgCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		//imgCloud->sensor_origin_ = Eigen::Vector4f(lastPointPos.x(), lastPointPos.y(), lastPointPos.z(), 0.0);
		//imgCloud->sensor_orientation_ = Eigen::AngleAxisf((float) lastPoint.mRotation, Eigen::Vector3f::UnitZ());

		UpdatePointCloud(mat, *imgCloud);

		RegionGrowingSegmenter growingRegion;
		SegmentedPointCloud segmented;
		segmented.mPointCloud = growingRegion.AsyncronousUpdate(imgCloud);
		segmented.mTimestamp = currentTime;

		return segmented;
	});

	mFutureWatcher.setFuture(future);
}



void Kratos2::FinishedPointCloud()
{
	//std::cout << "Finished point cloud;\n";

	auto pointCloud = mFutureWatcher.future().result();
	mSensorLog->receiveSegmentedPointcloud(pointCloud);

	auto kinect = GetKinect();
	if(kinect != nullptr)
		kinect->requestDepthFrame();

	mPlanner->UpdateObstacles(pointCloud);
}

Odometry Kratos2::GetOdometryTraveledSince(QDateTime startTime, QDateTime endTime)
{
	QSqlQuery query(mSensorLog->mDb);
	query.prepare("SELECT leftWheel, rightWheel FROM teensyLog WHERE timestamp > :startTime AND timestamp < :endTime");
	query.bindValue(":startTime", startTime.toMSecsSinceEpoch());
	query.bindValue(":endTime", endTime.toMSecsSinceEpoch());

	if(!query.exec())
		std::cout << "error SQL= " <<  query.lastError().text().toStdString() << std::endl;

	double lastLeft = 0.0;
	double lastRight = 0.0;

	if(query.next())
	{
		lastLeft = query.value(0).toDouble();
		lastRight = query.value(1).toDouble();
	}

	Odometry odometry(0.69);

	while (query.next()) {
		double left = query.value(0).toDouble();
		double right = query.value(1).toDouble();

		double diffLeft = static_cast<double>(left - lastLeft) / 23330.0 * 0.31 * M_PI;
		double diffRight = static_cast<double>(right - lastRight) / 23330.0 * 0.31 * M_PI;

		lastLeft = left;
		lastRight = right;

		if(std::abs(diffLeft) > 0.1 || std::abs(diffRight) > 0.1){
			std::cout << "\t Skipping large jump: " << diffLeft << "\t" << diffRight << "\n";
			continue;
		}

		odometry.Update(diffLeft, diffRight);
    }

	return odometry;
}

