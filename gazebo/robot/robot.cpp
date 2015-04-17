#include "robot.h"
#include "state.h"
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
		"timestamp INTEGER,"\
		"data COLLATE BINARY"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS teensyLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"leftWheel INTEGER, rightWheel INTEGER, accelX REAL, accelY REAL, accelZ REAL, autoFlag INTEGER"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS pointCloudLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"data COLLATE BINARY"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS decawaveLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"distance REAL"\
		")");

	mDb.exec("CREATE TABLE IF NOT EXISTS forceLog(" \
		"id INTEGER PRIMARY KEY ASC,"\
		"timestamp INTEGER,"\
		"leftForce REAL, rightForce REAL"\
		")");

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

void SensorLog::forceUpdated(double leftForce, double rightForce)
{
	QSqlQuery query(mDb);
	query.prepare("INSERT INTO forceLog(timestamp, leftForce, rightForce) VALUES "\
		"(:timestamp, :left, :right)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
	query.bindValue(":left", leftForce, QSql::In);
	query.bindValue(":right", rightForce, QSql::In);
	query.exec();

	std::vector<double> forces = {
		leftForce,
		rightForce
	};

	msgpack::sbuffer sbuf;
	msgpack::pack(sbuf, forces);

	QList<QByteArray> msg;
	msg += QByteArray("\x06");
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
	query.prepare("INSERT INTO pointCloudLog(timestamp, data) VALUES(:timestamp, :data)");
	query.bindValue(":timestamp", QDateTime::currentDateTime().toMSecsSinceEpoch() , QSql::In);
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

void SensorLog::SendObstacles(std::vector<Eigen::Vector2i> points)
{
	msgpack::sbuffer sbuf;
	msgpack::pack(sbuf, points);

	QList<QByteArray> msg;
	msg += QByteArray("\x05");
	msg += QByteArray(sbuf.data(), sbuf.size());
	mSocket->sendMessage(msg);
}

//////////////////////////
/// WheelPID
//////////////////////////

WheelPID::WheelPID(QObject* parent) : 
	mLeftDesiredVelocity(0.0), 
	mRightDesiredVelocity(0.0), 
	mLeftForce(0.0), 
	mRightForce(0.0)
{

}

void WheelPID::SetLeftDesiredVelocity(double speed)
{
	SetLeftDesiredAngularVelocity(speed / 0.155);
}

void WheelPID::SetLeftDesiredAngularVelocity(double speed)
{
	mLeftDesiredVelocity = speed;
}

void WheelPID::SetRightDesiredVelocity(double speed)
{
	SetRightDesiredAngularVelocity(speed / 0.155);
}

void WheelPID::SetRightDesiredAngularVelocity(double speed)
{
	mRightDesiredVelocity = speed;
}

void WheelPID::teensyStatus(TeenseyStatus status)
{
	QDateTime currentTime = QDateTime::currentDateTime();

	if(!mLastStatusTime.isValid() || mLastStatusTime.msecsTo(currentTime) > 1000 )
	{
		mLastStatusTime = QDateTime::currentDateTime();
		return;
	}

	double seconds = static_cast<double>(mLastStatusTime.msecsTo(currentTime)) / 1000.0;

	mLeftVelocity = static_cast<double>(mLastStatus.leftPosition - status.leftPosition) / 23330.0 / seconds;
	mRightVelocity = static_cast<double>(mLastStatus.rightPosition - status.rightPosition) / 23330.0 / seconds;

	double forceStep = 30.0;
	double forceLimit = 1000.0;

	std::cout << "Vels: " << mLeftVelocity << "\t" << mLeftDesiredVelocity << "\n";

	//TODO: Look at real implementation of PID
	if(mLeftVelocity < mLeftDesiredVelocity)
	{
		mLeftForce += forceStep;
	}
	else
	{
		mLeftForce -= forceStep;
	}

	if(mRightVelocity < mRightDesiredVelocity)
	{
		mRightForce += forceStep;
	}
	else
	{
		mRightForce -= forceStep;
	}

	if(mLeftForce >= forceLimit)
		mLeftForce = forceLimit;

	if(mLeftForce <= -forceLimit)
		mLeftForce = -forceLimit;

	if(mRightForce >= forceLimit)
		mRightForce = forceLimit;

	if(mRightForce <= -forceLimit)
		mRightForce = -forceLimit;

	mLastStatus = status;

	emit forceUpdated();
}

void WheelPID::Reset()
{
	mRightForce = 0.0;
	mLeftForce = 0.0;
	SetLeftDesiredAngularVelocity(0.0);
	SetRightDesiredAngularVelocity(0.0);

	emit forceUpdated();
}

//////////////////////////
/// Kratos2
//////////////////////////

Kratos2::Kratos2(QObject* parent) : QObject(parent), mState(nullptr)
{
	mContext = nzmqt::createDefaultContext(this);
	mContext->start();

	mSensorLog = new SensorLog(this, mContext);
	mPlanner = new TrajectoryPlanner2(this);
	mWheelPID = new WheelPID(this);
}


void Kratos2::Initialize()
{
	auto kinect = GetKinect();

	if(kinect != nullptr)
	{
		connect(kinect, &Kinect::receiveDepthImage, mSensorLog, &SensorLog::receiveDepthImage);
		connect(kinect, &Kinect::receiveDepthImage, this, &Kratos2::ProccessPointCloud);
		kinect->requestDepthFrame();
	}

	connect(GetTeensy(), &Teensy::statusUpdate, mSensorLog, &SensorLog::teensyStatus);
	connect(GetTeensy(), &Teensy::statusUpdate, mWheelPID, &WheelPID::teensyStatus);
	
	auto decawave = GetDecawave();

	if(decawave != nullptr)
	{
		connect(decawave, &Decawave::statusUpdate, mSensorLog, &SensorLog::decawaveUpdate);
	}	

	connect(&mFutureWatcher, SIGNAL(finished()), this, SLOT(FinishedPointCloud()));
	connect(mPlanner, SIGNAL(ObstacleMapUpdate(std::vector<Eigen::Vector2i>)), mSensorLog, SLOT(SendObstacles(std::vector<Eigen::Vector2i>)));
	connect(mWheelPID, &WheelPID::forceUpdated, this, &Kratos2::updateForces);

	MoveForwardState* newState = new MoveForwardState(this, 1.0);
	SetState(newState);
}

void Kratos2::updateForces()
{
	SetLeftWheelPower(mWheelPID->mLeftForce);
	SetRightWheelPower(mWheelPID->mRightForce);

	mSensorLog->forceUpdated(mWheelPID->mLeftForce, mWheelPID->mRightForce);
}

void Kratos2::ProccessPointCloud(DepthImgData mat)
{
	QDateTime currentTime = QDateTime::currentDateTime();

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

	mPlanner->UpdateObstacles(pointCloud);
	auto kinect = GetKinect();

	if(kinect != nullptr)
		kinect->requestDepthFrame();
}

void Kratos2::SetState(BaseState* nextState)
{
	nextState->setParent(this);

 	auto oldState = mState;

 	mState = nextState;
 	nextState->Start();

 	if(oldState != nullptr)
 		oldState->deleteLater();
}

#include <QSqlError>

Odometry Kratos2::GetOdometryTraveledSince(QDateTime time)
{
	QSqlQuery query(mSensorLog->mDb);
	query.prepare("SELECT leftWheel, rightWheel FROM teensyLog WHERE timestamp > :startTime");
	query.bindValue(":startTime", time.toMSecsSinceEpoch());

	if(!query.exec())
		cout << "error SQL= " <<  query.lastError().text().toStdString() << endl;

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

		//std::cout << "\tReading value: " << query.value(0).toDouble() << "\n";
		odometry.Update(
			static_cast<double>(lastLeft - left) / 23330.0 * 0.31 * M_PI,
			static_cast<double>(lastRight - right) / 23330.0 * 0.31 * M_PI);

		lastLeft = left;
		lastRight = right;
    }

	return odometry;
}


Q_DECLARE_METATYPE(Robot::DepthImgData)
Q_DECLARE_METATYPE(Robot::ImgData)
Q_DECLARE_METATYPE(Robot::TeenseyStatus)


