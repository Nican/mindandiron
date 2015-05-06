#include "trajectory2.h"
#include "robot.h"
#include "state.h"
#include <QDateTime>
#include <QtConcurrent>
#include <QVariant>
#include <QSqlError>
#include <QSqlQuery>


using namespace Robot;
using namespace Eigen;

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
	connect(kinect, &Kinect::receiveDepthImage, mSensorLog, &SensorLog::receiveDepthImage);
	connect(kinect, &Kinect::receiveDepthImage, this, &Kratos2::ProccessPointCloud);
	connect(kinect, &Kinect::receiveColorImage, mSensorLog, &SensorLog::receiveKinectImage);
	kinect->requestDepthFrame();

	connect(GetDecawave(), &Decawave::statusUpdate, mSensorLog, &SensorLog::decawaveUpdate);

	connect(GetTeensy(), &Teensy::statusUpdate, mSensorLog, &SensorLog::teensyStatus);
	connect(GetTeensy(), &Teensy::statusUpdate, this, &Kratos2::TeensyStatus);
	connect(GetTeensy2(), &Teensy2::statusUpdate, mSensorLog, &SensorLog::teensy2Status);

	connect(GetApril(), &AprilTagCamera::ReceiveFrame, mSensorLog, &SensorLog::ReceiveAprilTagImage);
	connect(GetApril(), &AprilTagCamera::tagsDetected, mSensorLog, &SensorLog::ReceiveAprilTags);
	connect(GetApril(), &AprilTagCamera::tagsDetected, this, &Kratos2::AprilTagDetected);
	
	connect(&mFutureWatcher, SIGNAL(finished()), this, SLOT(FinishedPointCloud()));
	connect(this, &Kratos2::WheelVelocityUpdate, mSensorLog, &SensorLog::WheelVelocityUpdate);

	auto timer2 = new QTimer(this);
	timer2->start(300); //time specified in ms
	QObject::connect(timer2, &QTimer::timeout, this, [this](){
		this->GetApril()->RequestFrame();
	});

	auto aprilTimer = new QTimer(this);
	aprilTimer->start(6000);
	QObject::connect(aprilTimer, &QTimer::timeout, this, &Kratos2::AprilScanTimer);

	mState = new RootState(this);
	mState->Start();
}

void Kratos2::AprilScanTimer()
{
	static QList<int> scanModeValues = {-88, -45, 0, 45, 88, 45, 0, -45};
	static int lastScanValue = 0;

	auto currentTime = QDateTime::currentDateTime();
	if(std::abs(this->mLastAprilDetection.msecsTo(currentTime)) > 4000)
	{
		std::cout << "Setting it to SCAN MODE ("<< scanModeValues[lastScanValue] <<")\n";	

		GetTeensy2()->sendRaw(scanModeValues[lastScanValue]);
		lastScanValue = (lastScanValue + 1) % scanModeValues.size();	
	}
}

void Kratos2::AprilTagDetected(QList<AprilTagDetectionItem> detections)
{
	for(auto& tag : detections)
	{
		if(tag.detection.id != 0)
			continue;

		QSqlQuery query(mSensorLog->mDb);
		query.prepare("SELECT servoAngle FROM teensy2Log WHERE timestamp > :startTime ORDER BY timestamp LIMIT 1");
		query.bindValue(":startTime", tag.time);

		if(!query.exec()){
			std::cout << "error SQL= " <<  query.lastError().text().toStdString() << std::endl;
			return;
		}

		if(!query.next())
		{	
			std::cout << "Got no data in teensy2Log" << std::endl;
			return;
		}

		double servoAngle = query.value(0).toDouble();

		mLastAprilDetection = QDateTime::currentDateTime();

		//Update the robot location estimation
		double euler = tag.euler.y() + (6.4 / 180 * M_PI);
		Vector2d translation = tag.translation.head<2>();

		translation = Rotation2Dd(euler) * translation;
		translation -= Vector2d(1.1, -0.5);

		euler -= servoAngle;

		Affine2d aprilLocation;
		aprilLocation.linear() = Rotation2Dd(euler).toRotationMatrix();
		aprilLocation.translation() = translation;

		mSensorLog->AprilLocationUpdate(QDateTime::fromMSecsSinceEpoch(tag.time), aprilLocation);

		emit AprilLocationUpdate(aprilLocation);

		//Update the teensy to update to the new angle
		double rot2 = std::atan2(tag.translation.y(), tag.translation.x());
		if(std::abs(rot2) < (10.0 * M_PI / 180.0))
			continue;

		GetTeensy2()->setAprilAngle(clamp(rot2 + servoAngle, -M_PI/2, M_PI/2));
		return;
	}
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
	if(mFutureWatcher.future().isRunning())
	{
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

	if(pointCloud.mPointCloud == nullptr)
		return;

	mSensorLog->receiveSegmentedPointcloud(pointCloud);

	auto kinect = GetKinect();
	if(kinect != nullptr)
	{
		kinect->requestColorFrame();
		kinect->requestDepthFrame();
	}

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

