#include "trajectory2.h"
#include "robot.h"
#include "state/state.h"
#include <QDateTime>
#include <QtConcurrent>
#include <QVariant>
#include <QSqlError>
#include <QSqlQuery>
#include <QThreadPool>


using namespace Robot;
using namespace Eigen;

//////////////////////////
/// Kratos2
//////////////////////////

LocationEstimation::LocationEstimation(Kratos2* parent) : QObject(parent), robot(parent)
{
	auto updateTimer = new QTimer(this);
	updateTimer->start(200);
	QObject::connect(updateTimer, &QTimer::timeout, this, &LocationEstimation::SendUpdate);
}

void LocationEstimation::teensyStatus(TeenseyStatus status)
{
	double diffLeft = static_cast<double>(status.leftPosition - lastTeensyStatus.leftPosition) / 23330.0 * 0.31 * M_PI;
	double diffRight = static_cast<double>(status.rightPosition - lastTeensyStatus.rightPosition) / 23330.0 * 0.31 * M_PI;

	lastTeensyStatus = status;

	if(std::abs(diffLeft) > 0.1 || std::abs(diffRight) > 0.1){
		//std::cout << "\t Skipping large jump: " << diffLeft << "\t" << diffRight << "\n";
		return;
	}

	odometry.Update(diffLeft, diffRight);
}

Eigen::Affine2d LocationEstimation::GetEstimate()
{
	Affine2d location;
	location.translation() = odometry.mPosition;
	location.linear() = Rotation2Dd(odometry.mTheta).toRotationMatrix();

	return location;
}

void LocationEstimation::AprilLocationUpdate(Affine2d newLocation)
{
	//std::cout << QDateTime::currentDateTime().toString("hh:mm:ss.zzz").toStdString() << "got april update: " << newLocation.translation().transpose() << "\n";
	
	odometry = Odometry();

	Rotation2Dd rotation2D(0);
	rotation2D.fromRotationMatrix(robot->mLastAprilLocation.linear());

	odometry.mPosition = robot->mLastAprilLocation.translation();
	odometry.SetTheta(rotation2D.angle());

	mHistory.clear();
	
}

void LocationEstimation::SendUpdate()
{
	mHistory.append(GetEstimate());

	QByteArray buffer;
	QDataStream stream(&buffer, QIODevice::WriteOnly);

	stream << mHistory;

	QList<QByteArray> msg;
	msg += QByteArray("\x15");
	msg += buffer;
	robot->mSensorLog->mSocket->sendMessage(msg);	
}

//////////////////////////
/// Kratos2
//////////////////////////

Kratos2::Kratos2(QObject* parent) : 
	QObject(parent),
	mLeftWheelVelocity(0.0),
	mRightWheelVelocity(0.0),
	mLocation(this),
	mState(nullptr), 
	mIsPaused(false),
	mLastAprilId(-1),
	mLastAprilDetection(QDateTime::currentDateTime())
{
	mContext = nzmqt::createDefaultContext(this);
	mContext->start();

	mApril2 = new AprilTagCamera(1064*2/3, 1064*2/3, 1280, 720, this);
	mApril2->setObjectName("KinectImage");
}


void Kratos2::Initialize()
{
	mSensorLog = new SensorLog(this, mContext);
	mPlanner = new TrajectoryPlanner2(this);
	//mWheelPID = new WheelPID(this);

	connect(GetKinect(), &Kinect::receiveDepthImage, mSensorLog, &SensorLog::receiveDepthImage, Qt::QueuedConnection);
	connect(GetKinect(), &Kinect::receiveDepthImage, this, &Kratos2::ProccessPointCloud, Qt::QueuedConnection);
	connect(GetKinect(), &Kinect::receiveColorImage, mSensorLog, &SensorLog::receiveKinectImage, Qt::QueuedConnection);
	connect(GetKinect(), &Kinect::receiveColorImage, this, &Kratos2::receiveKinectImage, Qt::QueuedConnection);
	GetKinect()->requestDepthFrame();
	GetKinect()->requestColorFrame();

	connect(GetDecawave(), &Decawave::statusUpdate, mSensorLog, &SensorLog::decawaveUpdate);

	connect(GetTeensy(), &Teensy::statusUpdate, mSensorLog, &SensorLog::teensyStatus);
	connect(GetTeensy(), &Teensy::statusUpdate, this, &Kratos2::TeensyStatus);
	connect(GetTeensy2(), &Teensy2::statusUpdate, mSensorLog, &SensorLog::teensy2Status);

	connect(GetApril(), &AprilTagCamera::ReceiveFrame, mSensorLog, &SensorLog::ReceiveAprilTagImage);
	connect(GetApril(), &AprilTagCamera::tagsDetected, mSensorLog, &SensorLog::ReceiveAprilTags, Qt::QueuedConnection);
	connect(GetApril(), &AprilTagCamera::tagsDetected, this, &Kratos2::AprilTagDetected, Qt::QueuedConnection);


	connect(mApril2, &AprilTagCamera::tagsDetected, mSensorLog, &SensorLog::ReceiveAprilTags2, Qt::QueuedConnection);
	connect(mApril2, &AprilTagCamera::tagsDetected, this, &Kratos2::AprilTag2Detected, Qt::QueuedConnection);

	
	connect(&mFutureWatcher, SIGNAL(finished()), this, SLOT(FinishedPointCloud()));
	connect(this, &Kratos2::WheelVelocityUpdate, mSensorLog, &SensorLog::WheelVelocityUpdate);

	connect(this, &Kratos2::AprilLocationUpdate, &mLocation, &LocationEstimation::AprilLocationUpdate);
	connect(GetTeensy(), &Teensy::statusUpdate, &mLocation, &LocationEstimation::teensyStatus);

	connect(GetSampleDetection(), &SampleDetection::SampleDetected, mSensorLog, &SensorLog::SampleDetected);

	

	auto aprilTimer = new QTimer(this);
	aprilTimer->start(2250);
	QObject::connect(aprilTimer, &QTimer::timeout, this, &Kratos2::AprilScanTimer);

	mState = new RootState(this);
	QTimer::singleShot(2000, this, SLOT(StartStateMachine()));
}

void Kratos2::StartStateMachine()
{
	this->mState->Start();
}

void Kratos2::AprilScanTimer()
{
	static QList<int> scanModeValues = {-90, -45, 0, 45, 90};  // Horizontal FOV is 70 degrees, this gives 25 degree overlap between shots
	static int lastScanValue = 0;

	auto currentTime = QDateTime::currentDateTime();
	if(std::abs(this->mLastAprilDetection.msecsTo(currentTime)) > 4000)
	{
		//std::cout << "Setting april camera to SCAN MODE ("<< scanModeValues[lastScanValue] <<")\n";	

		GetTeensy2()->sendRaw(scanModeValues[lastScanValue]);
		lastScanValue = (lastScanValue + 1) % scanModeValues.size();	
	}

	//auto pool = QThreadPool::globalInstance();
	//std::cout << "Thread pool: " << pool->activeThreadCount() << "/" << pool->maxThreadCount() << "\n";
}

void Kratos2::AprilTag2Detected(QList<AprilTagDetectionItem> detections)
{
	AprilTagDetectionItem tag;
	bool foundSameAsLast = false;

	for(auto& detection : detections)
	{
		if(detection.detection.id == mLastAprilId)
		{
			tag = detection;
			foundSameAsLast = true;
			break;
		}
	}

	if(foundSameAsLast == false)
	{
		if(detections.isEmpty())
			return;

		tag = detections[0];
	}

	const auto tagInfo = GetTagById(tag.detection.id);

	if(tagInfo == nullptr)
	{
		std::cerr << "Unkown tag of id: " << tag.detection.id << "\n";
		return;
	}

	//Update the robot location estimation
	double euler = tag.euler.y() + tagInfo->mOrientation;
	Vector2d translation = tag.translation.head<2>() + Vector2d(0.90, 0);

	translation = Rotation2Dd(euler) * translation;
	translation -= tagInfo->mOffset;

	Affine2d aprilLocation;
	aprilLocation.linear() = Rotation2Dd(M_PI + euler).toRotationMatrix(); //pi - euler = the tag is in front of the robot
	aprilLocation.translation() = translation;

	mLastAprilDetection = QDateTime::fromMSecsSinceEpoch(tag.time);
	mLastAprilLocation = aprilLocation;

	mSensorLog->AprilLocationUpdate(mLastAprilDetection, aprilLocation);

	emit AprilLocationUpdate(aprilLocation);
}

void Kratos2::AprilTagDetected(QList<AprilTagDetectionItem> detections)
{

	//cout << QDateTime::currentDateTime().toString("hh:mm:ss.zzz").toStdString()  << ": Robot found " << detections.size() << " detections\n";
	AprilTagDetectionItem tag;
	bool foundSameAsLast = false;

	for(auto& detection : detections)
	{
		if(detection.detection.id == mLastAprilId)
		{
			tag = detection;
			foundSameAsLast = true;
			break;
		}
	}

	if(foundSameAsLast == false)
	{
		if(detections.isEmpty())
			return;

		tag = detections[0];
	}

	const auto tagInfo = GetTagById(tag.detection.id);

	if(tagInfo == nullptr)
	{
		std::cerr << "Unkown tag of id: " << tag.detection.id << "\n";
		return;
	}

	QSqlQuery query(mSensorLog->mDb);
	query.prepare("SELECT servoAngle FROM teensy2Log WHERE timestamp > :startTime ORDER BY timestamp ASC LIMIT 1");
	query.bindValue(":startTime", tag.time);

	if(!query.exec()){
		std::cout << "error SQL= " <<  query.lastError().text().toStdString() << std::endl;
		return;
	}

	double servoAngle = 0.0;

	if(!query.next())
	{
		std::cout << "Got no data in teensy2Log" << std::endl;
	}
	else
	{
		servoAngle = query.value(0).toDouble();
	}

	//Update the robot location estimation
	double euler = tag.euler.y() + tagInfo->mOrientation;
	Vector2d translation = tag.translation.head<2>();

	translation = Rotation2Dd(euler) * translation;
	translation -= tagInfo->mOffset;

	euler -= servoAngle;

	Affine2d aprilLocation;
	aprilLocation.linear() = Rotation2Dd(euler).toRotationMatrix();
	aprilLocation.translation() = translation;

	mLastAprilDetection = QDateTime::fromMSecsSinceEpoch(tag.time);
	mLastAprilLocation = aprilLocation;

	mSensorLog->AprilLocationUpdate(mLastAprilDetection, aprilLocation);

	emit AprilLocationUpdate(aprilLocation);

	//Update the teensy to update to the new angle
	double rot2 = std::atan2(tag.translation.y(), tag.translation.x());
	if(std::abs(rot2) > (1.0 * M_PI / 180.0))
	{
		GetTeensy2()->setAprilAngle(clamp(rot2 + servoAngle, -M_PI/2, M_PI/2));
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

	if(status.autoFlag == 0)
	{
		//SetWheelVelocity(0.0, 0.0);
	}
}

void Kratos2::SetWheelVelocity(double left, double right)
{
	mLeftWheelVelocity = left;
	mRightWheelVelocity = right;

	//std::cout << "Wheel velocity: " << left << "\t" << right << "\n";

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

void Kratos2::receiveKinectImage(Robot::ImgData mat)
{
	QImage image(mat.data.data(), mat.width, mat.height, QImage::Format_RGB888);
	image = image.mirrored(true, false).scaled(1280, 720, Qt::KeepAspectRatio);

	mApril2->ReadFrame(image);
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

		//std::cout << "Received DepthImgData frame\n";

		auto imgCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		//imgCloud->sensor_origin_ = Eigen::Vector4f(lastPointPos.x(), lastPointPos.y(), lastPointPos.z(), 0.0);
		//imgCloud->sensor_orientation_ = Eigen::AngleAxisf((float) lastPoint.mRotation, Eigen::Vector3f::UnitZ());

		UpdatePointCloud(mat, *imgCloud);

		RegionGrowingSegmenter growingRegion;
		SegmentedPointCloud segmented;
		segmented.mPointCloud = growingRegion.AsyncronousUpdate(imgCloud);
		segmented.mTimestamp = currentTime;

		//std::cout << QDateTime::currentDateTime().toString("hh:mm:ss.zzz").toStdString() << ":Finished to segment\n";

		return segmented;
	});

	mFutureWatcher.setFuture(future);
}

void Kratos2::FinishedPointCloud()
{
	//std::cout << "Finished point cloud;\n";
	auto kinect = GetKinect();
	auto pointCloud = mFutureWatcher.future().result();

	//std::cout << QDateTime::currentDateTime().toString("hh:mm:ss.zzz").toStdString() << ": Got segmented cloud\n";

	if(kinect != nullptr)
	{
		kinect->requestColorFrame();
		kinect->requestDepthFrame();
	}

	if(pointCloud.mPointCloud != nullptr)
	{
		mSensorLog->ReceiveSegmentedPointcloud(pointCloud);
		mPlanner->UpdateObstacles(pointCloud);
	}
}

Odometry Kratos2::GetOdometryTraveledSince(QDateTime startTime, QDateTime endTime, bool useApril)
{
	Odometry odometry(0.69);
	std::size_t count = 0;
	double lastLeft = 0.0;
	double lastRight = 0.0;

	if(useApril && startTime <= mLastAprilDetection && mLastAprilDetection <= endTime)
	{
		startTime = mLastAprilDetection;
		Rotation2Dd rotation2D(0);
		rotation2D.fromRotationMatrix(mLastAprilLocation.linear());

		odometry.mPosition += mLastAprilLocation.translation();
		odometry.SetTheta(rotation2D.angle());
	}


	QSqlQuery query(mSensorLog->mDb);
	query.prepare("SELECT leftWheel, rightWheel FROM teensyLog WHERE timestamp > :startTime AND timestamp < :endTime");
	query.bindValue(":startTime", startTime.toMSecsSinceEpoch());
	query.bindValue(":endTime", endTime.toMSecsSinceEpoch());

	if(!query.exec())
		std::cout << "error SQL= " <<  query.lastError().text().toStdString() << std::endl;

	if(query.next())
	{
		lastLeft = query.value(0).toDouble();
		lastRight = query.value(1).toDouble();
	}

	while (query.next()) {
		double left = query.value(0).toDouble();
		double right = query.value(1).toDouble();

		double diffLeft = static_cast<double>(left - lastLeft) / 23330.0 * 0.31 * M_PI;
		double diffRight = static_cast<double>(right - lastRight) / 23330.0 * 0.31 * M_PI;

		lastLeft = left;
		lastRight = right;

		if(std::abs(diffLeft) > 0.1 || std::abs(diffRight) > 0.1){
			//std::cout << "\t Skipping large jump: " << diffLeft << "\t" << diffRight << "\n";
			continue;
		}

		odometry.Update(diffLeft, diffRight);

		count++;
		if(count >= 10000)
		{
			std::cerr << "Not reading more than 10,000 wheel encoder values for odometry.\n";
			break;
		}
    }

	return odometry;
}

