#include "robot.h"

#include <iostream>
#include <algorithm>
#include <stdlib.h>
#include "trajectory.h"
#include "util.h"

using namespace Robot;



Kratos::Kratos(const RobotMotion& motion, const RobotSensors& sensors) : 	
	mMotion(motion), 
	mSensors(sensors), 
	mStartSimTime(0.0), 
	mZmqContext(1), 
	mZmqSocket(mZmqContext, ZMQ_PUB),
	mOdometry(0.69),
	mCurTime(0.0)
{
	mZmqSocket.bind ("tcp://*:5555");

	//mState.reset(new State::MoveToWaypoint(this));
	//mState.reset(new State::ReturnToBase(this));
	mState.reset(new State::Explore(this));
	mState->Initialize();

	mBaseStation.reset(new BaseStationDetector(this));
	
}

void Kratos::Update(double simTime)
{

	if(mStartSimTime == 0.0)
	{
		std::cout << "First time: " << simTime << "\n";
		mLastSimTime = simTime;
		mStartSimTime = simTime;

		//Just initialization, we skip the first loop to get the time step correctly
		return;
	}

	Eigen::Vector3d pos = mSensors.mTRS->GetPosition();
	double angle = mSensors.mTRS->GetOrientation();
	LocationDataPoint historyPoint{simTime, pos, angle};
	mLocationHistory.mPoints.push_back(historyPoint);

	SendTelemetry(0, historyPoint);

	mCurTime = simTime - mStartSimTime;
	UpdateCameraRotation();
	mState->Think();

	mLastSimTime = simTime;
}

void Kratos::UpdateCameraRotation()
{
	Eigen::Vector3d pos = mSensors.mTRS->GetPosition();
	double rotation = mSensors.mTRS->GetOrientation();

	//Update the april tag camera
	//For now {-1.4, 0} is about where the tags are
	Eigen::Vector2d target = Eigen::Vector2d(-1.0, 0) - pos.head<2>();
	double angle2 = M_PI; 

	//Avoid singularities
	if(target.norm() > 0.01)
		angle2 = std::atan2(target.y(), target.x());

	//std::cout << "A: " << angle2 << "\t" << rotation << "\n";

	mMotion.mAprilServo->SetPosition(angle2 - rotation + M_PI/2);
}

void Kratos::ReceiveWheelTicks(int leftTicks, int rightTicks)
{
	double leftRevolutions = static_cast<double>(leftTicks) / 23330.0;
	double rightRevolutions = static_cast<double>(rightTicks) / 23330.0;

	double leftDistance = leftRevolutions * 0.31 * M_PI;
	double rightDistance = rightRevolutions * 0.31 * M_PI;

	mOdometry.Update(leftDistance, rightDistance);

	// Eigen::Vector2d pos = mSensors.mTRS->GetPosition().head<2>();
	// Eigen::Vector2d diff = mOdometry.mPosition - pos;

	// std::cout << "Measured Position:\t" << mOdometry.mPosition.transpose() << "\n";
	// std::cout << "Error:\t" << diff.transpose() << "\n";
}

void Kratos::ReceiveDepth(const DepthImgData &depthData)
{
	SendTelemetry(3, depthData);
}

void Kratos::ReceiveKinectImage(const ImgData &imageData)
{
	SendTelemetry(2, imageData);
}

void Kratos::ReceiveAprilImage(const ImgData &imageData)
{
	SendTelemetry(4, imageData);

	//Work around for the const cast
	auto copiedData = imageData.data;

	cv::Mat mat(
		imageData.height, 
		imageData.width, 
		CV_8UC3, 
		copiedData.data(), 
		imageData.data.size() / imageData.height);

	//mBaseStation->Update(mat.clone());
}
