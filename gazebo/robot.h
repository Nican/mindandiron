#pragma once

#include <memory>
#include <eigen3/Eigen/Dense>
#include <zmq.hpp>
#include "msgpack.h"
#include "state.h"
#include "odometry.h"
#include "april.h"

namespace Robot
{

class Wheel
{

public:
	//Returns the number of cycles/sec the wheel is moving in
	virtual double GetRotationVelocity() const = 0;

	virtual void SetForce(double force) = 0;
};

typedef std::shared_ptr<Wheel> WheelPtr;

class TotalRoboticStation
{
public:

	virtual Eigen::Vector3d GetPosition() const = 0; 
	virtual double GetOrientation() const = 0;
};

typedef std::shared_ptr<TotalRoboticStation> TotalRoboticStationPtr;


class AprilTagServo
{
public:

	virtual double GetPosition() const = 0; 
	virtual void SetPosition(double radians) = 0;
};

typedef std::shared_ptr<AprilTagServo> AprilTagServoPtr;


class RobotSensors
{
public:
	//Kinetic? Lidar? Camera?
	TotalRoboticStationPtr mTRS;
};

/**
	Hold the adapters for controlling the wheels
*/
class RobotMotion
{
public:
	WheelPtr mLeftWheel;
	WheelPtr mRightWheel;

	AprilTagServoPtr mAprilServo;
};

class LocationDataPoint
{
public:
	double mTime;
	Eigen::Vector3d mPosition;
	double mRotation;

	MSGPACK_DEFINE(mTime, mPosition, mRotation);
};

struct ImgData
{
  std::vector<unsigned char> data;
  unsigned int width;
  unsigned int height;

  MSGPACK_DEFINE(data, width, height);
};

struct DepthImgData
{
  std::vector<float> data;
  unsigned int width;
  unsigned int height;
  float hfov;

  MSGPACK_DEFINE(data, width, height, hfov);
};

struct DecawavePoint
{
	double mDistance;
	
};

/**
	Keep history of locations
*/
class LocationHistory
{
public:
	std::vector<LocationDataPoint> mPoints;
};

class BaseStationDetector;

/**
	Main class for the robot
*/
class Kratos : public std::enable_shared_from_this<Kratos>
{
public:
	RobotMotion mMotion;
	RobotSensors mSensors;

	LocationHistory mLocationHistory;

	double mStartSimTime;
	double mLastSimTime;

	zmq::context_t mZmqContext;
	zmq::socket_t mZmqSocket;

	std::shared_ptr<State::Base> mState;

	std::shared_ptr<BaseStationDetector> mBaseStation;

	Odometry mOdometry;

	double mCurTime;

	Kratos(const RobotMotion& motion, const RobotSensors& sensors);

	void Update(double simTime);

	void ReceiveDepth(const DepthImgData &depthData);
	void ReceiveKinectImage(const ImgData &imageData);
	void ReceiveAprilImage(const ImgData &imageData);

	void UpdateCameraRotation();

	void ReceiveWheelTicks(int leftTicks, int rightTicks);


	template<typename T> 
	void SendTelemetry(char id, const T obj)
	{
		msgpack::sbuffer sbuf;
		sbuf.write(&id, sizeof(id));
		msgpack::pack(sbuf, obj);

		zmq::message_t msg(sbuf.size());
		memcpy(msg.data(), sbuf.data(), sbuf.size());
		mZmqSocket.send (msg);
	};

};

}