#pragma once

#include <memory>
#include <eigen3/Eigen/Dense>

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
};

typedef std::shared_ptr<TotalRoboticStation> TotalRoboticStationPtr;


class RobotSensors
{
public:
	//Kinetic? Lidar? Camera?
	TotalRoboticStationPtr mTRS;
};

class RobotMotion
{
public:
	WheelPtr mLeftWheel;
	WheelPtr mRightWheel;
};


class Kratos
{
	RobotMotion mMotion;
	RobotSensors mSensors;

public:

	Kratos(const RobotMotion& motion, const RobotSensors& sensors);

	void Update(double simTime);

};

}