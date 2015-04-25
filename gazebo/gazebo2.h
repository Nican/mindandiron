#pragma once
#include "robot.h"

struct RobotGazeboControl
{
	double leftVelocity;
	double rightVelocity;

	MSGPACK_DEFINE(leftVelocity, rightVelocity)
};

struct RobotGazeboTickData
{
	double simTime;
	int leftWheelTicks;
	int rightWheelTicks;

	double leftWheelVelocity;
	double rightWheelVelocity;

	Eigen::Vector3d linearAcceleration;

	Eigen::Vector3d robotPosition;
	double robotOrientation;

	MSGPACK_DEFINE(simTime, leftWheelTicks, rightWheelTicks, leftWheelVelocity, rightWheelVelocity, linearAcceleration, robotPosition, robotOrientation);
};


