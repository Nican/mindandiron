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

	double leftWheelForce;
	double rightWheelForce;

	Eigen::Vector3d linearAcceleration;

	Eigen::Vector3d robotPosition;
	double robotOrientation;

	MSGPACK_DEFINE(simTime, leftWheelTicks, rightWheelTicks, leftWheelForce, rightWheelForce, linearAcceleration, robotPosition, robotOrientation);
};


