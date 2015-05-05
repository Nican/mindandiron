#pragma once
#include "robot.h"

struct RobotGazeboControl
{
	double leftVelocity;
	double rightVelocity;

	double aprilAngle;

	MSGPACK_DEFINE(leftVelocity, rightVelocity, aprilAngle)
};

struct RobotGazeboTickData
{
	int leftWheelTicks;
	int rightWheelTicks;

	double leftWheelVelocity;
	double rightWheelVelocity;

	Eigen::Vector3d linearAcceleration;

	Eigen::Vector3d robotPosition;
	double robotOrientation;

	double aprilAngle;

	MSGPACK_DEFINE(leftWheelTicks, rightWheelTicks, leftWheelVelocity, rightWheelVelocity, linearAcceleration, robotPosition, robotOrientation, aprilAngle);
};


