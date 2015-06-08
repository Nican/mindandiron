#include "state/state.h"
#include <QtConcurrent>

using namespace Robot;
using namespace Eigen;

////////////////////
////	TravelToWayPoint
////////////////////

static double GetEuler(Matrix2d matrix)
{
	Rotation2Dd r(0);
	r.fromRotationMatrix(matrix);
	return r.angle();
}

TravelToWayPoint::TravelToWayPoint(Vector2d position, QObject *parent) : 
	ProgressState(parent)
	,mTargetPosition(position)
	,mReverse(false)
{
}

void TravelToWayPoint::Start()
{
	connect(Robot()->GetTeensy(), &Teensy::statusUpdate, this, &TravelToWayPoint::TeensyStatus);
}

void TravelToWayPoint::TeensyStatus(TeenseyStatus status)
{
	if(IsFinished())
		return;

	const double TURN_STRENGTH = 0.1; 
	const double SIDE_VELOCITY_OFFSET = 0.02;  // meters/second
	const double FORWARD_VELOCITY = 0.12;  // meters/second
	const double ARRIVAL_RADIUS = 0.4 + 0.01 * mTargetPosition.norm();  // meters, gets bigger as goal goes away from base

	auto goal = mTargetPosition;
	auto current = Robot()->mLocation.GetEstimate();
	double currentAngle = GetEuler(current.linear());

	std::cout << "Distance from goal: " << (goal - current.translation()).norm() << "\n";

	if((goal - current.translation()).norm() < ARRIVAL_RADIUS)
	{
		Robot()->SetWheelVelocity(0.0, 0.0);
		SetFinished();
		return;
	}

	Vector2d diff(goal - current.translation());
	double desiredAngle = std::atan2(diff.y(), diff.x());

	//Message to Mali: Complex numbers are way better. 
	double angleDiff = std::arg(std::polar(1.0, desiredAngle) * std::polar(1.0, -currentAngle));

	if(std::abs(angleDiff) > (30 * M_PI / 180.0))
	{
		//How did we get rotated so far apart from our goal?
		if(angleDiff < 0.0)
			Robot()->SetWheelVelocity(TURN_STRENGTH, -TURN_STRENGTH);
		else
			Robot()->SetWheelVelocity(-TURN_STRENGTH, TURN_STRENGTH);

		return;
	}

	if(angleDiff < -0.05)
	{
		if(mReverse)
			Robot()->SetWheelVelocity(-(FORWARD_VELOCITY - SIDE_VELOCITY_OFFSET),
									  -(FORWARD_VELOCITY + SIDE_VELOCITY_OFFSET));
		else
			Robot()->SetWheelVelocity((FORWARD_VELOCITY + SIDE_VELOCITY_OFFSET),
									  (FORWARD_VELOCITY - SIDE_VELOCITY_OFFSET));
	}
	else if(angleDiff > 0.05)
	{
		if(mReverse)
			Robot()->SetWheelVelocity(-(FORWARD_VELOCITY + SIDE_VELOCITY_OFFSET),
									  -(FORWARD_VELOCITY - SIDE_VELOCITY_OFFSET));
		else
			Robot()->SetWheelVelocity((FORWARD_VELOCITY - SIDE_VELOCITY_OFFSET),
									  (FORWARD_VELOCITY + SIDE_VELOCITY_OFFSET));
	} 
	else 
	{
		if(mReverse)
			Robot()->SetWheelVelocity(-FORWARD_VELOCITY, -FORWARD_VELOCITY);
		else
			Robot()->SetWheelVelocity(FORWARD_VELOCITY, FORWARD_VELOCITY);
	}

}