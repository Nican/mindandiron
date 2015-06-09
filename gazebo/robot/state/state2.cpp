#include "state/state.h"
#include <QtConcurrent>

using namespace Robot;
using namespace Eigen;

////////////////////
////	TravelToWayPoint
////////////////////

double GetEuler(Matrix2d matrix)
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
	const double SIDE_VELOCITY_OFFSET = 0.03;  // meters/second
	const double FORWARD_VELOCITY = 0.5;  // meters/second
	const double ARRIVAL_RADIUS = 0.4 + 0.01 * mTargetPosition.norm();  // meters, gets bigger as goal goes away from base

	auto goal = mTargetPosition;
	auto current = Robot()->mLocation.GetEstimate();
	double currentAngle = GetEuler(current.linear());

	//std::cout << "Distance from goal: " << (goal - current.translation()).norm() << "\n";

	if((goal - current.translation()).norm() < ARRIVAL_RADIUS)
	{
		std::cout << "Arrived at destination. :)\n";
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





////////////////////
////	DecawaveMoveRadialState
////	Move forward while attempting to minimize the distance between odometry and decawave distance traveled
////////////////////

void DecawaveMoveRadialState::Start()
{
	mLastPerformance = 0.0;
	returnType = ReturnMoveEnum::FORWARD;
	Robot()->SetWheelVelocity(0.2, 0.2);

	connect(Robot()->GetDecawave(), &Decawave::statusUpdate, this, &DecawaveMoveRadialState::DecawaveUpdate);
}

void DecawaveMoveRadialState::DecawaveUpdate(double value)
{
	mLastReadings[lastReadingId] = value;

	if(lastReadingId == mLastReadings.size() - 1)
	{
		UpdateDirection(GetAverageDecawaveVelocity(), in);
		mLastReadings[0] = mLastReadings[lastReadingId];
		lastReadingId = 1;
	}
	else
	{
		lastReadingId++;
	}

	// if(value < 10.0){
	// 	Robot()->SetWheelVelocity(0.0, 0.0);
	// 	SetFinished();
	// 	return;
	// }
}

// Should return average velocity in m/s
double DecawaveMoveRadialState::GetAverageDecawaveVelocity()
{
	double averageValue = 0.0;
	for(std::size_t i = 1; i < mLastReadings.size(); i++)
	{
		double diff = mLastReadings[i-1] - mLastReadings[i];
		// Decawave updates at about 0.8 seconds
		averageValue += diff / 0.8;  // THIS IS MORE ACCURATE THAN MEASURING TIME DIFF
									 // There's some weird data buffering going on
	}
	return averageValue / (double) mLastReadings.size();
}

void DecawaveMoveRadialState::UpdateDirection(double averageVelocity, int in)
{
	// Note to Henrique: if any velocities are positive/negative when they
	// shouldn't be, change state to realign?

	static double lastVelocity = 0.0;  // Stores last average velocity for comparison
	const double MAX_SIDE_VELOCITY = 0.075;
	const double MAX_FORWARD_VELOCITY = 0.57;
	static double maxSeenVelocity = MAX_FORWARD_VELOCITY;

	if ((!in && averageVelocity < maxSeenVelocity) ||
	    (in  && averageVelocity > maxSeenVelocity))
	{
			maxSeenVelocity = averageVelocity;
	}
	std::cout << "Average m/s value is: " << averageVelocity << "\n";
	std::cout << "Last m/s value is: " << lastVelocity << "\n";

	double forwardVelocity = MAX_FORWARD_VELOCITY;

	// The "get average velocity" function overestimates a little, or the
	// Teensy overdoes the velocity. Hard to know. Test whether the multiplier
	// (currently 1.0) works on grass, it's good on concrete
	if ((!in && averageVelocity <= (-1.0 * MAX_FORWARD_VELOCITY)) ||
		(in  && averageVelocity >= ( 1.0 * MAX_FORWARD_VELOCITY)))
	{
		std::cout << "Doing well, moving FORWARD\n";
		Robot()->SetWheelVelocity(forwardVelocity, forwardVelocity);
		lastVelocity = averageVelocity;  // For comparison to next computed velocity
		return;
	}

	if ((!in && averageVelocity <= lastVelocity) ||
		(in  && averageVelocity >= lastVelocity))
	{
		std::cout << "State improved from last run, continuing on without DIRECTION change\n";
	} else if (returnType == ReturnMoveEnum::RIGHT) {
		std::cout << "Moving LEFT\n";
		returnType = ReturnMoveEnum::LEFT;
	} else {
		std::cout << "Moving RIGHT\n";
		returnType = ReturnMoveEnum::RIGHT;
	}

	double proportionalReaction = (1.15 - averageVelocity / maxSeenVelocity) * MAX_SIDE_VELOCITY;
	// std::cout << "forwardVelocity: " << forwardVelocity << "\n";
	// std::cout << "proportionalReaction: " << proportionalReaction << "\n";
	CommandVelocity(forwardVelocity, proportionalReaction);
	lastVelocity = averageVelocity;  // For comparison to next computed velocity
}


void DecawaveMoveRadialState::CommandVelocity(double forwardVelocity, double proportionalReaction)
{
	switch(returnType) {
		case ReturnMoveEnum::FORWARD:
			Robot()->SetWheelVelocity(forwardVelocity, forwardVelocity);
			break;
		case ReturnMoveEnum::RIGHT: 
			Robot()->SetWheelVelocity(forwardVelocity + proportionalReaction,
									  forwardVelocity - proportionalReaction);
			break;
		case ReturnMoveEnum::LEFT: 
			Robot()->SetWheelVelocity(forwardVelocity - proportionalReaction,
									  forwardVelocity + proportionalReaction);
			break;
	}
}


void DecawaveMoveTangentialState::UpdateDirection(double averageVelocity, int in)
{
	static int guessIsLeftIn = 0;  // Guesses that "in" is to the left of the robot
	static double lastVelocity = 0.0;  // Stores last average velocity for comparison
	const double MAX_SIDE_VELOCITY = 0.075;
	const double MAX_FORWARD_VELOCITY = 0.35;

	double forwardVelocity = MAX_FORWARD_VELOCITY;

	const double guessVelocityCutoff = 0.2 * MAX_FORWARD_VELOCITY;
	if ((returnType == ReturnMoveEnum::LEFT  && (averageVelocity > (lastVelocity + guessVelocityCutoff))) ||
	    (returnType == ReturnMoveEnum::RIGHT && (averageVelocity < (lastVelocity - guessVelocityCutoff))))
	{
		// std::cout << "Making guess higher\n";
		if (guessIsLeftIn < 2)
			guessIsLeftIn++;
	}
	if ((returnType == ReturnMoveEnum::LEFT  && (averageVelocity < (lastVelocity - guessVelocityCutoff))) ||
	    (returnType == ReturnMoveEnum::RIGHT && (averageVelocity > (lastVelocity + guessVelocityCutoff))))
	{
		// std::cout << "Making guess lower\n";
		if (guessIsLeftIn > -2)
			guessIsLeftIn--;
	}
	// std::cout << "returnType: " << (int) returnType << "\n";
	// std::cout << "Average m/s value is: " << averageVelocity << "\n";
	// std::cout << "Last m/s value is: " << lastVelocity << "\n";
	std::cout << "guessIsLeftIn: " << guessIsLeftIn << "\n";


	if (abs(averageVelocity) <= (0.15 * MAX_FORWARD_VELOCITY))
	{
		std::cout << "\tDoing well, moving FORWARD: " << forwardVelocity << " m/s\n";
		Robot()->SetWheelVelocity(forwardVelocity, forwardVelocity);
		lastVelocity = averageVelocity;  // For comparison to next computed velocity
		return;
	}

	if (((averageVelocity < 0 && lastVelocity < 0) ||
		 (averageVelocity > 0 && lastVelocity > 0)) &&
		abs(averageVelocity) < abs(lastVelocity))
	{
		std::cout << "State improved from last run, continuing on without DIRECTION change\n";
	} else if ((averageVelocity < 0 && guessIsLeftIn >= 0) ||
		   	   (averageVelocity > 0 && guessIsLeftIn < 0)) {
		std::cout << "\tMoving LEFT\n";
		returnType = ReturnMoveEnum::LEFT;
	} else {
		std::cout << "\tMoving RIGHT\n";
		returnType = ReturnMoveEnum::RIGHT;
	}

	double proportionalReaction = abs(averageVelocity) / MAX_FORWARD_VELOCITY * MAX_SIDE_VELOCITY;
	// std::cout << "forwardVelocity: " << forwardVelocity << "\n";
	// std::cout << "proportionalReaction: " << proportionalReaction << "\n";
	CommandVelocity(forwardVelocity, proportionalReaction);
	lastVelocity = averageVelocity;  // For comparison to next computed velocity
}