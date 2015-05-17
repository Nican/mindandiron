#pragma once

#include <eigen3/Eigen/Dense>

class Odometry 
{
	//Distance between axle and the center
	double mAxle;
	double cosPrev;
	double sinPrev;

public:

	//Total distance traveled
	double mDistanceTraveled;  
	
	Eigen::Vector2d mPosition;
	double mTheta;

	Odometry(double axleDistance = 0.69) : 
		mAxle(axleDistance), 
		cosPrev(1.0), 
		sinPrev(0.0), 
		mDistanceTraveled(0.0), 
		mPosition({0.0, 0.0}), 
		mTheta(0.0)
	{
	}

	void Update(double leftWheel, double rightWheel);
};

std::ostream& operator<<(std::ostream &strm, const Odometry &a); 