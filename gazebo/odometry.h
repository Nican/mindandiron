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

	Odometry(double axleDistance) : mAxle(axleDistance), cosPrev(1.0), sinPrev(0.0)
	{
	}

	void Update(double leftWheel, double rightWheel);

};
