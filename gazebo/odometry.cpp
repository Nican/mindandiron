#include <cmath>
#include "odometry.h"


//TODO: Inverting wheels to keep axis consistent - Permanent this way?
void Odometry::Update(double rightWheel, double leftWheel)
{
	//http://guiott.com/Rino/dsNavCon/dsNavCon_Odo.htm
	Eigen::Vector2d Dpos;
	double DSpace;

	double SrMinus = leftWheel - rightWheel;
	double SrPlus = leftWheel + rightWheel;

	if(std::fabs(SrMinus) <= 0.003)
	{
		//Traveling near a straight line
		DSpace = SrPlus / 2.0; //Take the average
		Dpos.x() = DSpace * cosPrev;
		Dpos.y() = DSpace * sinPrev;
	}
	else if(std::fabs(SrPlus) <= 0.003)
	{
		//Pivoting around, without translation
		double DTheta = SrMinus / mAxle;
		mTheta = std::fmod(mTheta + DTheta, M_PI * 2);//current orientation in 2PI range
		cosPrev = std::cos(mTheta); // for the next cycle
		sinPrev = std::sin(mTheta);
		Dpos.x() = 0;
		Dpos.y() = 0;
		DSpace = 0;
	}
	else
	{
		// rounding a curve  

		double DTheta = SrMinus / mAxle;
		mTheta = std::fmod(mTheta + DTheta, M_PI * 2);//current orientation in 2PI range
		double CosNow = std::cos(mTheta);
		double SinNow = std::sin(mTheta);

		DSpace = SrPlus / 2.0;
		double Radius = (mAxle / 2.0) * (SrPlus / SrMinus);

		Dpos.x() = Radius * (SinNow - sinPrev);
		Dpos.y() = Radius * (cosPrev - CosNow);

		cosPrev = CosNow;// to avoid re-calculation on the next cycle
		sinPrev = SinNow;
	}

	mDistanceTraveled += DSpace;   // total traveled distance
    mPosition += Dpos;     // current position
}

void Odometry::SetTheta(double angle)
{
	mTheta = angle;
	cosPrev = std::cos(mTheta);
	sinPrev = std::sin(mTheta);
}

std::ostream& operator<<(std::ostream &strm, const Odometry &a) {
 	return strm << "Odometry({" << a.mPosition.transpose() << "}, " << (a.mTheta * 180.0 / M_PI) << "°)";
}