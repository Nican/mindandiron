#include "msgpack.h"
#include "gazebo2.h"
#include "teensy.h"

using namespace Robot;

QDataStream &operator<<(QDataStream &out, const ImgData &item)
{
	out << item.width;
	out << item.height;

	out.writeRawData(reinterpret_cast<const char*>(item.data.data()), item.data.size());

	return out;
}

QDataStream &operator>>(QDataStream &in, ImgData &item)
{
	in >> item.width;
	in >> item.height;

	item.data.resize(item.width * item.height * 3);
	in.readRawData(reinterpret_cast<char*>(item.data.data()), item.data.size());

	return in;
}

QDataStream &operator<<(QDataStream &out, const DepthImgData &item)
{
	out << item.width;
	out << item.height;
	out << item.hfov;

	out.writeRawData(reinterpret_cast<const char*>(item.data.data()), item.data.size() * sizeof(float));

	return out;
}

QDataStream &operator>>(QDataStream &in, DepthImgData &item)
{
	in >> item.width;
	in >> item.height;
	in >> item.hfov;

	item.data.resize(item.width * item.height);
	in.readRawData(reinterpret_cast<char*>(item.data.data()), item.data.size() * sizeof(float));

	return in;
}


QDataStream &operator<<(QDataStream &out, const RobotGazeboControl &item)
{
	out << item.leftVelocity;
	out << item.rightVelocity;
	out << item.aprilAngle;

	return out;
}

QDataStream &operator>>(QDataStream &in, RobotGazeboControl &item)
{
	in >> item.leftVelocity;
	in >> item.rightVelocity;
	in >> item.aprilAngle;

	return in;
}

QDataStream &operator<<(QDataStream &out, const RobotGazeboTickData &item)
{
	out << item.leftWheelTicks;
	out << item.rightWheelTicks;
	out << item.leftWheelVelocity;
	out << item.rightWheelVelocity;
	out << item.linearAcceleration.x();
	out << item.linearAcceleration.y();
	out << item.linearAcceleration.z();
	out << item.robotPosition.x();
	out << item.robotPosition.y();
	out << item.robotPosition.z();
	out << item.robotOrientation;
	out << item.aprilAngle;

	return out;
}

QDataStream &operator>>(QDataStream &in, RobotGazeboTickData &item)
{
	in >> item.leftWheelTicks;
	in >> item.rightWheelTicks;
	in >> item.leftWheelVelocity;
	in >> item.rightWheelVelocity;
	in >> item.linearAcceleration.x();
	in >> item.linearAcceleration.y();
	in >> item.linearAcceleration.z();
	in >> item.robotPosition.x();
	in >> item.robotPosition.y();
	in >> item.robotPosition.z();
	in >> item.robotOrientation;
	in >> item.aprilAngle;

	return in;
}

QDataStream &operator<<(QDataStream &out, const Robot::TeenseyStatus &item)
{
	out << item.leftPosition;
	out << item.rightPosition;
	out << item.leftVelocity;
	out << item.rightVelocity;
	out << item.acceleration.x();
	out << item.acceleration.y();
	out << item.acceleration.z();
	out << item.autoFlag;

	return out;
}

QDataStream &operator>>(QDataStream &in, Robot::TeenseyStatus &item)
{
	in >> item.leftPosition;
	in >> item.rightPosition;
	in >> item.leftVelocity;
	in >> item.rightVelocity;
	in >> item.acceleration.x();
	in >> item.acceleration.y();
	in >> item.acceleration.z();
	in >> item.autoFlag;

	return in;
}


QDataStream &operator<<(QDataStream &out, const Robot::Teensy2Status &item)
{
	out << item.servoAngle;
	out << item.current;
	out << item.voltage;
	out << item.isPaused;

	return out;
}

QDataStream &operator>>(QDataStream &in, Robot::Teensy2Status &item)
{
	in >> item.servoAngle;
	in >> item.current;
	in >> item.voltage;
	in >> item.isPaused;

	return in;
}