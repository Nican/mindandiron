#pragma once

#include "robot.h"
#include <QDateTime>

namespace Robot
{

class BaseState : public QObject
{
	Q_OBJECT
public:

	Kratos2* mRobot;

	BaseState(Kratos2 *parent) : QObject(parent), mRobot(parent)
	{

	}

	virtual void Start() = 0;

	bool IsActive();
};


class MoveForwardState : public BaseState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	double mDistance;

	MoveForwardState(Kratos2 *parent, double distance) : BaseState(parent), mDistance(distance)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
};


class RotateState : public BaseState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	double mRotation;

	RotateState(Kratos2 *parent, double rotation) : BaseState(parent), mRotation(rotation)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
};

}