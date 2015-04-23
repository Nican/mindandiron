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

	virtual bool IsValid();
	virtual void Start(){};
	virtual void End(){};
};

class ProgressState;

class RootState : public BaseState
{
	Q_OBJECT

	ProgressState *mState;

public:
	RootState(Kratos2 *parent);

	void SetState(ProgressState* nextState);

public slots:
	void MoveToNextState();

};


class ProgressState : public BaseState
{
	Q_OBJECT

private:
	bool mIsFinished;

public:
	ProgressState(Kratos2 *parent) : BaseState(parent), mIsFinished(false)
	{
	}

	void SetFinished();
	bool IsFinished();
	virtual bool IsValid() override;

signals:
	void Finished();
};



class MoveTowardsGoalState : public ProgressState
{
	Q_OBJECT
public:

	MoveTowardsGoalState(Kratos2 *parent) : ProgressState(parent)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
};



class MoveForwardState : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	double mDistance;

	MoveForwardState(Kratos2 *parent, double distance) : ProgressState(parent), mDistance(distance)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
};


class RotateState : public ProgressState
{
	Q_OBJECT
public:

	QDateTime mStartTime;
	double mRotation;

	RotateState(Kratos2 *parent, double rotation) : ProgressState(parent), mRotation(rotation)
	{
	}

	virtual void Start() override;

public slots:
	void TeensyStatus(TeenseyStatus status);
};

}