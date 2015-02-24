#pragma once
#include <trajectory.h>

namespace Robot {

class Kratos;

namespace State {

class Base
{
protected:
	/// mRobot should as long as this state exists.
	Kratos* mRobot;

public:
	Base(Kratos* robot) : mRobot(robot)
	{
	}

	virtual void Initialize() = 0;
	virtual void Think() = 0;

	virtual std::shared_ptr<msgpack::sbuffer> GetTelemetry(){};
};


class MoveForward : public Base
{
public:
	MoveForward(Kratos* robot) : Base(robot)
	{
	}

	virtual void Initialize() override;
	virtual void Think() override;
};

enum class ReturnToBaseStage
{
	PositionAfar,
	Alignmenet, 
	BackingUp
};

class ReturnToBase : public Base
{
	double mStateStartTime;

	ReturnToBaseStage mStage;

public:
	ReturnToBase(Kratos* robot) : Base(robot), mStage(ReturnToBaseStage::PositionAfar)
	{
	}

	virtual void Initialize() override;
	virtual void Think() override;
};




class MoveToWaypointTelemetry
{
public:
	int mCurrentPoint;
	std::vector<Eigen::Vector2d> mWaypoints;

	MSGPACK_DEFINE(mCurrentPoint, mWaypoints);
};

class MoveToWaypoint : public Base
{
protected:
	std::shared_ptr<TrajectoryPlanner> mPlanner;
	std::vector<TrajectoryTreeNode*> mPath;
	int mCurrentPoint;

	double lastReplan;

public:
	MoveToWaypoint(Kratos* robot) : Base(robot)
	{
		this->Replan();
	}

	virtual void Initialize() override;
	virtual void Think() override;

	virtual std::shared_ptr<msgpack::sbuffer> GetTelemetry() override;

	virtual void Replan();
};

} //namespace State
} //namespace Robot



