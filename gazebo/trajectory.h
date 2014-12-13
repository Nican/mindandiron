#pragma once

#include <memory>
#include <list>
#include <complex>
#include <Box2D/Box2D.h>
#include <eigen3/Eigen/Dense>

class TrajectoryTreeNode;
class TrajectoryPlanner;

typedef std::complex<double> Complex;
typedef std::unique_ptr<TrajectoryTreeNode> TrajectoryTreeNodePtr;


template<class T>
std::vector<T> GetRobotPoints()
{
	return std::vector<T>({
		{0.130-0.33, -0.447675},
		{0.5366-0.33, -0.447675},
		{1.25095-0.33, -0.1383},
		{1.25095-0.33, 0.1383},
		{0.5366-0.33, 0.447675},
		{0.1302-0.33, 0.447675},
		{0.0-0.33, 0.2286},
		{0.0-0.33, -0.2286}
	});	
};

class TrajectoryTreeNode
{
public:
	const Eigen::Vector2d mPoint;
	const Complex mRotation;

	std::list<TrajectoryTreeNodePtr> childs;
	std::list<Complex> availableAngles;

	TrajectoryPlanner* mPlanner;

	TrajectoryTreeNode(TrajectoryPlanner* planner, const Eigen::Vector2d &point, Complex rotation);

	void explore();

	Complex getNextBestAngle() const;

	inline bool inGoal() const;
};



class TrajectoryPlanner
{
	b2World world;
	b2Fixture* robotFixture;
	b2Fixture* obstacleFixture;

public:
	std::unique_ptr<TrajectoryTreeNode> rootNode;

	const Eigen::Vector2d mGoal;

	TrajectoryPlanner(const Eigen::Vector2d &point, Complex rotation, const Eigen::Vector2d &goal);

	bool testPosition(Eigen::Vector2d pos, double rotation);

	void run(int iterations);

	std::vector<TrajectoryTreeNode*> getResult() const;
};