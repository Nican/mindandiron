#pragma once

#include <QObject>
#include <QVector>
#include <Box2D/Box2D.h>
#include "../odometry.h"
#include "../pointcloud.h"

namespace Robot
{ 

class TrajectoryTreeNode2;
class TrajectoryPlanner2;
class TrajectorySearch;

typedef std::unique_ptr<TrajectoryTreeNode2> TrajectoryTreeNode2Ptr;
	

class TrajectoryTreeNode2
{
public:
	Eigen::Vector2d mPoint;
	Complex mRotation;

	std::list<TrajectoryTreeNode2Ptr> childs;
	std::list<Complex> availableAngles;

	TrajectorySearch* mPlanner;

	TrajectoryTreeNode2(TrajectorySearch* planner, const Eigen::Vector2d &point, Complex rotation);
	//TrajectoryTreeNode(TrajectoryPlanner2* planner, const msgpack::object &o);

	bool explore();

	Complex getNextBestAngle() const;

	inline bool inGoal() const;
};

class TrajectorySearch 
{
public:
	TrajectoryPlanner2* mPlanner;
	Eigen::Vector2d mGoal;

	TrajectorySearch(TrajectoryPlanner2* planner) : mPlanner(planner)
	{
	}

	bool TestPosition(Eigen::Vector2d pos, double rotation);
};

class TrajectoryPlanner2 : public QObject
{
	Q_OBJECT

	b2Fixture* CreateRobot();

public:
	Odometry mOdometry;

	b2World world;

	b2Fixture* mRobotFixture;
	QVector<b2Body*> mObstacles;

	TrajectoryPlanner2(QObject* parent);

	void UpdateObstacles(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);
	
	//void UpdateOdometry(double leftWheel, double rightWheel);

	void AddObstacle(float x, float y);

signals:
	void ObstacleMapUpdate(std::vector<Eigen::Vector2i>);

};



}