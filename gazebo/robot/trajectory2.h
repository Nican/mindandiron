#pragma once

#include <QObject>
#include <QVector>
#include <Box2D/Box2D.h>
#include <QDateTime>
#include "../odometry.h"
#include "../pointcloud.h"
#include "../msgpack.h"

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

class TrajectorySearch : public QObject
{
	Q_OBJECT
	b2Fixture* CreateRobot();

public:
	b2World world;
	b2Fixture* mRobotFixture;
	QVector<b2Body*> mObstacles;

	TrajectoryPlanner2* mPlanner;
	Eigen::Vector2d mGoal;
	std::unique_ptr<TrajectoryTreeNode2> rootNode;

	QDateTime mCreatedTime;

	bool foundSolution;

	TrajectorySearch(const std::vector<Eigen::Vector2i> &obstacleList, Eigen::Vector2d goal, QObject* parent = nullptr);

	bool TestPosition(Eigen::Vector2d pos, double rotation);
	void AddObstacle(float x, float y);

	bool GetResult(std::vector<Eigen::Vector2d> &points);
	void Iterate();
};

class TrajectoryPlanner2 : public QObject
{
	Q_OBJECT

public:
	std::vector<Eigen::Vector2i> mObstacleList;
	
	TrajectoryPlanner2(QObject* parent);

	void UpdateObstacles(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);
	
	//void UpdateOdometry(double leftWheel, double rightWheel);

	//std::shared_ptr<TrajectorySearch> FindPath(const Eigen::Vector2d &goal, int iterations = 5000);

signals:
	void ObstacleMapUpdate(std::vector<Eigen::Vector2i>);

};



}