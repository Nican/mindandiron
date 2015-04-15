#pragma once

#include <QObject>
#include <QVector>
#include <Box2D/Box2D.h>
#include "../odometry.h"
#include "../pointcloud.h"

namespace Robot
{ 

	

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
	void UpdateOdometry(double leftWheel, double rightWheel);

	void AddObstacle(float x, float y);

signals:
	void ObstacleMapUpdate(std::vector<Eigen::Vector2i>);

};



}