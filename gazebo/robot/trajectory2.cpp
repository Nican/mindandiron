#include "trajectory2.h"


using namespace Robot;


TrajectoryPlanner2::TrajectoryPlanner2(QObject* parent) : 
QObject(parent), mOdometry(0.69), world(b2Vec2(0.0, 0.0))
{
	world.SetAllowSleeping(false);

	mRobotFixture = CreateRobot();
}

b2Fixture* TrajectoryPlanner2::CreateRobot()
{
	b2BodyDef robotBodyDef;
	robotBodyDef.position.Set(0.0f, 0.0f);
	robotBodyDef.type = b2_dynamicBody;

	b2Body* robotBody = world.CreateBody(&robotBodyDef);

	b2PolygonShape robotShape;
	static std::vector<b2Vec2> robotPoints = GetRobotPoints<b2Vec2>();

	robotShape.Set(robotPoints.data(), robotPoints.size());

	b2FixtureDef spriteShapeDef;
    spriteShapeDef.shape = &robotShape;
    spriteShapeDef.density = 0.0f;
    spriteShapeDef.isSensor = true;
	return robotBody->CreateFixture(&spriteShapeDef);
}


void TrajectoryPlanner2::AddObstacle(float x, float y)
{
	b2BodyDef obstacleBodyDef;
    obstacleBodyDef.position.Set(x, y);
    obstacleBodyDef.type = b2_dynamicBody;

    b2Body* obstacleBody = world.CreateBody(&obstacleBodyDef);

    b2PolygonShape groundBox;
    groundBox.SetAsBox(5.0 / 512.0, 5.0 / 512.0);

    b2FixtureDef spriteShapeDef2;
    spriteShapeDef2.shape = &groundBox;
    spriteShapeDef2.density = 0.0f;
    spriteShapeDef2.isSensor = true;
    obstacleBody->CreateFixture(&spriteShapeDef2);    

    mObstacles.append(obstacleBody);
}

void TrajectoryPlanner2::UpdateObstacles(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
	//We want to transform the points from x=-2.5...2.5 and z=0...5 
    //To a 2d image, 512x512 
	Eigen::MatrixXi walkabilityMap = Eigen::MatrixXi::Zero(512, 512);
	Eigen::Affine2f toImageTransform = Eigen::Scaling(512.0f / 5.0f, 512.0f / 5.0f) * Eigen::Translation2f(2.5f, 0.0f);
	std::vector<Eigen::Vector2i> obstaclePoints;

	for(const auto& pt : pointCloud->points)
	{
    	//Green points are good to move through
		if(pt.g == 255)
			continue;

		auto pt2d = Eigen::Vector2f(pt.x, pt.z);
		Eigen::Vector2i imagePt = (toImageTransform * pt2d).cast<int>();

		if(imagePt.x() < 0 || imagePt.x() > walkabilityMap.rows())
			continue;

		if(imagePt.y() < 0 || imagePt.y() > walkabilityMap.cols())
			continue;

		if(walkabilityMap(imagePt.x(), imagePt.y()) != 0)
			continue;

        //1 = bad point
        //2 = not grouped in any cluster
		walkabilityMap(imagePt.x(), imagePt.y()) = pt.b == 255 ? 2 : 1;


		if(pt.r == 255)
		{
			obstaclePoints.push_back(imagePt);
		}
        /*
        Eigen::Vector2f newPt(imagePt.y() * 5.0 / 512.0 + 0.6, imagePt.x() * 5.0 / 512.0 - 2.5);

        auto rect = new QGraphicsRectItem(newPt.x(), newPt.y(), 5.0 / 512.0, 5.0 / 512.0);
        rect->setPen(QPen(Qt::red, 0));
        mCore->addToGroup(rect);
        */
    }

    emit ObstacleMapUpdate(obstaclePoints);

    for(auto& obstacle : mObstacles)
	{
		world.DestroyBody(obstacle);
	}
	mObstacles.clear();

	for(const Eigen::Vector2i& pt : obstaclePoints)
	{
		AddObstacle(pt.y() * 5.0f / 512.0f + 0.6f, pt.x() * 5.0f / 512.0f - 2.5f);
	}
}

void TrajectoryPlanner2::UpdateOdometry(double leftWheel, double rightWheel)
{

}
