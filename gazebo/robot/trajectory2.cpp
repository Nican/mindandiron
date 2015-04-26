#include "trajectory2.h"
#include "../trajectory.h"
#include "robot.h"

using namespace Robot;


///////////////////////////////
//// TrajectoryTreeNode2
///////////////////////////////

TrajectoryTreeNode2::TrajectoryTreeNode2(TrajectorySearch* planner, const Eigen::Vector2d &point, Complex rotation)
	: mPoint(point), mRotation(rotation), mPlanner(planner)
{
	double minAngle = -M_PI/4;
	double maxAngle = -minAngle;
	int steps = 10;

	for(int i = 0; i < steps; i++)
	{
		double angle = minAngle + ((maxAngle - minAngle) / steps) * i;
		availableAngles.emplace_back(rotationToCompex(angle) * mRotation);
	}


	for(double i = -M_PI/8; i < M_PI/8; i += M_PI/32)
	{
		//availableAngles.emplace_back(rotationToCompex(i) * mRotation);
	}
}

bool TrajectoryTreeNode2::explore()
{
	if(inGoal()){
		mPlanner->foundSolution = true;
		return true;
	}

	//std::cout << "Exploring child at " << mPoint.transpose() << "\n";

	if(!childs.empty())
	{
		for(auto& child : childs)
		{
			if(child->explore())
				return true;
		}
	}


	while(!availableAngles.empty())
	{
		Complex bestAngle(getNextBestAngle());

		availableAngles.remove(bestAngle);

		auto newPoint = mPoint + Eigen::Vector2d(std::real(bestAngle), std::imag(bestAngle)) * 0.3;
		auto node = std::unique_ptr<TrajectoryTreeNode2>(new TrajectoryTreeNode2(mPlanner, newPoint, bestAngle));
		//double x = node->mPoint.x();
		//double y = node->mPoint.y();

		if(mPlanner->TestPosition(node->mPoint, std::arg(node->mRotation)))
		{
			node->availableAngles.clear();
			childs.emplace_back(std::move(node));

			continue;
		}
		
		childs.emplace_back(std::move(node));
		return true;
	}

	return false;
}

Complex TrajectoryTreeNode2::getNextBestAngle() const
{
	Complex bestAngle;
	Eigen::Vector2d diff(mPlanner->mGoal - mPoint);
	Complex targetAngle = rotationToCompex(std::atan2(diff.y(), diff.x()));

	for(auto angle : availableAngles)
	{
		if(std::norm(bestAngle) == 0.0 || std::norm(targetAngle - angle) < std::norm(targetAngle - bestAngle))
		{
			bestAngle = angle;
		}
	}

	return bestAngle;
}

inline bool TrajectoryTreeNode2::inGoal() const
{
	return (mPoint - mPlanner->mGoal).norm() < 0.3;
}


///////////////////////////////
//// TrajectorySearch
///////////////////////////////
TrajectorySearch::TrajectorySearch(const std::vector<Eigen::Vector2d> &obstacleList, Eigen::Vector2d goal, QObject* parent) : 
	QObject(parent), world(b2Vec2(0.0, 0.0)), mGoal(goal), foundSolution(false)
{
	world.SetAllowSleeping(false);

	mRobotFixture = CreateRobot();

	for(const auto& pt : obstacleList)
	{
		AddObstacle(pt.x(), pt.y());
	}

	rootNode.reset(new TrajectoryTreeNode2(this, {0.0, 0.0}, rotationToCompex(0.0)));

	mCreatedTime = QDateTime::currentDateTime();
}

b2Fixture* TrajectorySearch::CreateRobot()
{
	b2BodyDef robotBodyDef;
	robotBodyDef.position.Set(0.0f, 0.0f);
	robotBodyDef.type = b2_dynamicBody;

	b2Body* robotBody = world.CreateBody(&robotBodyDef);

	b2PolygonShape robotShape;
	static std::vector<b2Vec2> robotPoints = GetRobotPoints<b2Vec2>();

	// for(auto& pt : robotPoints)
	// 	pt *= 1.5;

	robotShape.Set(robotPoints.data(), robotPoints.size());

	b2FixtureDef spriteShapeDef;
    spriteShapeDef.shape = &robotShape;
    spriteShapeDef.density = 0.0f;
    spriteShapeDef.isSensor = true;
	return robotBody->CreateFixture(&spriteShapeDef);
}

bool TrajectorySearch::TestPosition(Eigen::Vector2d pos, double rotation)
{
	mRobotFixture->GetBody()->SetTransform({(float) pos.x(), (float) pos.y()}, (float) rotation);

	//Recalculate collissions
	world.Step(1.0f / 60.0f, 1, 1);

	return mRobotFixture->GetBody()->GetContactList() != nullptr;
}

void TrajectorySearch::AddObstacle(float x, float y)
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


static bool GetPath(TrajectoryTreeNode2* node, std::vector<TrajectoryTreeNode2*> &outPath)
{
	if(node->inGoal())
	{
		outPath.push_back(node);
		return true;
	}

	for(auto& child : node->childs)
	{
		if(GetPath(child.get(), outPath))
		{
			outPath.push_back(node);
			return true;
		}
	}

	return false;
}

bool TrajectorySearch::GetResult(std::vector<Eigen::Vector2d> &points)
{
	std::vector<TrajectoryTreeNode2*> outVector;

	if(GetPath(rootNode.get(), outVector))
	{
		for(auto &node : outVector)
		{
			points.push_back(node->mPoint);
		}
		return true;
	}

	return false;
}

void TrajectorySearch::Iterate()
{
	rootNode->explore();
}

///////////////////////////////
//// TrajectoryPlanner2
///////////////////////////////

TrajectoryPlanner2::TrajectoryPlanner2(Kratos2* parent) : 
QObject(parent), mRobot(parent) //, mOdometry(0.69), world(b2Vec2(0.0, 0.0))
{
}

std::size_t TrajectoryPlanner2::GetOldestObstacle()
{
	std::size_t oldest = 0;

	for(std::size_t i = 0; i < mObstacleHistory.size(); i++)
	{
		if(!mObstacleHistory[i].mCreatedTime.isValid())
			return i;

		if(mObstacleHistory[oldest].mCreatedTime < mObstacleHistory[i].mCreatedTime)
			oldest = i;
	}

	return oldest;
}

void TrajectoryPlanner2::UpdateObstacles(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
	using namespace Eigen;
	//We want to transform the points from x=-2.5...2.5 and z=0...5 
    //To a 2d image, 512x512 
	Eigen::MatrixXi walkabilityMap = Eigen::MatrixXi::Zero(512, 512);
	Eigen::Affine2f toImageTransform = Eigen::Scaling(512.0f / 5.0f, 512.0f / 5.0f) * Eigen::Translation2f(2.5f, 0.0f);
	//std::vector<Eigen::Vector2i> obstaclePoints;
	//mObstacleList.clear();
	std::vector<Eigen::Vector2d> obstacleList;

	for(const auto& pt : pointCloud->points)
	{
    	//Green points are good to move through
		if(pt.g == 255)
			continue;

		auto pt2d = Eigen::Vector2f(pt.x, pt.z);
		Eigen::Vector2i imagePt = (toImageTransform * pt2d).cast<int>();

		if(imagePt.x() < 0 || imagePt.x() >= walkabilityMap.rows())
			continue;

		if(imagePt.y() < 0 || imagePt.y() >= walkabilityMap.cols())
			continue;

		if(walkabilityMap(imagePt.x(), imagePt.y()) != 0)
			continue;

        //1 = bad point
        //2 = not grouped in any cluster
		walkabilityMap(imagePt.x(), imagePt.y()) = pt.b == 255 ? 2 : 1;


		if(pt.r == 255)
		{
			Eigen::Vector2d newPt(imagePt.y() * 5.0f / 512.0f + 0.6f, imagePt.x() * 5.0f / 512.0f - 2.5f);
			obstacleList.push_back(newPt);
		}
    }

    auto oldest = GetOldestObstacle();
    auto currentTime = QDateTime::currentDateTime();
    auto validRange = currentTime.addSecs(-25);

    mObstacleHistory[oldest] = {currentTime, obstacleList};

    mObstacleList.clear();

    for(const auto& item : mObstacleHistory)
	{
		if(!item.mCreatedTime.isValid())
			continue;

		if(item.mCreatedTime < validRange)
			continue;

		auto odometry = mRobot->GetOdometryTraveledSince(item.mCreatedTime);

		auto rotationTf = Rotation2Dd(odometry.mTheta);
		auto tf = Translation2d(-odometry.mPosition);

		for(const auto pt : item.mObstacleList){
			mObstacleList.push_back(rotationTf * tf * pt);
		}

	}



    emit ObstacleMapUpdate(mObstacleList);
}
