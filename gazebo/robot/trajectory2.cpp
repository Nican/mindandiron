#include "trajectory2.h"
//#include "../trajectory.h"
#include "robot.h"
#include <QtConcurrent>

using namespace Robot;
using namespace Eigen;


///////////////////////////////
//// TrajectoryTreeNode2
///////////////////////////////

TrajectoryTreeNode2::TrajectoryTreeNode2(TrajectorySearch* planner, const Vector2d &point, Complex rotation)
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
}

bool TrajectoryTreeNode2::explore()
{
	if(inGoal())
	{
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

		Vector2d newPoint(mPoint + Vector2d(std::real(bestAngle), std::imag(bestAngle)) * 0.3);
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
	Vector2d diff(mPlanner->mGoal - mPoint);
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
TrajectorySearch::TrajectorySearch(const std::vector<Vector2d> &obstacleList, Vector2d current, double currentAngle, Eigen::Vector2d goal, QObject* parent) : 
	QObject(parent), world(b2Vec2(0.0, 0.0)), mGoal(goal), foundSolution(false)
{
	world.SetAllowSleeping(false);

	mRobotFixture = CreateRobot();

	for(const auto& pt : obstacleList)
	{
		AddObstacle(pt.x(), pt.y());
	}

	rootNode.reset(new TrajectoryTreeNode2(this, current, rotationToCompex(currentAngle)));

	mCreatedTime = QDateTime::currentDateTime();
}

static b2PolygonShape getRobotShape()
{
	b2PolygonShape robotShape;
	static std::vector<b2Vec2> robotPoints = GetRobotPoints<b2Vec2>();

	 for(auto& pt : robotPoints)
	 	pt *= 1.05;

	robotShape.Set(robotPoints.data(), robotPoints.size());
	return robotShape;
}

b2Fixture* TrajectorySearch::CreateRobot()
{
	static b2PolygonShape robotShape = getRobotShape();

	b2BodyDef robotBodyDef;
	robotBodyDef.position.Set(0.0f, 0.0f);
	robotBodyDef.type = b2_dynamicBody;

	b2Body* robotBody = world.CreateBody(&robotBodyDef);

	b2FixtureDef spriteShapeDef;
    spriteShapeDef.shape = &robotShape;
    spriteShapeDef.density = 0.0f;
    spriteShapeDef.isSensor = true;
	return robotBody->CreateFixture(&spriteShapeDef);
}

bool TrajectorySearch::TestPosition(Vector2d pos, double rotation)
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

bool TrajectorySearch::GetResult(std::vector<Vector2d> &points)
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
QObject(parent), mRobot(parent)
{
	connect(&mFutureWatcher, SIGNAL(finished()), this, SLOT(FinishedObstacles()));
}


void TrajectoryPlanner2::UpdateObstacles(SegmentedPointCloud pointCloud)
{
	if(mFutureWatcher.future().isRunning())
	{
		std::cout << "TrajectoryPlanner2::UpdateObstacles is still processing. Not starting a new one\n";
		return;
	}

	QFuture<ObstacleMap> future = QtConcurrent::run([pointCloud](){
		ObstacleMap obstacleMap;
		obstacleMap.mCreatedTime = pointCloud.mTimestamp;

		//We want to transform the points from x=-2.5...2.5 and z=0...5 
	    //To a 2d image, 512x512 
		MatrixXi walkabilityMap = MatrixXi::Zero(512, 512);
		Affine2f toImageTransform = Scaling(512.0f / 5.0f, 512.0f / 5.0f) * Translation2f(2.5f, 0.0f);
		//std::vector<Eigen::Vector2i> obstaclePoints;
		//mObstacleList.clear();
		std::vector<Vector2d> obstacleList;

		for(const auto& pt : pointCloud.mPointCloud->points)
		{
	    	//Green points are good to move through
			if(pt.g == 255)
				continue;

			auto pt2d = Vector2f(pt.x, pt.z);
			Vector2i imagePt = (toImageTransform * pt2d).cast<int>();

			if(imagePt.x() < 0 || imagePt.x() >= walkabilityMap.rows())
				continue;

			if(imagePt.y() < 0 || imagePt.y() >= walkabilityMap.cols())
				continue;

			if(walkabilityMap(imagePt.x(), imagePt.y()) != 0)
				continue;

	        //1 = bad point
	        //2 = not grouped in any cluster
			walkabilityMap(imagePt.x(), imagePt.y()) = pt.b == 255 ? 2 : 1;

			//Red points mean obstacles
			if(pt.r == 255)
			{
				Vector2d newPt(imagePt.y() * 5.0f / 512.0f + 0.6f, imagePt.x() * 5.0f / 512.0f - 2.5f);
				obstacleMap.mObstacleList.push_back(newPt);
			}
	    }	    

	    return obstacleMap;
	});

	mFutureWatcher.setFuture(future);	
}

void TrajectoryPlanner2::FinishedObstacles()
{
	mObstacleMap = mFutureWatcher.future().result();

	mRobot->mSensorLog->SendObstacles(mObstacleMap.mObstacleList);

	 emit ObstacleMapUpdate(mObstacleMap);
}