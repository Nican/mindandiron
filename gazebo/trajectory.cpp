#include "trajectory.h"
#include <iostream>

static Complex rotationToCompex(double theta)
{
	return std::polar<double>(1.0, theta);
}



TrajectoryTreeNode::TrajectoryTreeNode(TrajectoryPlanner* planner, const Eigen::Vector2d &point, Complex rotation)
	: mPoint(point), mRotation(rotation), mPlanner(planner)
{
	for(double i = -M_PI/8; i < M_PI/8; i += M_PI/32)
	{
		availableAngles.emplace_back(rotationToCompex(i) * mRotation);
	}
}

TrajectoryTreeNode::TrajectoryTreeNode(TrajectoryPlanner* planner, const msgpack::object &o)
	: mPlanner(planner)
{
	if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
    if (o.via.array.size != 3) throw msgpack::type_error();

    o.via.array.ptr[0].convert(mPoint);
    o.via.array.ptr[1].convert(mRotation);
    //o.via.array.ptr[2].convert(availableAngles);
    
    auto& childs = o.via.array.ptr[2];

    if (childs.type != msgpack::type::ARRAY) throw msgpack::type_error();

    for(uint32_t i = 0; i < childs.via.array.size; i++)
    {
    	this->childs.emplace_back(new TrajectoryTreeNode(mPlanner, childs.via.array.ptr[i]));
    }
}

bool TrajectoryTreeNode::explore()
{
	if(inGoal())
		return true;

	if(!childs.empty())
	{
		for(auto& child : childs)
		{
			if(child->explore())
				return true;

			/*
			if(!child->availableAngles.empty())
			{
				return;
			}
			*/
		}

		/*
		childs.remove_if([](const TrajectoryTreeNodePtr& child){
			return child->availableAngles.empty() && child->childs.empty();
		});
		*/
		//return false;
	}


	while(!availableAngles.empty())
	{
		Complex bestAngle(getNextBestAngle());

		availableAngles.remove(bestAngle);

		auto newPoint = mPoint + Eigen::Vector2d(std::real(bestAngle), std::imag(bestAngle)) * 0.1;
		auto node = std::unique_ptr<TrajectoryTreeNode>(new TrajectoryTreeNode(mPlanner, newPoint, bestAngle));
		double x = node->mPoint.x();
		double y = node->mPoint.y();

		if(mPlanner->testPosition(node->mPoint, std::arg(node->mRotation)))
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

Complex TrajectoryTreeNode::getNextBestAngle() const
{
	Complex bestAngle;
	Eigen::Vector2d diff(mPlanner->mGoal - mPoint);
	Complex targetAngle = rotationToCompex(std::atan2(diff.y(), diff.x()));

	//std::cout << "targetAngle: " << targetAngle << "\n";

	for(auto angle : availableAngles)
	{
		if(std::norm(bestAngle) == 0.0 || std::norm(targetAngle - angle) < std::norm(targetAngle - bestAngle))
		{
			bestAngle = angle;
			//std::cout << "\t New Best Angle: " << bestAngle << "\n";
		}
	}

	return bestAngle;
}

inline bool TrajectoryTreeNode::inGoal() const
{
	return (mPoint - mPlanner->mGoal).norm() < 0.1;
}


static bool GetPath(TrajectoryTreeNode* node, std::vector<TrajectoryTreeNode*> &outPath)
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


TrajectoryPlanner::TrajectoryPlanner(const msgpack::object &o)
	: world(b2Vec2(0.0, 0.0))
{
	if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
    if (o.via.array.size != 2) throw msgpack::type_error();

    InitializeWorld();

    o.via.array.ptr[0].convert(mGoal);

    rootNode.reset(new TrajectoryTreeNode(this, o.via.array.ptr[1]));
}


TrajectoryPlanner::TrajectoryPlanner(const Eigen::Vector2d &point, Complex rotation, const Eigen::Vector2d &goal) : 
	world(b2Vec2(0.0, 0.0)),
	rootNode(new TrajectoryTreeNode(this, point, rotation)),
	mGoal(goal)
{
	InitializeWorld();
}

void TrajectoryPlanner::InitializeWorld()
{
	world.SetAllowSleeping(false);

	b2BodyDef robotBodyDef;
	robotBodyDef.position.Set(0.0f, 0.0f);
	robotBodyDef.type = b2_dynamicBody;

	b2Body* robotBody = world.CreateBody(&robotBodyDef);

	b2PolygonShape robotShape;
	static std::vector<b2Vec2> robotPoints = GetRobotPoints<b2Vec2>();

	//std::transform(eigenRobotPoints.begin(), eigenRobotPoints.end(), robotPoints.begin(), [](Eigen::Vector2d pt){ return b2Vec2(pt.x(), pt.y()); } );

	robotShape.Set(robotPoints.data(), robotPoints.size());

	b2FixtureDef spriteShapeDef;
    spriteShapeDef.shape = &robotShape;
    spriteShapeDef.density = 0.0f;
    spriteShapeDef.isSensor = true;
	robotFixture = robotBody->CreateFixture(&spriteShapeDef);


	//For the obstacle
	b2BodyDef obstacleBodyDef;
    obstacleBodyDef.position.Set(0.0f, 3.5f);
    obstacleBodyDef.type = b2_dynamicBody;

    b2Body* obstacleBody = world.CreateBody(&obstacleBodyDef);

    b2PolygonShape groundBox;
    groundBox.SetAsBox(2.0f, 0.5f);

    b2FixtureDef spriteShapeDef2;
    spriteShapeDef2.shape = &groundBox;
    spriteShapeDef2.density = 0.0f;
    spriteShapeDef2.isSensor = true;
    obstacleFixture = obstacleBody->CreateFixture(&spriteShapeDef2);    
}

bool TrajectoryPlanner::testPosition(Eigen::Vector2d pos, double rotation)
{
	robotFixture->GetBody()->SetTransform({(float) pos.x(), (float) pos.y()}, (float) rotation);

	//Recalculate collissions
	world.Step(1.0f / 60.0f, 1, 1);

	return robotFixture->GetBody()->GetContactList() != nullptr;
}

void TrajectoryPlanner::run(int iterations)
{
	for(int i = 0; i < iterations; i ++)
	{
		rootNode->explore();
	}
}


std::vector<TrajectoryTreeNode*> TrajectoryPlanner::getResult() const
{
	std::vector<TrajectoryTreeNode*> outVector;

	GetPath(rootNode.get(), outVector);

	return outVector;
}
