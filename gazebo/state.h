#pragma once
#include <robot.h>

namespace Robot {

class Kratos;

namespace State {

class Base
{
public:
	virtual void Initialize(Kratos& robot) = 0;
	virtual void Think(Kratos &robot) = 0;
};

class LeavingPlataform : public Base
{
public:
	virtual void Initialize(Kratos& robot) override;
	virtual void Think(Kratos &robot) override;
};

} //namespace State
} //namespace Robot



