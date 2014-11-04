#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/RaySensor.hh>

namespace gazebo
{
  class Kratos : public ModelPlugin
  {
    physics::LinkPtr m_leftWheelLink;
    physics::LinkPtr m_rightWheelLink;

    physics::JointPtr m_leftWheelJoint;
    physics::JointPtr m_rightWheelJoint;

    sensors::RaySensorPtr m_sensor;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      m_leftWheelLink = _parent->GetLink("left_wheel");
      m_rightWheelLink = _parent->GetLink("right_wheel");
      assert(m_leftWheelLink);
      assert(m_rightWheelLink);

      m_leftWheelJoint = _parent->GetJoint("left_wheel_hinge");
      m_rightWheelJoint = _parent->GetJoint("right_wheel_hinge");
      assert(m_leftWheelJoint);
      assert(m_rightWheelJoint);

      //for(int i = 0; i < _parent->GetChildCount (); i++)
      //{
      //  std::cout << "Child: " << _parent->GetChild(i)->GetName() << "\n";
      //}

      auto link = _parent->GetLink("hokuyo::link");
      assert(link);
      assert(link->GetSensorCount () > 0);

      m_sensor = boost::dynamic_pointer_cast<sensors::RaySensor>(sensors::get_sensor(link->GetSensorName(0)));
      assert(m_sensor);

      //this->node->Subscribe(m_sensor->GetTopic(), &Kratos::OnScan, this);
    

      //std::cout << "Set the force for" << m_leftWheelJoint << std::endl;

      // Listen to the updathtope event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Kratos::OnUpdate, this, _1));

    }

    void OnScan(ConstLaserScanStampedPtr &_msg)
    {
      
    }

    int counter;

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      //std::cout << "UPDATED LIN VEL" << std::endl;
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(math::Vector3(.03, 0, 0));
      //m_rightWheelLink->AddRelativeTorque(math::Vector3(10,0,0));

      
      m_leftWheelJoint->SetForce(1, 0.2);
     

      
      if(counter++ % 100 == 0)
      {
        double rightVel = m_rightWheelJoint->GetVelocity(0xDEADBEEF); //Ugh, the parameter is useless. 
        double leftVel = m_leftWheelJoint->GetVelocity(0xDEADBEEF);

        std::cout << "Velocity: " << leftVel << "\t" << rightVel << std::endl;
        /*
        std::vector<double> pieces;
        m_sensor->GetRanges(pieces);

        for(double dis : pieces)
        {
          std::cout << dis << ", ";
        }
        std::cout << "\n";
*/
      }
      
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Kratos)
}