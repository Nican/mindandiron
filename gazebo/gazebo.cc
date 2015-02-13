#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/RaySensor.hh>

#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo/sensors/CameraSensor.hh"

#include "gazebo/rendering/DepthCamera.hh"
#include "gazebo/rendering/Camera.hh"

#include "robot.h"

namespace gazebo
{

class WheelJoint : public Robot::Wheel
{
public:

  physics::JointPtr mJoint;

  WheelJoint(physics::JointPtr joint) : mJoint(joint)
  {
  }

  virtual double GetRotationVelocity() const override
  {
    return mJoint->GetVelocity(0xDEADBEEF);
  }

  virtual void SetForce(double force) override
  {
    mJoint->SetForce(0, -force);
  }

};

class TRSGazebo : public Robot::TotalRoboticStation 
{
public:
  physics::EntityPtr mEntity;
  math::Pose startPose;


  TRSGazebo(physics::EntityPtr entity) : mEntity(entity)
  {
    startPose = mEntity->GetWorldPose();
  }

  virtual Eigen::Vector3d GetPosition() const override
  {
    math::Vector3 pos = mEntity->GetWorldPose().pos - startPose.pos;

    return Eigen::Vector3d(pos.x, pos.y, pos.z);
  }

  virtual double GetOrientation() const override
  {
    const auto pose = mEntity->GetWorldPose();
    Eigen::Quaterniond quat(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);

    auto p1 = quat * Eigen::Vector3d(0,1,0);

    return std::atan2(p1.y(), p1.x());
  }
};


class AprilTagCameraMotorGazebo : public Robot::AprilTagServo
{
public:

  physics::EntityPtr mEntity;

  AprilTagCameraMotorGazebo(physics::EntityPtr entity) : mEntity(entity)
  {
  }

  virtual double GetPosition() const override
  {
    auto pose = mEntity->GetRelativePose();

    return pose.rot.GetYaw();
  }

  virtual void SetPosition(double radians) override
  {
    auto pose = mEntity->GetRelativePose();
    pose.rot.SetFromEuler( pose.rot.GetPitch(), pose.rot.GetRoll(), radians );

    mEntity->SetRelativePose(pose);
  }
};


  class Kratos : public ModelPlugin
  {
    event::ConnectionPtr newDepthFrameConnection;
    event::ConnectionPtr newImageFrameConnection;

    event::ConnectionPtr newAprilImageFrameConnection;


    physics::LinkPtr m_leftWheelLink;
    physics::LinkPtr m_rightWheelLink;

    physics::JointPtr m_leftWheelJoint;
    physics::JointPtr m_rightWheelJoint;

    sensors::RaySensorPtr m_sensor;

    sensors::DepthCameraSensorPtr m_depthCameraSensor;

    std::shared_ptr<Robot::Kratos> m_kratos;

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

      auto aprilMotor = _parent->GetLink("april_tag_link");


      for(sensors::SensorPtr &sensor : sensors::SensorManager::Instance()->GetSensors())
      {
        std::cout << "Looking at sensor: " << sensor->GetName() << " ("<< sensor->GetType ()  << ")\n";

        //auto sensor2 = sensor.get();
        //auto cast = dynamic_cast<sensors::DepthCameraSensor*>(sensor2);
        auto cast = boost::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);

        if(cast != nullptr )
        {
          //std::cout << "\tWe found our princess!\n";

          auto depthCamera = cast->GetDepthCamera();
          this->newDepthFrameConnection = depthCamera->ConnectNewDepthFrame(
            boost::bind(&Kratos::OnNewDepthFrame,
            this, _1, _2, _3, _4, _5));

          this->newImageFrameConnection = depthCamera->ConnectNewImageFrame(
            boost::bind(&Kratos::OnNewImageFrame,
            this, _1, _2, _3, _4, _5));


          cast->SetActive(true);
          m_depthCameraSensor = cast;
        }

        auto camera_cast = boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

        if(camera_cast != nullptr)
        {
          auto camera = camera_cast->GetCamera();

          this->newAprilImageFrameConnection = camera->ConnectNewImageFrame(
            boost::bind(&Kratos::OnNewImageAprilTagFrame, this, _1, _2, _3, _4, _5));
        }
      }        

      // Listen to the updathtope event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Kratos::OnUpdate, this, _1));


      Robot::RobotMotion motion;
      Robot::RobotSensors sensors;

      motion.mLeftWheel = std::make_shared<WheelJoint>(m_leftWheelJoint);
      motion.mRightWheel = std::make_shared<WheelJoint>(m_rightWheelJoint);
      motion.mAprilServo = std::make_shared<AprilTagCameraMotorGazebo>(aprilMotor);

      sensors.mTRS = std::make_shared<TRSGazebo>(this->model);

      m_kratos.reset(new Robot::Kratos(motion, sensors));
    }

    void OnNewImageAprilTagFrame(const unsigned char * image, unsigned int width, unsigned int height, unsigned int depth, const std::string &)
    {
      std::vector<unsigned char> imgData(width * height * depth);
      assert(imgData.size() > 0);
      memcpy( imgData.data(), image, imgData.size());

      Robot::ImgData data;
      data.width = width;
      data.height = height;
      data.data = imgData;

      
      m_kratos->ReceiveAprilImage(data);
    }

    void OnNewDepthFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &/*_format*/)
    {
      /*
      float min, max;
      min = 1000;
      max = 0;
      for (unsigned int i = 0; i < _width * _height; i++)
      {
        if (_image[i] > max)
          max = _image[i];
        if (_image[i] < min)
          min = _image[i];
      }
      */

      Robot::DepthImgData image;
      image.data.resize(_width * _height);
      image.width = _width;
      image.height = _height;
      image.hfov = m_depthCameraSensor->GetDepthCamera()->GetHFOV().Radian();

      for(int i = 0; i < (_width * _height); i++)
      {
        image.data[i] = _image[i];
      }

      m_kratos->ReceiveDepth(image);

      /*rendering::Camera::SaveFrame(_image, this->width,
        this->height, this->depth, this->format,
        "/tmp/depthCamera/me.jpg");
        */
    }

    void OnNewImageFrame(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int _depth,
                              const std::string &_format)
    {
      std::vector<unsigned char> imgData(_width * _height * _depth);
      assert(imgData.size() > 0);
      memcpy( imgData.data(), _image, imgData.size());

      Robot::ImgData data;
      data.width = _width;
      data.height = _height;
      data.data = imgData;

      m_kratos->ReceiveKinectImage(data);

      /*rendering::Camera::SaveFrame(_image, this->width,
        this->height, this->depth, this->format,
        "/tmp/depthCamera/me.jpg");
        */
    }

    int counter;

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo &info)
    {      
      m_kratos->Update(info.simTime.Double());

      
      if(counter++ % 100 == 0)
      {
        double rightVel = m_rightWheelJoint->GetVelocity(0xDEADBEEF); //Ugh, the parameter is useless. 
        double leftVel = m_leftWheelJoint->GetVelocity(0xDEADBEEF);

        //std::cout << "Velocity: " << leftVel << "\t" << rightVel << std::endl;
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