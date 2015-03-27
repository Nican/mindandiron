#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/common/PID.hh>

#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/sensors/CameraSensor.hh>

#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/rendering/Camera.hh>

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
    //std::cout << "Angle: " << mJoint->GetAngle(1).Radian() << "\n";
    return mJoint->GetVelocity(0xDEADBEEF);
  }

  virtual void SetForce(double force) override
  {
    mJoint->SetForce(0, force);
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
    auto station = GetBaseStation();

    if(station != nullptr)
    {
      pos = mEntity->GetWorldPose().pos - station->GetWorldPose().pos;
    }
    else
    {
      static bool showedWarning = false;

      if(!showedWarning)
      {
        showedWarning = true;
        std::cerr << "Did not find the base station 'kratos_plataform' when grabing the position from TRS\n";
      }
    }

    return Eigen::Vector3d(pos.x, pos.y, pos.z);
  }

  virtual double GetOrientation() const override
  {
    const auto pose = mEntity->GetWorldPose();
    Eigen::Quaterniond quat(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);

    auto p1 = quat * Eigen::Vector3d(1,0,0);

    return std::atan2(p1.y(), p1.x());
  }

  physics::EntityPtr GetBaseStation() const
  {
    return mEntity->GetWorld()->GetEntity("kratos_plataform");
  }
};


class AprilTagCameraMotorGazebo : public Robot::AprilTagServo
{
public:

  physics::JointPtr mJoint;
  //common::PID PID;

  AprilTagCameraMotorGazebo(physics::JointPtr entity) : mJoint(entity)
  {
    assert(entity);

    //PID.Init(800, 0, 3, 50, -50, 50, -50);
  }

  virtual double GetPosition() const override
  {
    //auto pose = mEntity->GetRelativePose();

    return mJoint->GetAngle(0).Radian();
  }

  virtual void SetPosition(double radians) override
  {
    //To make this better http://answers.gazebosim.org/question/2541/setanglesetposition-having-erratic-effect-on-robot/
    //Setting the position of a joint is broken
    //mJoint->SetPosition(0, radians);

    //double gasError = this->gasPedalState - this->gasPedalCmd;
    //double gasCmd = PID.Update(gasError, dt);
    //this->gasPedalJoint->SetForce(0, gasCmd);

    /*
    auto pose = mEntity->GetRelativePose();
    pose.rot.SetFromEuler( pose.rot.GetPitch(), pose.rot.GetRoll(), radians );

    mEntity->SetRelativePose(pose);
    */
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
      motion.mAprilServo = std::make_shared<AprilTagCameraMotorGazebo>(_parent->GetJoint("april_tag_hinge"));

      sensors.mTRS = std::make_shared<TRSGazebo>(this->model);

      m_kratos.reset(new Robot::Kratos(motion, sensors));
      m_kratos->mOdometry.mPosition = sensors.mTRS->GetPosition().head<2>();
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

      for(std::size_t i = 0; i < (_width * _height); i++)
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

    double nextTickerUpdate;
    double lastLeftWheelAngle;
    double lastRightWheelAngle;

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo &info)
    {      
      double simTime = info.simTime.Double();
      m_kratos->Update(info.simTime.Double());

      if(simTime > nextTickerUpdate)
      {
        nextTickerUpdate = simTime + 0.01;

        double currentLeft = m_leftWheelJoint->GetAngle(1).Radian();
        double currentRight = m_rightWheelJoint->GetAngle(1).Radian();

        //One full rotation should be 23330 ticks
        int leftTicks = static_cast<int>((currentLeft - lastLeftWheelAngle) / (M_PI*2) * 23330.0);
        int rightTicks = static_cast<int>((currentRight - lastRightWheelAngle) / (M_PI*2) * 23330.0);

        m_kratos->ReceiveWheelTicks(leftTicks, rightTicks);

        lastLeftWheelAngle = currentLeft;
        lastRightWheelAngle = currentRight;
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