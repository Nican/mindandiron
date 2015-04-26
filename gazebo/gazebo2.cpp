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
#include <gazebo/sensors/ImuSensor.hh>

#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/rendering/Camera.hh>
#include "gazebo2.h"

#include <zmq.hpp>

namespace gazebo
{

  class Kratos : public ModelPlugin
  {
  public:
    zmq::context_t mZmqContext;

    zmq::socket_t mZmqPubSocket;
    zmq::socket_t mZmqSubSocket;

    event::ConnectionPtr newDepthFrameConnection;
    event::ConnectionPtr newImageFrameConnection;
    event::ConnectionPtr newAprilImageFrameConnection;
    event::ConnectionPtr updateConnection;

    physics::ModelPtr mModel;

    physics::LinkPtr m_leftWheelLink;
    physics::LinkPtr m_rightWheelLink;

    physics::JointPtr m_leftWheelJoint;
    physics::JointPtr m_rightWheelJoint;

    sensors::RaySensorPtr m_sensor;
    sensors::DepthCameraSensorPtr m_depthCameraSensor;
    sensors::ImuSensorPtr mImuSensor;

    math::Pose mStartRobotPose;

    RobotGazeboControl mControl;
    double lastUpdate;

    Kratos() : 
      mZmqContext(1), 
      mZmqPubSocket(mZmqContext, ZMQ_PUB),
      mZmqSubSocket(mZmqContext, ZMQ_SUB),
      lastUpdate(0.0)
    {
      mZmqPubSocket.bind ("tcp://*:5556");

      mZmqSubSocket.connect("tcp://127.0.0.1:5557");
      mZmqSubSocket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    }

    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override
    {
      // Store the pointer to the model
      this->mModel = _parent;

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
        //std::cout << "Looking at sensor: " << sensor->GetName() << " ("<< sensor->GetType ()  << ")\n";
        auto cast = boost::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);

        if(cast != nullptr )
        {
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

        auto imu_cast = boost::dynamic_pointer_cast<sensors::ImuSensor>(sensor);

        if(imu_cast != nullptr)
        {
          mImuSensor = imu_cast;
          std::cout << "Found IMU :D\n";
        }
      }        

      // Listen to the updathtope event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Kratos::OnUpdate, this, _1));

      mStartRobotPose = mModel->GetWorldPose();
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

      SendTelemetry(3, data);
    }

    void OnNewDepthFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &/*_format*/)
    {
      Robot::DepthImgData image;
      image.data.resize(_width * _height);
      image.width = _width;
      image.height = _height;
      image.hfov = m_depthCameraSensor->GetDepthCamera()->GetHFOV().Radian();

      for(std::size_t i = 0; i < (_width * _height); i++)
      {
        image.data[i] = _image[i];
      }

      SendTelemetry(2, image);
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

      SendTelemetry(1, data);
    }

    physics::EntityPtr GetBaseStation() const
    {
      return mModel->GetWorld()->GetEntity("kratos_plataform");
    }

    Eigen::Vector3d GetRobotPosition() const
    {
      math::Vector3 pos = mModel->GetWorldPose().pos - mStartRobotPose.pos;
      auto station = GetBaseStation();

      if(station != nullptr)
      {
        pos = mModel->GetWorldPose().pos - station->GetWorldPose().pos;
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

    double GetOrientation() const
    {
      const auto pose = mModel->GetWorldPose();
      Eigen::Quaterniond quat(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);

      auto p1 = quat * Eigen::Vector3d(1,0,0);

      return std::atan2(p1.y(), p1.x());
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo &info)
    {      
      RobotGazeboTickData tickData;
      tickData.simTime = info.simTime.Double();

      double currentLeft = m_leftWheelJoint->GetAngle(1).Radian();
      double currentRight = m_rightWheelJoint->GetAngle(1).Radian();
      auto linAccel = mImuSensor->GetLinearAcceleration();

      //One full rotation should be 23330 ticks
      tickData.leftWheelTicks = static_cast<int>((currentLeft) / (M_PI*2) * 23330.0);
      tickData.rightWheelTicks = static_cast<int>((currentRight) / (M_PI*2) * 23330.0);

      tickData.leftWheelVelocity = m_leftWheelJoint->GetVelocity(0xDEADBEEF);
      tickData.rightWheelVelocity = m_rightWheelJoint->GetVelocity(0xDEADBEEF);

      tickData.robotPosition = GetRobotPosition();
      tickData.robotOrientation = GetOrientation();
      tickData.linearAcceleration = Eigen::Vector3d(linAccel.x, linAccel.y, linAccel.z);

      SendTelemetry(0, tickData);

      auto jointController = mModel->GetJointController();

      zmq::message_t msg;
      while(mZmqSubSocket.recv(&msg, ZMQ_DONTWAIT))
      {
        char id = ((char*) msg.data())[0];

        //Read result from the network
        msgpack::unpacked result;
        try { 
          msgpack::unpack(result, ((char*) msg.data())+1, msg.size() - 1);
        }
        catch(std::exception &e)
        {
          std::cerr << "Failed to parse: " << e.what() << "\n";
          return;
        }

        //std::cout << "rceived message of id " << ((int)id) << "\n";

        if(id == 0)
        {
          RobotGazeboControl controlData;
          result.get().convert(&controlData);

          mControl = controlData;
          lastUpdate = tickData.simTime;
        }
      }

      const double factor = 1 / (0.155);
      double leftVel = mControl.leftVelocity * factor;
      double rightVel = mControl.rightVelocity * factor;

      if(std::abs(lastUpdate - tickData.simTime) > 0.4)
      {
        leftVel = 0.0;
        rightVel = 0.0;
      }

      m_leftWheelJoint->SetMaxForce(0, 100);
      m_rightWheelJoint->SetMaxForce(0, 100);

      m_leftWheelJoint->SetVelocity(0, leftVel);
      m_rightWheelJoint->SetVelocity(0, rightVel);

      //std::cout << "Setting veolocities: " << leftVel << "\t/\t" << rightVel << "\n";
    }

    std::mutex zmqLock;

    template<typename T> 
    void SendTelemetry(char id, const T obj)
    {
      msgpack::sbuffer sbuf;
      sbuf.write(&id, sizeof(id));
      msgpack::pack(sbuf, obj);

      zmq::message_t msg(sbuf.size());
      memcpy(msg.data(), sbuf.data(), sbuf.size());
      
      {
        std::lock_guard<std::mutex> lock(zmqLock);
        mZmqPubSocket.send (msg);
      }
    };
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Kratos)
}