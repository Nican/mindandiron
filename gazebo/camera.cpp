
#include <gazebo/plugins/DepthCameraPlugin.hh>

namespace gazebo
{
  class KratosDepthCameraPlugin : public DepthCameraPlugin
  {
    public: 
      KratosDepthCameraPlugin()
      {
        std::cout << "BUILD PLUGIN\n";
      }


        
          virtual void OnNewDepthFrame(const float *_image,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format)
        {
          std::cout << "GOT NEW DEPTH FRAME!\n";
        }

  };

  GZ_REGISTER_SENSOR_PLUGIN(KratosDepthCameraPlugin)
}

class A {
public:
  A(){ std::cout << "AAAA\n"; }
};

static A a;