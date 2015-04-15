#pragma once

#include <memory>
#include <eigen3/Eigen/Dense>
#include <zmq.hpp>
#include "msgpack.h"
#include "state.h"
#include "odometry.h"
#include "april.h"

namespace Robot
{

struct ImgData
{
  std::vector<unsigned char> data;
  unsigned int width;
  unsigned int height;

  MSGPACK_DEFINE(data, width, height);
};

struct DepthImgData
{
  std::vector<float> data;
  unsigned int width;
  unsigned int height;
  float hfov;

  MSGPACK_DEFINE(data, width, height, hfov);
};

}