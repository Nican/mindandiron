#pragma once

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/planar_region.h>
#include <boost/make_shared.hpp>

#include "util.h"

namespace Robot
{
class DepthImgData;
}

void UpdatePointCloud(const Robot::DepthImgData &imgData, pcl::PointCloud<pcl::PointXYZRGB> &cloud);


template <typename PointT> void
filterPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                     pcl::PointCloud<PointT> &cloud_out,
                     std::function<bool(PointT)> filter)
{
  // Do we want to copy everything?
  // if (indices.size () == cloud_in.points.size ())
  // {
  //   cloud_out = cloud_in;
  //   return;
  // }

  // Allocate enough space and copy the basics
  cloud_out.points.clear(); //resize (indices.size ());
  cloud_out.header   = cloud_in.header;
  //cloud_out.width    = static_cast<uint32_t>(indices.size ());
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;
  cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
  cloud_out.sensor_origin_ = cloud_in.sensor_origin_;

  // Iterate over each point
  //for (size_t i = 0; i < indices.size (); ++i)
  //  cloud_out.points[i] = cloud_in.points[indices[i]];

  //for(int i = 0; i < size; i++)
  for(const auto& pt: cloud_in.points)
  {
  	if(filter(pt))
  	{
  		cloud_out.points.push_back(pt);
  	}
  }

  cloud_out.width    = static_cast<uint32_t>(cloud_out.points.size ());
}


namespace Robot
{

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> RegionsType;

struct RegionGrowingSegmenter : public BaseGroundProcessor<PointCloud::Ptr, PointCloud::Ptr>
{
	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

	int numberOfNeighbours;
	double smoothnessThreshold;
	double curvatureThreshold;

	RegionGrowingSegmenter() :
		numberOfNeighbours(60),
		smoothnessThreshold(5.0),
		curvatureThreshold(0.1)
	{
	}

	virtual PointCloud::Ptr AsyncronousUpdate(PointCloud::Ptr imgCloud) override;
};

struct MultiPlaneSegmenter : public BaseGroundProcessor<PointCloud::Ptr, RegionsType>
{
	float threshold;
	float maxDepthChangeFactor;
	float normalSmoothingSize;

	int minInliers;
	float angularThreshold;
	float distanceThreshold;

	bool refine;

	MultiPlaneSegmenter() : 
		threshold(0.02f), 
		maxDepthChangeFactor(0.02f),
		normalSmoothingSize (20.0f),
		minInliers (200),
		angularThreshold (3.0),
		distanceThreshold (0.02),
		refine(true)
	{
	}

	virtual RegionsType AsyncronousUpdate(PointCloud::Ptr imgCloud) override;
};


};