#pragma once

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>


#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/geometry/polygon_operations.h>
#include <pcl/segmentation/planar_region.h>


#include <future>

#include "robot.h"

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

template <typename T>
struct BaseGroundProcessor 
{
	T lastProccessed;

	std::future<T> segmentFuture;

	bool Update(PointCloud::Ptr cloud)
	{
		if(segmentFuture.valid())
		{
			auto status = segmentFuture.wait_for(std::chrono::seconds(0));

		    if(status != std::future_status::ready)
		        return false;

		    std::cout << "Updated out cloud!\n";
    		lastProccessed = segmentFuture.get();
    		segmentFuture = std::future<T>();

    		return true;
		}

		if(!segmentFuture.valid())
	    {
	        std::cout << "Future does not have a valid state. Begin to process\n";

	        segmentFuture = std::async(std::launch::async, [this, cloud](){
	            auto val = this->AsyncronousUpdate(cloud);
	            std::cout << "Finished async proccessing\n";

	            return val;
	        });

	        //std::cout << "Future state: " << segmentFuture->valid() << "\n";
	    }

	    return false;
	}

	virtual T AsyncronousUpdate(PointCloud::Ptr cloud) = 0;
};


struct RegionGrowingSegmenter : public BaseGroundProcessor<PointCloud::Ptr>
{
	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

	virtual PointCloud::Ptr AsyncronousUpdate(PointCloud::Ptr imgCloud) override;
};

struct MultiPlaneSegmenter : public BaseGroundProcessor<RegionsType>
{
	float threshold_;

	MultiPlaneSegmenter() : threshold_(0.02f)
	{
	}

	virtual RegionsType AsyncronousUpdate(PointCloud::Ptr imgCloud) override;
};


};