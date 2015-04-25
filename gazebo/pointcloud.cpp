#include "pointcloud.h"
#include "msgpack.h"

#include <pcl/filters/voxel_grid.h>

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

void UpdatePointCloud(const Robot::DepthImgData &imgData, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	//cloud.points.resize(0);
	cloud.is_dense = true;
	cloud.width = imgData.width;
	cloud.height = imgData.height;

	//float* toCopyFrom = (float*)data_arg;
	int index = 0;

	double hfov = imgData.hfov;
	double fl = ((double)imgData.width) / (2.0 *tan(hfov/2.0));

	//double fl2 = ((double)imgData.height) / (2.0 *tan(60.0/2.0));

	// convert depth to point cloud
	cloud.points.resize(imgData.data.size());
	for (uint32_t j=0; j<imgData.height; j++)
	{
		double pAngle;

		//if (imgData.height>1) 
			pAngle = ((double)j - 0.5*(double)(imgData.height-1)) / fl; //atan2( (double)j - 0.5*(double)(imgData.height-1), fl);
		//else            
		//	pAngle = 0.0;

			for (uint32_t i=0; i<imgData.width; i++)
			{
				double yAngle;
			//if (imgData.width>1) 
				yAngle = ((double)i - 0.5*(double)(imgData.width-1)) / fl; //atan2( (double)i - 0.5*(double)(imgData.width-1), fl);
			//else            
			//	yAngle = 0.0;

			double depth = imgData.data[index++]; // + 0.0*this->myParent->GetNearClip();

			// in optical frame
			// hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
			// to urdf, where the *_optical_frame should have above relative
			// rotation from the physical camera *_frame
			pcl::PointXYZRGB point;
			point.x      = -depth * yAngle; //tan(yAngle);
			point.y      = -depth * pAngle; //tan(pAngle);
			//if(depth > this->point_cloud_cutoff_)

			if(depth > 0.01)
			{
				point.z    = depth;
			}
			else //point in the unseeable range
			{
				point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
				cloud.is_dense = false;
			}

			point.r = 255;
			point.g = (char) (std::min(1.0, std::max((point.z + 3.0) / 6.0, 0.0)) * 255.0);

			cloud.points[i + j * imgData.width] = point;
		}
	}

}

namespace Robot
{
PointCloud::Ptr RegionGrowingSegmenter::AsyncronousUpdate(PointCloud::Ptr imgCloud)
{
	using namespace Eigen;
	PointCloud::Ptr imgCloud2(new PointCloud());

	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (imgCloud);
	sor.setLeafSize (0.02f, 0.02f, 0.02f);
	sor.filter (*imgCloud2);


	//Rotate back the camera angle
	const Eigen::Affine3f transform(AngleAxisf(0.52359878, Vector3f::UnitX()));

	for(auto &point : imgCloud2->points)
	{
		point.getVector3fMap() = transform * point.getVector3fMap();
	}

//std::cout << "Running point cloud with: " << numberOfNeighbours << "/" << smoothnessThreshold << "/" << curvatureThreshold << "\n";

	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT>>(new pcl::search::KdTree<pcl::PointXYZRGB>);

	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (imgCloud2);
	normal_estimator.setKSearch (50);
	normal_estimator.compute (*normals);

	pcl::IndicesPtr indices (new std::vector <int>);

	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize (50);
	reg.setMaxClusterSize (1000000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (numberOfNeighbours);
	reg.setInputCloud (imgCloud2);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (smoothnessThreshold / 180.0 * M_PI);
	reg.setCurvatureThreshold (curvatureThreshold);

//std::chrono::time_point<std::chrono::system_clock> start, end;
//start = std::chrono::system_clock::now();

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	if (clusters.empty ())
		return nullptr;

	auto colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

	colored_cloud->sensor_origin_ = imgCloud->sensor_origin_;
	colored_cloud->sensor_orientation_ = imgCloud->sensor_orientation_;
	colored_cloud->width = imgCloud2->width;
	colored_cloud->height = imgCloud2->height;
	colored_cloud->is_dense = false;

	for(auto& i_point : imgCloud2->points )
	{
		pcl::PointXYZRGB point;
		point.x = *(i_point.data);
		point.y = *(i_point.data + 1);
		point.z = *(i_point.data + 2);
		point.r = 255;
		point.g = 255;
		point.b = 255;
		colored_cloud->points.push_back (point);
	}

	std::vector<Eigen::Vector2f> badPoints; 

	for(auto &segment : clusters)
	{
		const auto segmentSize = segment.indices.size();

		//std::cout << "Segment with: " << segmentSize << " points.\n";

		if(segmentSize < 150)
			continue;



		Eigen::Vector3f min(100,100,100);
		Eigen::Vector3f max(-100,-100,-100);

		for(auto &index : segment.indices)
		{
			auto pt = colored_cloud->points[index].getVector3fMap();

			min = min.cwiseMin(pt);
			max = max.cwiseMax(pt);
		}

		//std::cout << "Segment " << " (" << segment.indices.size() << ")\n";
		//std::cout << "\tmin: " << min.transpose() << "\n";
		//std::cout << "\tmax: " << max.transpose() << "\n";
		//std::cout << "\tvolume: " << (min-max).prod() << "\n";

		size_t pointUpCount = 0;
		size_t sampleSize = std::min<size_t>(200, segmentSize);
		for(size_t i = 0; i < segmentSize; i += segmentSize / sampleSize)
		{
			auto pt = normals->points[segment.indices[i]].getNormalVector3fMap();
			float dot = pt.dot(Eigen::Vector3f(0,1,0));
			const float threshold = 0.7;

			//std::cout << "\t\tNorm: " << pt.transpose() << " ("<< dot <<")\n";

			//Check for up and down vectors
			if(dot > threshold || dot < -threshold)
				pointUpCount++;
			//std::cout << "Normal: " << segment.indices[i] << "| \t" << pt.transpose() << "\n";
		}

		//std::cout << "\tpointUpCount: " << pointUpCount << "\n\n";

		uint8_t r = 0, g = 0, b = 0;
		if( pointUpCount < sampleSize / 2) //max.y() > -0.6)
		{
			r = 255;

			//It is a blocking item
			for(auto &index : segment.indices)
			{
				const auto& pt = colored_cloud->points[index].getVector3fMap();
				badPoints.emplace_back(pt.x(), pt.z());
			}
		} 
		else 
		{
			g = 255;
		}

		for(auto &index : segment.indices)
		{
			colored_cloud->points[index].r = r;
			colored_cloud->points[index].g = g;
			colored_cloud->points[index].b = b;
		}
	}


	/*
	for(auto &point : colored_cloud->points)
	{
		if(point.r == 0 && point.g == 0 && point.b == 0)
		{
			//point.x = point.y = point.z = std::numeric_limits<double>::quiet_NaN();
		}
	}
	*/

	return colored_cloud;
}


RegionsType MultiPlaneSegmenter::AsyncronousUpdate(PointCloud::Ptr imgCloud)
{
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
	ne.setMaxDepthChangeFactor (maxDepthChangeFactor);
	ne.setNormalSmoothingSize (normalSmoothingSize);

	pcl::PlaneRefinementComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>::Ptr refinement_compare (new pcl::PlaneRefinementComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label> ());
	refinement_compare->setDistanceThreshold (threshold, true);

	pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> mps;
	mps.setMinInliers (minInliers);
	mps.setAngularThreshold (0.017453 * angularThreshold); //3 degrees
	mps.setDistanceThreshold (distanceThreshold); //2cm
	mps.setRefinementComparator (refinement_compare);

	RegionsType regions;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr contour (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr approx_contour (new pcl::PointCloud<pcl::PointXYZRGB>);
	//char name[1024];

	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
	//double normal_start = pcl::getTime ();
	ne.setInputCloud (imgCloud);
	ne.compute (*normal_cloud);
	//double normal_end = pcl::getTime ();
	//std::cout << "Normal Estimation took " << double (normal_end - normal_start) << std::endl;

	//double plane_extract_start = pcl::getTime ();
	mps.setInputNormals (normal_cloud);
	mps.setInputCloud (imgCloud);

	if (refine)
		mps.segmentAndRefine (regions);
	else
		mps.segment (regions);

	return regions;
}

}