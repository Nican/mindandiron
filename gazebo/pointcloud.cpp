#include "pointcloud.h"


void UpdatePointCloud(const Robot::DepthImgData &imgData, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	cloud.points.resize(0);
	cloud.is_dense = true;
	cloud.width = imgData.width;
	cloud.height = imgData.height;

	//float* toCopyFrom = (float*)data_arg;
	int index = 0;

	double hfov = imgData.hfov;
	double fl = ((double)imgData.width) / (2.0 *tan(hfov/2.0));

	// convert depth to point cloud
	cloud.points.resize(imgData.data.size());
	for (uint32_t j=0; j<imgData.height; j++)
	{
		double pAngle;

		if (imgData.height>1) 
			pAngle = ((double)j - 0.5*(double)(imgData.height-1)) / fl; //atan2( (double)j - 0.5*(double)(imgData.height-1), fl);
		else            
			pAngle = 0.0;

		for (uint32_t i=0; i<imgData.width; i++)
		{
			double yAngle;
			if (imgData.width>1) 
				yAngle = ((double)i - 0.5*(double)(imgData.width-1)) / fl; //atan2( (double)i - 0.5*(double)(imgData.width-1), fl);
			else            
				yAngle = 0.0;

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
	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (imgCloud);
	normal_estimator.setKSearch (50);
	normal_estimator.compute (*normals);

	pcl::IndicesPtr indices (new std::vector <int>);

	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize (50);
	reg.setMaxClusterSize (1000000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (30);
	reg.setInputCloud (imgCloud);
	//reg.setIndices (indices);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (6.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold (1.0);

	//std::chrono::time_point<std::chrono::system_clock> start, end;
	//start = std::chrono::system_clock::now();

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	//pcl::PointIndices floorIndicies;
	//reg.getSegmentFromPoint( imgData.width * 500 + imgData.width / 2 , floorIndicies);

	// pcl::PointCloud <pcl::PointXYZRGB>::Ptr imgCloud2 (new pcl::PointCloud <pcl::PointXYZRGB>(*imgCloud, floorIndicies.indices));


	//end = std::chrono::system_clock::now();

	//std::chrono::duration<double> elapsed_seconds = end-start;
	// std::time_t end_time = std::chrono::system_clock::to_time_t(end);

	//std::cout << "finished computation at " << std::ctime(&end_time) << "elapsed time: " << elapsed_seconds.count() << "s\n";

	//filterPointCloud(imgCloud, *cloud, filter);

	auto newCloud = reg.getColoredCloud ();
	assert(newCloud != nullptr);

	return newCloud;
}


RegionsType MultiPlaneSegmenter::AsyncronousUpdate(PointCloud::Ptr imgCloud)
{
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
	ne.setMaxDepthChangeFactor (0.02f);
	ne.setNormalSmoothingSize (20.0f);

	pcl::PlaneRefinementComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>::Ptr refinement_compare (new pcl::PlaneRefinementComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label> ());
	refinement_compare->setDistanceThreshold (threshold_, true);

	pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> mps;
	mps.setMinInliers (200);
	mps.setAngularThreshold (0.017453 * 3.0); //3 degrees
	mps.setDistanceThreshold (0.02); //2cm
	mps.setRefinementComparator (refinement_compare);

	RegionsType regions;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr contour (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr approx_contour (new pcl::PointCloud<pcl::PointXYZRGB>);
	//char name[1024];

	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
	double normal_start = pcl::getTime ();
	ne.setInputCloud (imgCloud);
	ne.compute (*normal_cloud);
	double normal_end = pcl::getTime ();
	std::cout << "Normal Estimation took " << double (normal_end - normal_start) << std::endl;

	double plane_extract_start = pcl::getTime ();
	mps.setInputNormals (normal_cloud);
	mps.setInputCloud (imgCloud);

	if (true)
		mps.segmentAndRefine (regions);
	else
		mps.segment (regions);

	return regions;
}

}