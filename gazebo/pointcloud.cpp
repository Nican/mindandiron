#include "pointcloud.h"


void UpdatePointCloud(const Robot::DepthImgData &imgData, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
	cloud->points.resize(0);
	cloud->is_dense = true;

	//float* toCopyFrom = (float*)data_arg;
	int index = 0;

	double hfov = imgData.hfov;
	double fl = ((double)imgData.width) / (2.0 *tan(hfov/2.0));

	// convert depth to point cloud
	cloud->points.resize(imgData.data.size());
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
			pcl::PointXYZRGBA point;
			point.x      = depth * yAngle; //tan(yAngle);
			point.y      = depth * pAngle; //tan(pAngle);
			//if(depth > this->point_cloud_cutoff_)

			if(depth > 0.01)
			{
				point.z    = depth;
			}
			else //point in the unseeable range
			{
				point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
				cloud->is_dense = false;
			}


			//point.r = 255 * (1024 * rand () / (RAND_MAX + 1.0f));
			//point.g = 255 * (1024 * rand () / (RAND_MAX + 1.0f));
			//point.b = 255 * (1024 * rand () / (RAND_MAX + 1.0f));

			point.r = 255;

			cloud->points[i + j * imgData.width] = point;
		}
	}

}