#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>

#include <fstream>
#include <sstream>
#include <string>
#include <cerrno>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include "pointcloud.h"
#include "msgpack.h"

std::string get_file_contents(const char *filename)
{
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in)
  {
    std::ostringstream contents;
    contents << in.rdbuf();
    in.close();
    return(contents.str());
  }
  throw(errno);
}

int main(int argc, char *argv[])
{
	Robot::DepthImgData imgData;
	imgData.width = 512;
	imgData.height = 424;
	imgData.hfov = 70.6 / 180.0 * M_PI;
	imgData.data.resize(imgData.width * imgData.height);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	const int rgb_width = 1920;
	const int rgb_height = 1080;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	//viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->addPointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(), "cloud");

	Robot::RegionGrowingSegmenter segmenter;

	char basePath[] = "/home/nican/zfs/storage/kinect/kinect2";

	for(int i = 155; i < 2000; i++)
	{
		char path[5000];
		sprintf(path, "%s/rgb_%d.bin", basePath, i);

		
		char rgbData[rgb_width*rgb_height*3];
		FILE* pFile = fopen (path , "rb");
		fread (rgbData, 1, sizeof(rgbData),pFile);
		fclose (pFile);
		cv::imshow("rgb", cv::Mat(rgb_height, rgb_width, CV_8UC3, rgbData));

		
		sprintf(path, "%s/depth_%d.bin", basePath, i);
		pFile = fopen (path , "rb");
		fread (&imgData.data.at(0), 1, imgData.data.size() * sizeof(float),pFile);
		fclose (pFile);
		
    	//cv::imshow("depth", cv::Mat(depth_height, depth_width, CV_32FC1, depthData) / 4500.0f);
		/*
		for(auto &point : imgData.data)
			point /= 1000.0;
		
    	UpdatePointCloud(imgData, *cloud);

    	auto cloud2 = segmenter.AsyncronousUpdate(cloud);

    	if(cloud2 == nullptr)
    		continue;
		*/

		sprintf(path, "%s/segmented_%d.pcd", basePath, i);
		
		pcl::io::loadPCDFile(path, *cloud);
		viewer->updatePointCloud(cloud, "cloud");

		//pcl::io::savePCDFileBinaryCompressed (std::string(path), *cloud2);

    	int key = cv::waitKey(1);
    	viewer->spinOnce (1);


    	std::cout << "Showing frame " << i << "\n";
	}

	//cv::waitKey(-1);
	return 0;
}