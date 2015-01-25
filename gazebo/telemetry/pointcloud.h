#pragma once

#include <QWidget>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../pointcloud.h"


class PointCloudWidget : public QWidget
{
  Q_OBJECT

public:
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	QVTKWidget* qvtkWidget;

	PointCloudWidget(QWidget *parent = 0);

	virtual void ReceivePointCloud(Robot::PointCloud::Ptr &cloud) = 0;
};

class GrowingRegionPointCloudWidget : public PointCloudWidget
{
public:
	Robot::RegionGrowingSegmenter segmenter;

	GrowingRegionPointCloudWidget(QWidget *parent = 0) 
	: PointCloudWidget(parent)
	{
	}

	virtual void ReceivePointCloud(Robot::PointCloud::Ptr &cloud) override;
};


class PlaneSegmentCloudWidget : public PointCloudWidget
{
public:
	typedef Robot::PointT PointT;
	typedef Robot::PointCloud PointCloud;
	typedef Robot::RegionsType RegionsType;

	Robot::MultiPlaneSegmenter segmenter;

	PlaneSegmentCloudWidget(QWidget *parent = 0)
	: PointCloudWidget(parent)
	{
	}

	virtual void ReceivePointCloud(Robot::PointCloud::Ptr &cloud) override;
};
