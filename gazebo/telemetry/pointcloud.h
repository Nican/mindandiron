#pragma once

#include <QWidget>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../pointcloud.h"

class QDialog;
class QFormLayout;
class QDoubleSpinBox;
class QCheckBox;
class QSpinBox;

class PointCloudWidget : public QWidget
{
  Q_OBJECT

public:
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	QVTKWidget* qvtkWidget;

	QDialog* mDialog;
	QFormLayout* mDialogLayout;

	PointCloudWidget(QWidget *parent = 0);

	virtual void ReceivePointCloud(Robot::PointCloud::Ptr &cloud) = 0;

public slots:
	void OpenSettings();
};

class GrowingRegionPointCloudWidget : public PointCloudWidget
{
	Q_OBJECT

public:
	Robot::RegionGrowingSegmenter segmenter;

	QSpinBox* numberOfNeighbours;
	QDoubleSpinBox* smoothnessThreshold;
	QDoubleSpinBox* curvatureThreshold;

	GrowingRegionPointCloudWidget(QWidget *parent = 0);

	virtual void ReceivePointCloud(Robot::PointCloud::Ptr &cloud) override;

public slots:
	void UpdateSettings();
};


class PlaneSegmentCloudWidget : public PointCloudWidget
{
	Q_OBJECT
	
public:
	typedef Robot::PointT PointT;
	typedef Robot::PointCloud PointCloud;
	typedef Robot::RegionsType RegionsType;

	Robot::MultiPlaneSegmenter segmenter;

	QDoubleSpinBox* threshold;
	QDoubleSpinBox* maxDepthChangeFactor;
	QDoubleSpinBox* normalSmoothingSize;

	QSpinBox* minInliers;
	QDoubleSpinBox* angularThreshold;
	QDoubleSpinBox* distanceThreshold;

	QCheckBox* refine; 

	PlaneSegmentCloudWidget(QWidget *parent = 0);

	virtual void ReceivePointCloud(Robot::PointCloud::Ptr &cloud) override;

public slots:
	void UpdateSettings();
};
