#pragma once
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include "../robot.h"
#include "pointcloud.h"


#include "qr.cpp"



class MainWindow : public QMainWindow
{
    Q_OBJECT

    

public:
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PclPointCloud;
  typedef std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> RegionsType;

  MainWindow(QWidget *parent = 0);
  ~MainWindow(){};

  zmq::context_t mZmqContext;
	zmq::socket_t mZmqSocket;

	QGraphicsScene scene;
	QGraphicsView view;

	QGraphicsPolygonItem* mRobotInstance;
	QLabel* mStatusText;
	QLabel* mDepthCamera;
	AprilTagLabel* mAprilTag;

	Robot::LocationHistory mHistory;

	QGraphicsPathItem* trajectoryPath;

	QVector<QGraphicsLineItem*> mLines;

  /**
    POINT CLOUD
  */
  GrowingRegionPointCloudWidget* growingRegion;  
  PlaneSegmentCloudWidget* planeSegments;


	void ReadLocation(const Robot::LocationDataPoint &historyPoint);
	void DrawExploreChild(TrajectoryTreeNode* parent, TrajectoryTreeNode* child, int &id);

	//void UpdatePointCloud(const Robot::DepthImgData &imgData);

public slots:
	void update();
};

