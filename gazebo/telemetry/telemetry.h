#pragma once
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include "../robot.h"
#include "../teensy.h"
#include "../nzmqt/nzmqt.hpp"
#include "pointcloud.h"
#include "qcustomplot.h"


#include "qr.cpp"


class QGraphicsItemGroup;



class DepthViewerTab : public QWidget
{
	Q_OBJECT

public:
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PclPointCloud;
	typedef std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> RegionsType;
	
	DepthViewerTab(QWidget *parent = 0);

	void ReceiveData(const Robot::DepthImgData &data);
	void ReceivePointCloud(const PclPointCloud::Ptr pointCloud);

	//GrowingRegionPointCloudWidget* mGrowingRegion; 
	QLabel* mDepthCamera; 
	QLabel* mDepthCamera2;

	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	QVTKWidget* qvtkWidget;
};

class WheelOdometryPlot : public QCustomPlot
{
	Q_OBJECT

public:
	WheelOdometryPlot(QWidget *parent = 0);
	void addPoint(double x, double y);
};

class WheelOdometryTab : public QWidget
{
	Q_OBJECT

	WheelOdometryPlot* leftPlot;
	WheelOdometryPlot* rightPlot;

	WheelOdometryPlot* leftForcePlot;
	WheelOdometryPlot* rightForcePlot;

public:
	WheelOdometryTab(QWidget *parent = 0);
	void ReceiveData(const Robot::TeenseyStatus &data);
	void ReceiveWheelForce(const std::vector<double> &forces);
};



class MapOverview : public QWidget
{
	Q_OBJECT

public:
	MapOverview(QWidget *parent = 0);

	QGraphicsScene scene;
	QGraphicsView view;

	QGraphicsItemGroup* mCore;
	QGraphicsPathItem* trajectoryPath;
	QGraphicsPathItem* mPlannedTrajectory;
	QGraphicsPolygonItem* mRobotInstance;

	QVector<QGraphicsLineItem*> mLines;

	QGraphicsEllipseItem* mDecawaveCircle;

	//void ReadLocation(const Robot::LocationDataPoint &historyPoint);
	void DrawExploreChild(TrajectoryTreeNode* parent, TrajectoryTreeNode* child, int &id);
	void UpdateWalkabilityMap(DepthViewerTab::PclPointCloud::Ptr pointCloud);

	void ReceiveDecawaveReading(double distance);
	void ReceiveObstacleMap(std::vector<Eigen::Vector2i> points);

	void ReceivePath(std::vector<Eigen::Vector2d> points);
};



class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

	nzmqt::ZMQContext* mContext;
    nzmqt::ZMQSocket* mSocket;

	MainWindow(QWidget *parent = 0);
	~MainWindow(){};

	//zmq::context_t mZmqContext;
	//zmq::socket_t mZmqSocket;

	WheelOdometryTab* mWheelOdometry;
	DepthViewerTab* mDepthViewer;
	MapOverview* mGridView;
	AprilTagLabel* mAprilTag;


public slots:
	void messageReceived(const QList<QByteArray>& messages);
	//void update();
	void UpdateWalkabilityMap();
	
};

