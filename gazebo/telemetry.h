#pragma once
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include "robot.h"
#include "pointcloud.h"


// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    zmq::context_t mZmqContext;
	zmq::socket_t mZmqSocket;

	QGraphicsScene scene;
	QGraphicsView view;

	QGraphicsPolygonItem* mRobotInstance;
	QLabel* mStatusText;
	QLabel* mDepthCamera;

	Robot::LocationHistory mHistory;

	QGraphicsPathItem* trajectoryPath;

	QVector<QGraphicsLineItem*> mLines;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;;
	QVTKWidget* qvtkWidget;

	void ReadLocation(const Robot::LocationDataPoint &historyPoint);
	void DrawExploreChild(TrajectoryTreeNode* parent, TrajectoryTreeNode* child, int &id);

	//void UpdatePointCloud(const Robot::DepthImgData &imgData);

public slots:
	void update();
};
