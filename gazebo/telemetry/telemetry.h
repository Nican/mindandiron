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
#include <QGraphicsRectItem>

#include "qr.h"


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
	void ReceiveWheelVelocities(const std::vector<double> &forces);
};

class WheelUsageRect : public QGraphicsRectItem
{
public:
	double mProgress;

	WheelUsageRect(qreal x, qreal y, qreal w, qreal h, QGraphicsItem *parent = 0);

	virtual void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = 0) override;
};

class RobotRect : public QGraphicsPolygonItem
{
public:
	RobotRect(QGraphicsItem *parent = 0);

	WheelUsageRect* mLeftWheel;
	WheelUsageRect* mRightWheel;
};


class LabelWithPlot : public QLabel
{
	Q_OBJECT
public:
	LabelWithPlot(QWidget *parent = 0);

	void addValue(double value){};

protected:
	virtual void paintEvent(QPaintEvent *) override;
};


class ValuesGrid : public QWidget
{
	Q_OBJECT
public:
	ValuesGrid(QWidget *parent = 0);
	//Time
	QLabel* mTotalTime;

	//Teensy1
	QLabel* mLeftWheel;
	QLabel* mRightWheel;
	QLabel* mLeftVel;
	QLabel* mRightVel;
	QLabel* mAccelX;
	QLabel* mAccelY;
	QLabel* mAceelZ;
	QLabel* mAUTO;

	//Teensy2
	QLabel* mServo;
	QLabel* mCurrent;
	QLabel* mVoltage;
	QLabel* mPaused;

	//Decawave
	QLabel* mDecawave;

	void ReceiveTeensyData(const Robot::TeenseyStatus &data);
	void ReceiveTeensy2Data(const Robot::Teensy2Status &data);
	void ReceiveDecawave(double value);
	void ReceiveTime(double value);
};



class MapOverview : public QWidget
{
	Q_OBJECT
public:
	MapOverview(QWidget *parent = 0);

	QGraphicsScene scene;
	QGraphicsView view;

	QGraphicsItemGroup* mSampleDetections;
	QGraphicsItemGroup* mCore;
	QGraphicsPathItem* trajectoryPath;
	QGraphicsPathItem* mPlannedTrajectory;
	RobotRect* mRobotInstance;

	QVector<QGraphicsLineItem*> mLines;

	QGraphicsEllipseItem* mDecawaveCircle;

	ValuesGrid* mValueGrid;

	QGraphicsItemGroup* mTagDetections;

	QGraphicsLineItem* mMovingLine;



	void ReceiveControlStatus(const std::vector<double> &velocities);

	void ReceiveDecawaveReading(double distance);
	void ReceiveObstacleMap(std::vector<Eigen::Vector2d> points);

	void ReceivePath(std::vector<Eigen::Vector2d> points);
	void SetRobot(Eigen::Vector2d pos, double angle);

	void ReadTags(const QList<Robot::AprilTagDetectionItem> &tags);
	void RobotTagLocation(Eigen::Affine2d location);

	void ShowSamples(const QList<Robot::DetectedSample> &samples);

	void ShowPositionHistory(QVector<Eigen::Affine2d>);
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
	AprilTagLabel* mAprilTag2;


public slots:
	void messageReceived(const QList<QByteArray>& messages);
	
};

