#pragma once
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include "robot.h"


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

	void ReadLocation(const Robot::LocationDataPoint &historyPoint);
	void DrawExploreChild(TrajectoryTreeNode* parent, TrajectoryTreeNode* child, int &id);

public slots:
	void update();
};
