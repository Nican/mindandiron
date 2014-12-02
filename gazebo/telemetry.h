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

	Robot::LocationHistory mHistory;

public slots:
	void update();
};
