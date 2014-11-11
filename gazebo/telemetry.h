#pragma once
#include <QMainWindow>
#include "robot.h"


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    zmq::context_t mZmqContext;
	zmq::socket_t mZmqSocket;

public slots:
	void update();
};
