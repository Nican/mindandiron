#include "telemetry.h"
#include <QApplication>
#include <QTimer>
#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), mZmqContext(1), mZmqSocket(mZmqContext, ZMQ_SUB)
{

	mZmqSocket.connect("tcp://localhost:5563");
    mZmqSocket.setsockopt( ZMQ_SUBSCRIBE, "B", 1);


	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	timer->start(1000);

}

MainWindow::~MainWindow()
{

}

void MainWindow::update()
{
	std::cout << "UPDATE!\n";

	zmq::message_t msg;
	while(mZmqSocket.recv(&msg, ZMQ_DONTWAIT))
	{
		
	}
}



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
