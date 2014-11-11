#include "telemetry.h"
#include <QApplication>
#include <QTimer>
#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), mZmqContext(1), mZmqSocket(mZmqContext, ZMQ_SUB)
{

	mZmqSocket.connect("tcp://127.0.0.1:5555");
    mZmqSocket.setsockopt( ZMQ_SUBSCRIBE, "", 0);


	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	timer->start(1000);

}

MainWindow::~MainWindow()
{

}

void MainWindow::update()
{
	std::cout << "Reading bytes!\n";
	zmq::message_t msg;
	while(mZmqSocket.recv(&msg, ZMQ_DONTWAIT))
	{
		msgpack::unpacked result;
    	msgpack::unpack(result, (char*) msg.data(), msg.size());
    	Robot::LocationDataPoint historyPoint;

    	result.get().convert(&historyPoint);

    	std::cout << "Got point: " << historyPoint.mPosition.transpose() << std::endl;

	}
}



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
