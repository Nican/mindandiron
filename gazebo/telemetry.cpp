#include "telemetry.h"
#include <QApplication>
#include <QGraphicsRectItem>
#include <QDockWidget>
#include <QTextStream>
#include <QTimer>
#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), mZmqContext(1), mZmqSocket(mZmqContext, ZMQ_SUB), view(&scene)
{

	mZmqSocket.connect("tcp://127.0.0.1:5555");
    mZmqSocket.setsockopt( ZMQ_SUBSCRIBE, "", 0);

	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	timer->start(100);


	mStatusText = new QLabel("AAA", this);

	QDockWidget *dockWidget = new QDockWidget(tr("Dock Widget"), this);
	dockWidget->setAllowedAreas(Qt::TopDockWidgetArea);
	dockWidget->setWidget(mStatusText);
	addDockWidget(Qt::TopDockWidgetArea, dockWidget);

	setCentralWidget(&view);

	qreal windowSize = 10.0;
	for(qreal x = -windowSize; x <= windowSize; x += 1.0)
	{
		scene.addLine(x, -windowSize, x, windowSize, QPen(Qt::black, 0));
		scene.addLine(-windowSize, x, windowSize, x, QPen(Qt::black, 0));
	}

	//scene.addText("Hello, world!");

	//Make a robot with a nose.
	mRobotInstance = scene.addRect(-0.25, -0.25, 0.5, 0.5, QPen(Qt::black, 0));
	auto nose = new QGraphicsRectItem(0.25, -0.1, 0.2, 0.2, mRobotInstance);
	nose->setPen(QPen(Qt::black, 0));

	//Make the whole world visible
	view.setSceneRect(-3, -3, 6, 6);
	view.fitInView(-3, -3, 6, 6);

	//Scale the axis to fit nice visually.
	view.scale(1, -1);
}

MainWindow::~MainWindow()
{

}

void MainWindow::update()
{
	//std::cout << "Reading bytes!\n";
	zmq::message_t msg;
	while(mZmqSocket.recv(&msg, ZMQ_DONTWAIT))
	{

		//Read result from the network
		msgpack::unpacked result;
    	msgpack::unpack(result, (char*) msg.data(), msg.size());
    	Robot::LocationDataPoint historyPoint;

    	result.get().convert(&historyPoint);

    	//Keep history
    	mHistory.mPoints.push_back(historyPoint);

    	//Make lines every so often to keep a history of where we moved
    	if(mHistory.mPoints.size() % 200 == 0)
    	{
    		int points = mHistory.mPoints.size();
    		auto lastPoint = mHistory.mPoints[points - 200];

    		scene.addLine(historyPoint.mPosition.x(), historyPoint.mPosition.y(), lastPoint.mPosition.x(), lastPoint.mPosition.y(), QPen(Qt::red, 0));
    	}

    	//Update our robot position
    	mRobotInstance->setRotation(historyPoint.mRotation * 180.0 / M_PI);
    	mRobotInstance->setPos(historyPoint.mPosition.x(), historyPoint.mPosition.y());

    	//Show the positions
    	QString statusStr;
    	std::ostringstream stringStream;
  		stringStream << historyPoint.mPosition.head<2>().transpose();
 		QTextStream(&statusStr) << "Position = " << stringStream.str().c_str() << "\n" << "rotation = " << historyPoint.mRotation;
    	mStatusText->setText(statusStr);
	}
}



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    //w.show();
    w.showMaximized();

    return a.exec();
}
