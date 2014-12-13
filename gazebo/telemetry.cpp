#include "telemetry.h"
#include "trajectory.h"

#include <iostream>

#include <QApplication>
#include <QGraphicsRectItem>
#include <QDockWidget>
#include <QTextStream>
#include <QTimer>
#include <QGridLayout>


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
	//setCentralWidget(new ViewerWidget());

	qreal windowSize = 10.0;
	for(qreal x = -windowSize; x <= windowSize; x += 1.0)
	{
        auto color = x == 0 ? Qt::black : Qt::gray;

		scene.addLine(x, -windowSize, x, windowSize, QPen(color, 0));
		scene.addLine(-windowSize, x, windowSize, x, QPen(color, 0));
	}

	//scene.addText("Hello, world!");

	//Make a robot with a nose.
	scene.addRect(-0.5, 3.0, 1.0, 1.0, QPen(Qt::red, 0));
	//auto nose = new QGraphicsRectItem(0.25, -0.1, 0.2, 0.2, mRobotInstance);
	//nose->setPen(QPen(Qt::black, 0));

	mRobotInstance = new QGraphicsPolygonItem();
	mRobotInstance->setPen(QPen(Qt::black, 0));
	mRobotInstance->setPolygon(QVector<QPointF>::fromStdVector(GetRobotPoints<QPointF>()));
	scene.addItem(mRobotInstance);

	//exploreChild(planner.rootNode.get(), planner.rootNode->childs.front().get());

	//Make the whole world visible
	view.setSceneRect(-3, -3, 6, 6);
	view.fitInView(-3, -3, 6, 6);

	//Scale the axis to fit nice visually.
	view.scale(1, -1);


    trajectoryPath = scene.addPath(QPainterPath(), QPen(Qt::black, 0));
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
        char id = ((char*) msg.data())[0];

		//Read result from the network
		msgpack::unpacked result;
    	msgpack::unpack(result, ((char*) msg.data())+1, msg.size() - 1);

        if(id == 0)
        {
            Robot::LocationDataPoint historyPoint;
            result.get().convert(&historyPoint);
            ReadLocation(historyPoint);
        }
        else if (id == 1)
        {
            //std::vector<Eigen::Vector2d> waypoints;
            Robot::State::MoveToWaypointTelemetry telemetry;
            result.get().convert(&telemetry);

            const int waypointsSize = telemetry.mWaypoints.size();

            if(waypointsSize == 0)
                return;

            QPainterPath path(QPointF(telemetry.mWaypoints[0].x(), telemetry.mWaypoints[0].y()));

            for(int i = 1; i < waypointsSize; i++)
            {
                QPointF pt(telemetry.mWaypoints[i].x(), telemetry.mWaypoints[i].y());
                path.lineTo(pt);

                if((waypointsSize - i) == telemetry.mCurrentPoint)
                {
                    path.addEllipse(pt, 0.1, 0.1);
                    path.lineTo(pt);
                }
            }



            trajectoryPath->setPath(path);
        }

	}
}

void MainWindow::ReadLocation(const Robot::LocationDataPoint &historyPoint)
{
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

int main(int argc, char *argv[])
{
	//osg::ArgumentParser arguments(&argc, argv);

    QApplication a(argc, argv);
    MainWindow w;
    //w.show();
    w.showMaximized();

    return a.exec();
}
