#include "telemetry.h"
#include <QApplication>
#include <QGraphicsRectItem>
#include <QDockWidget>
#include <QTextStream>
#include <QTimer>
#include <iostream>
#include "trajectory.h"



MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), mZmqContext(1), mZmqSocket(mZmqContext, ZMQ_SUB), view(&scene)
{

	TrajectoryPlanner planner({0.0,0.0}, Complex(1.0,0.0), {6.0,1.0}); //0 degrees
	planner.run(2000);
	

	auto result = planner.getResult();

	for(int i = 1; i < result.size(); i++)
	{
		auto pt1 = result[i-1];
		auto pt2 = result[i];
		scene.addLine(pt1->mPoint.x(), pt1->mPoint.y(), pt2->mPoint.x(), pt2->mPoint.y(), QPen(Qt::blue, 0));
	}

	/*
	std::function<void(TrajectoryTreeNode*, TrajectoryTreeNode*)> exploreChild = [&](TrajectoryTreeNode* parent, TrajectoryTreeNode* node)
	{
		scene.addLine(parent->mPoint.x(), parent->mPoint.y(), node->mPoint.x(), node->mPoint.y(), QPen(Qt::blue, 0));

		for(auto &newChild : node->childs)
		{
			if(newChild)
				exploreChild(node, newChild.get());
		}
	};
	assert(!planner.rootNode->childs.empty());
	*/
	

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
	scene.addRect(3.0, -0.5, 1.0, 1.0, QPen(Qt::red, 0));
	//auto nose = new QGraphicsRectItem(0.25, -0.1, 0.2, 0.2, mRobotInstance);
	//nose->setPen(QPen(Qt::black, 0));

	mRobotInstance = new QGraphicsPolygonItem();
	mRobotInstance->setPen(QPen(Qt::black, 0));
	mRobotInstance->setPolygon(QPolygonF({
		{0.130f, -0.447675f},
		{0.5366f, -0.447675f},
		{1.25095f, -0.1383f},
		{1.25095f, 0.1383f},
		{0.5366f, 0.447675f},
		{0.1302f, 0.447675f},
		{0.0f, 0.2286f},
		{0.0f, -0.2286f}
	}));
	scene.addItem(mRobotInstance);

	//exploreChild(planner.rootNode.get(), planner.rootNode->childs.front().get());

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
