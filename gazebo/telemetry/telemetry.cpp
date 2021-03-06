#include "telemetry.h"
//#include "../trajectory.h"
#include "../april.h"

#include <iostream>

#include <QApplication>
#include <QTimer>
#include <QGridLayout>

using namespace Robot;
using namespace Eigen;



DepthViewerTab::DepthViewerTab(QWidget *parent)
: QWidget(parent), viewer(new pcl::visualization::PCLVisualizer ("viewer", false))
{
	//mGrowingRegion = new GrowingRegionPointCloudWidget(this);
	mDepthCamera = new QLabel("AAA", this);
	mDepthCamera2 = new QLabel("AAA", this);

	QGridLayout *layout = new QGridLayout;
	layout->addWidget(mDepthCamera, 0, 0);
	layout->addWidget(mDepthCamera2, 0, 1);
	//layout->addWidget(mGrowingRegion, 1, 0, 2, 2);

	qvtkWidget = new QVTKWidget(this);
	qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());

	qvtkWidget->update ();
	viewer->addPointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(), "cloud");
	viewer->resetCamera ();
	qvtkWidget->update ();

	layout->addWidget(qvtkWidget, 0, 1);

	setLayout(layout);
}

void DepthViewerTab::ReceiveData(const Robot::DepthImgData &imgData)
{
	auto size = imgData.data.size();
	std::vector<unsigned char> blobData(imgData.width * imgData.height * 3);
	double maxValue = *std::max_element( imgData.data.begin(), imgData.data.end() );

	for(std::size_t i = 0; i < size; i++)
	{
		float dist = imgData.data[i];

		if(dist <= 0.0001)
			dist = 5.0;

		blobData[i*3] = blobData[i*3+1] = blobData[i*3+2] = (unsigned char)(dist / maxValue * 255);
	}

	QImage qImage(blobData.data(), imgData.width, imgData.height, QImage::Format_RGB888 );
	mDepthCamera->setPixmap(QPixmap::fromImage(qImage));
}

void DepthViewerTab::ReceivePointCloud(const PclPointCloud::Ptr pointCloud)
{
	viewer->updatePointCloud (pointCloud, "cloud");
	viewer->resetCamera ();
	qvtkWidget->update ();
}


WheelOdometryPlot::WheelOdometryPlot(QWidget *parent) : QCustomPlot(parent)
{
	addGraph();
	graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
	graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue

	// configure right and top axis to show ticks but no labels:
	// (see QCPAxisRect::setupFullAxesBox for a quicker method to do this)
	xAxis2->setVisible(true);
	xAxis2->setTickLabels(false);
	yAxis2->setVisible(true);
	yAxis2->setTickLabels(false);

	xAxis->setTickLabelType(QCPAxis::ltDateTime);
	xAxis->setDateTimeFormat("hh:mm:ss");
	xAxis->setAutoTickStep(false);
	xAxis->setTickStep(2);

	// make left and bottom axes always transfer their ranges to right and top axes:
	connect(xAxis, SIGNAL(rangeChanged(QCPRange)), xAxis2, SLOT(setRange(QCPRange)));
	connect(yAxis, SIGNAL(rangeChanged(QCPRange)), yAxis2, SLOT(setRange(QCPRange)));
}

void WheelOdometryPlot::addPoint(double x, double y)
{
	graph(0)->addData(x, y);
	graph(0)->removeDataBefore(x-30);
	graph(0)->rescaleValueAxis();

	// make key axis range scroll with the data (at a constant range size of 8):
	xAxis->setRange(x+0.25, 30, Qt::AlignRight);
	replot();
}


WheelOdometryTab::WheelOdometryTab(QWidget *parent) : QWidget(parent)
{
	leftPlot = new WheelOdometryPlot(this);
	rightPlot = new WheelOdometryPlot(this);

	leftForcePlot = new WheelOdometryPlot(this);
	rightForcePlot = new WheelOdometryPlot(this);

	QGridLayout *layout = new QGridLayout;
	layout->addWidget(leftPlot, 0, 0);
	layout->addWidget(rightPlot, 1, 0);

	layout->addWidget(leftForcePlot, 0, 1);
	layout->addWidget(rightForcePlot, 1, 1);

	setLayout(layout);
}

void WheelOdometryTab::ReceiveData(const Robot::TeenseyStatus &data)
{
	double key = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0;

	leftPlot->addPoint(key, data.leftPosition);
	rightPlot->addPoint(key, data.rightPosition);
}

void WheelOdometryTab::ReceiveWheelVelocities(const std::vector<double> &velocities)
{
	double key = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0;

	leftForcePlot->addPoint(key, velocities[0]);
	rightForcePlot->addPoint(key, velocities[1]);
}




MainWindow::MainWindow(QWidget *parent)
: QMainWindow(parent)
{
	mContext = nzmqt::createDefaultContext(this);
	mContext->start();

	mSocket = mContext->createSocket(nzmqt::ZMQSocket::TYP_SUB, this);
	mSocket->setObjectName("Subscriber.Socket.socket(SUB)");
	mSocket->connectTo("tcp://127.0.0.1:5555");
	mSocket->setOption(nzmqt::ZMQSocket::OPT_SUBSCRIBE, "", 0);

	auto tabbed = new QTabWidget();
	setCentralWidget(tabbed);

	mGridView = new MapOverview(this);
	tabbed->addTab(mGridView, "MAP VIEW");

	mDepthViewer = new DepthViewerTab(this);
	tabbed->addTab(mDepthViewer, "DEPTH MAP");

	mAprilTag = new AprilTagLabel(this);
	tabbed->addTab(mAprilTag, "APRIL TAG");

	mAprilTag2 = new AprilTagLabel(this);
	tabbed->addTab(mAprilTag2, "APRIL TAG2");

	mWheelOdometry = new WheelOdometryTab(this);
	tabbed->addTab(mWheelOdometry, "WHEEL ODOMETRY");

	connect(mSocket, SIGNAL(messageReceived(const QList<QByteArray>&)), SLOT(messageReceived(const QList<QByteArray>&)));

}

void MainWindow::messageReceived(const QList<QByteArray>& messages)
{
	char id = messages[0][0];

	if(id == '\x01')
	{
		QDataStream stream(messages[1]);
		DepthImgData imgData;

		stream >> imgData;

		mDepthViewer->ReceiveData(imgData);
	}

	 if(id == '\x02')
	{
		QDataStream stream(messages[1]);
		TeenseyStatus teensy;
		double time;

		stream >> teensy;
		stream >> time;

		mWheelOdometry->ReceiveData(teensy);
		mGridView->mValueGrid->ReceiveTeensyData(teensy);
		mGridView->mValueGrid->ReceiveTime(time);
	}

	if(id == '\x03')
	{
		auto imgCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		imgCloud->points.resize(messages[1].size() / sizeof(pcl::PointXYZRGB));

		memcpy(imgCloud->points.data(), messages[1].data(), messages[1].size());

		mDepthViewer->ReceivePointCloud(imgCloud);
	}

	if(id == '\x04')
	{
		double distance = *(reinterpret_cast<const double*>(messages[1].data()));
		mGridView->ReceiveDecawaveReading(distance);
		mGridView->mValueGrid->ReceiveDecawave(distance);

	}

	if(id == '\x05')
	{
		QDataStream stream(messages[1]);
		QVector<Eigen::Vector2d> points;

		stream >> points;

		mGridView->ReceiveObstacleMap(points.toStdVector());
	}

	if(id == '\x06')
	{
		double left, right;
		QDataStream stream(messages[1]);

		stream >> left;
		stream >> right;

		std::vector<double> vels;
		vels.push_back(left);
		vels.push_back(right);

		mWheelOdometry->ReceiveWheelVelocities(vels);
		mGridView->ReceiveControlStatus(vels);
	}

	if(id == '\x07')
	{
		QVector<Eigen::Vector2d> points;
		QDataStream stream(messages[1]);

		stream >> points;

		mGridView->ReceivePath(points.toStdVector());
	}

	if(id == '\x08')
	{
		QDataStream stream(messages[1]);
		Teensy2Status teensy;

		stream >> teensy;

		mGridView->mValueGrid->ReceiveTeensy2Data(teensy);
	}

	if(id == '\x09')
	{
		QDataStream stream(messages[1]);
		Vector2d pt;
		double ang;

		stream >> pt;
		stream >> ang;

		mGridView->SetRobot(pt, ang);
	}

	if(id == '\x10')
	{
		QImage image;
		{
			QDataStream stream(messages[1]);
			stream.setVersion(QDataStream::Qt_4_8);
			stream >> image;
		}

		mAprilTag->UpdateImage(image);
	}

	if(id == '\x11')
	{
		//std::cout << "poop\n";
		QList<AprilTagDetectionItem> tags;
		QDataStream stream(messages[1]);
		stream >> tags;

		mAprilTag->ReadTags(tags);
		mGridView->ReadTags(tags);		
	}

	if(id == '\x12')
	{
		Affine2d tf;
		memcpy(&tf, messages[1].data(), messages[1].size());
		mGridView->RobotTagLocation(tf);	
	}

	if(id == '\x13')
	{
		QDataStream stream(messages[1]);
		Vector2d from;
		Vector2d to;

		stream >> from.x();
		stream >> from.y();
		stream >> to.x();
		stream >> to.y();

		mGridView->mMovingLine->setLine(from.x(), from.y(), to.x(), to.y());
	}

	if(id == '\x14')
	{
		QDataStream stream(messages[1]);
		QList<DetectedSample> samples;

		stream >> samples;

		mGridView->ShowSamples(samples);
	}

	if(id == '\x15')
	{
		QDataStream stream(messages[1]);
		QVector<Affine2d> positions;

		stream >> positions;

		mGridView->ShowPositionHistory(positions);
	}


	if(id == '\x16')
	{
		QList<AprilTagDetectionItem> tags;
		QDataStream stream(messages[1]);
		stream >> tags;

		mAprilTag2->ReadTags(tags);
	}

	if(id == '\x17')
	{
		Robot::ImgData mat;
		QDataStream stream(messages[1]);
		stream >> mat;

		QImage image(mat.data.data(), mat.width, mat.height, QImage::Format_RGB888);
		image = image.mirrored(true, false).scaled(1280, 720, Qt::KeepAspectRatio);
		mAprilTag2->UpdateImage(image);
	}
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

