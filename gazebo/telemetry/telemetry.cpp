#include "telemetry.h"
#include "../trajectory.h"

#include <iostream>

#include <QApplication>
#include <QGraphicsRectItem>
#include <QDockWidget>
#include <QTextStream>
#include <QTimer>
#include <QGridLayout>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), mZmqContext(1), mZmqSocket(mZmqContext, ZMQ_SUB), view(&scene), mCore(nullptr)
{
	mZmqSocket.connect("tcp://127.0.0.1:5555");
    mZmqSocket.setsockopt( ZMQ_SUBSCRIBE, "", 0);

	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	timer->start(100);


	mStatusText = new QLabel("AAA", this);
    mDepthCamera = new QLabel("AAA", this);
    mAprilTag = new AprilTagLabel(this);

	//QDockWidget *dockWidget = new QDockWidget(tr("Dock Widget"), this);
	//dockWidget->setAllowedAreas(Qt::TopDockWidgetArea);
	//dockWidget->setWidget(mStatusText);
	//addDockWidget(Qt::RightDockWidgetArea, dockWidget);

    QDockWidget *dockWidget2 = new QDockWidget(tr("Dock Widget2"), this);
    //dockWidget2->setAllowedAreas(Qt::TopDockWidgetArea);
    dockWidget2->setWidget(mDepthCamera);
    addDockWidget(Qt::RightDockWidgetArea, dockWidget2);

    //setCentralWidget(mAprilTag);
	setCentralWidget(&view);
	//setCentralWidget(new ViewerWidget());




    /**
        SETUP THE POINT CLOUD
    */    
    growingRegion = new GrowingRegionPointCloudWidget(this);
    //planeSegments = new PlaneSegmentCloudWidget(this);

    QDockWidget *dockWidget3 = new QDockWidget(tr("Dock Widget3"), this);
    //dockWidget3->setAllowedAreas(Qt::TopDockWidgetArea);
    dockWidget3->setWidget(growingRegion);
    addDockWidget(Qt::RightDockWidgetArea, dockWidget3);


    //QDockWidget *dockWidget4 = new QDockWidget(tr("Dock Widget4"), this);
    //dockWidget3->setAllowedAreas(Qt::TopDockWidgetArea);
    //dockWidget4->setWidget(planeSegments);
    //addDockWidget(Qt::RightDockWidgetArea, dockWidget4);


    /**
        SETUP THE GRID
    */

	qreal windowSize = 10.0;
	for(qreal x = -windowSize; x <= windowSize; x += 1.0)
	{
        auto color = x == 0 ? Qt::black : Qt::gray;

		scene.addLine(x, -windowSize, x, windowSize, QPen(color, 0));
		scene.addLine(-windowSize, x, windowSize, x, QPen(color, 0));
	}



	//scene.addText("Hello, world!");

	//Make a robot with a nose.
	//scene.addRect(-2.0, 3.0, 4.0, 1.0, QPen(Qt::red, 0));
	//auto nose = new QGraphicsRectItem(0.25, -0.1, 0.2, 0.2, mRobotInstance);
	//nose->setPen(QPen(Qt::black, 0));

	mRobotInstance = new QGraphicsPolygonItem();
	mRobotInstance->setPen(QPen(Qt::black, 0));
	mRobotInstance->setPolygon(QVector<QPointF>::fromStdVector(GetRobotPoints<QPointF>()));
	scene.addItem(mRobotInstance);

	//exploreChild(planner.rootNode.get(), planner.rootNode->childs.front().get());

	//Make the whole world visible
	view.setSceneRect(-5, -5, 10, 10);
	view.fitInView(-5, -5, 10, 10);

	//Scale the axis to fit nice visually.
	view.scale(1, -1);


    trajectoryPath = scene.addPath(QPainterPath(), QPen(Qt::blue, 0));


    TrajectoryPlanner mPlanner({0,0}, std::polar(M_PI/2.0, 1.0), {1.0,6.0});
    mPlanner.run(2000);

    int id = 0;
    DrawExploreChild(mPlanner.rootNode.get(), mPlanner.rootNode->childs.front().get(), id);


    connect(growingRegion, SIGNAL(CloudProcessed()), this, SLOT(UpdateWalkabilityMap()));

}

#include <QGraphicsItemGroup>

void MainWindow::UpdateWalkabilityMap()
{
    if(mCore != nullptr)
        delete mCore;

    mCore = new QGraphicsItemGroup();
    //mCore->setScale(5.0 / 512.0);

    //We want to transform the points from x=-2.5...2.5 and z=0...5 
    //To a 2d image, 512x512 
    Eigen::MatrixXi walkabilityMap = Eigen::MatrixXi::Zero(512, 512);
    Eigen::Affine2f toImageTransform = Eigen::Scaling(512.0f / 5.0f, 512.0f / 5.0f) * Eigen::Translation2f(2.5f, 0.0f);
    //int notWalkable = 0;

    auto pointCloud = growingRegion->segmenter.lastProccessed;

    std::cout << "Point cloud " << *pointCloud << "\n";

    for(const auto& pt : pointCloud->points)
    {
        if(pt.g == 255)
            continue;

        auto pt2d = Eigen::Vector2f(pt.x, pt.z);
        Eigen::Vector2i imagePt = (toImageTransform * pt2d).cast<int>();

        if(imagePt.x() < 0 || imagePt.x() > walkabilityMap.rows())
            continue;

        if(imagePt.y() < 0 || imagePt.y() > walkabilityMap.cols())
            continue;

        if(walkabilityMap(imagePt.x(), imagePt.y()) != 0)
            continue;

        walkabilityMap(imagePt.x(), imagePt.y()) = pt.b == 255 ? 2 : 1;
        //notWalkable++;

        //std::cout << "\t" << imagePt.transpose() << "\n";

        Eigen::Vector2f newPt(imagePt.y() * 5.0 / 512.0 + 0.6, imagePt.x() * 5.0 / 512.0 - 2.5);
        //newPt += pointCloud->sensor_origin_.head<2>();

        //std::cout << "\tPt: \t" << pt.getVector3fMap().transpose() << "\n";
        //std::cout << "\t\t\t" << imagePt.transpose() << "\n";
        //std::cout << "\t\t\t" << newPt.transpose() << "\n";

        auto rect = new QGraphicsRectItem(newPt.x(), newPt.y(), 5.0 / 512.0, 5.0 / 512.0);
        rect->setPen(QPen(Qt::red, 0));
        mCore->addToGroup(rect);
    }

    auto forward = pointCloud->sensor_orientation_ * Eigen::Vector3f::UnitX();
    float angle = atan2(forward.y(), forward.x());

    mCore->setPos(pointCloud->sensor_origin_.x(), pointCloud->sensor_origin_.y());
    mCore->setRotation(angle * 180.0 / M_PI);
    
    //mCore->setParentItem(mRobotInstance);
    scene.addItem(mCore);


}

void MainWindow::DrawExploreChild(TrajectoryTreeNode* parent, TrajectoryTreeNode* node, int &id)
{
    QColor color;
    color.setHsvF( (id/2000.0) , 1.0, 1.0);
    id++;

    auto line = scene.addLine(parent->mPoint.x(), parent->mPoint.y(), node->mPoint.x(), node->mPoint.y(), QPen(color, 0));
    mLines.push_back(line);

    mRobotInstance->setRotation(std::arg(node->mRotation) * 180.0 / M_PI);
    mRobotInstance->setPos(node->mPoint.x(), node->mPoint.y());

    for(auto &newChild : node->childs)
    {
       if(newChild)
           DrawExploreChild(node, newChild.get(), id);
    }
}

void MainWindow::update()
{

	zmq::message_t msg;
	while(mZmqSocket.recv(&msg, ZMQ_DONTWAIT))
	{
        char id = ((char*) msg.data())[0];


		//Read result from the network
		msgpack::unpacked result;
    	try { 
            msgpack::unpack(result, ((char*) msg.data())+1, msg.size() - 1);
        }
        catch(std::exception &e)
        {
            std::cerr << "Failed to parse: " << e.what() << "\n";
            return;
        }

        if(id == 0)
        {
            Robot::LocationDataPoint historyPoint;
            result.get().convert(&historyPoint);
            ReadLocation(historyPoint);
        }
        else if (id == 1)
        {
            //Receive path -- TODO
        }
        else if (id == 2)
        {
            Robot::ImgData imgData;
            result.get().convert(&imgData);

            //std::cout << "Got new image!\n";

            QImage qImage(imgData.data.data(), imgData.width, imgData.height, QImage::Format_RGB888 );
            mStatusText->setPixmap(QPixmap::fromImage(qImage));
        }
        else if (id ==3)
        {


            Robot::DepthImgData imgData;
            result.get().convert(&imgData);

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

            //std::cout << "The max val is: " << maxValue << "\n";

            QImage qImage(blobData.data(), imgData.width, imgData.height, QImage::Format_RGB888 );
            mDepthCamera->setPixmap(QPixmap::fromImage(qImage));

            auto& lastPoint = mHistory.mPoints.back();
            auto lastPointPos = lastPoint.mPosition.cast<float>();

            pcl::PointCloud <pcl::PointXYZRGB>::Ptr imgCloud (new pcl::PointCloud <pcl::PointXYZRGB>);
            imgCloud->sensor_origin_ = Eigen::Vector4f(lastPointPos.x(), lastPointPos.y(), lastPointPos.z(), 0.0);
            imgCloud->sensor_orientation_ = Eigen::AngleAxisf((float) lastPoint.mRotation, Eigen::Vector3f::UnitZ());

            UpdatePointCloud(imgData, *imgCloud);
            
            growingRegion->ReceivePointCloud(imgCloud);
            //planeSegments->ReceivePointCloud(imgCloud);

        }
        else if (id == 4)
        {
            Robot::ImgData imgData;
            result.get().convert(&imgData);

            QImage qImage(imgData.data.data(), imgData.width, imgData.height, QImage::Format_RGB888 );

            mAprilTag->ReadAprilTags(qImage);
        }
        else 
        {
            std::cerr << "Reading message of unkown id: " << id << "\n";
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
    //mStatusText->setText(statusStr);
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

