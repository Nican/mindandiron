#include "telemetry.h"
#include <pcl/geometry/polygon_operations.h>
#include <iostream>

#include <QGridLayout>
#include <QTextStream>
#include <QGraphicsRectItem>
#include <QGraphicsItemGroup>



MapOverview::MapOverview(QWidget *parent)
    : QWidget(parent), view(&scene), mCore(nullptr)
{

    //SETUP THE GRID
    qreal windowSize = 10.0;
    for(qreal x = -windowSize; x <= windowSize; x += 1.0)
    {
        auto color = x == 0 ? Qt::black : Qt::gray;

        scene.addLine(x, -windowSize, x, windowSize, QPen(color, 0));
        scene.addLine(-windowSize, x, windowSize, x, QPen(color, 0));
    }

    //make the robot shape
    mRobotInstance = new QGraphicsPolygonItem();
    mRobotInstance->setPen(QPen(Qt::black, 0));
    mRobotInstance->setPolygon(QVector<QPointF>::fromStdVector(GetRobotPoints<QPointF>()));
    scene.addItem(mRobotInstance);

    //Make the whole world visible
    view.setSceneRect(-5, -5, 10, 10);
    view.fitInView(-5, -5, 10, 10);

    //Scale the axis to fit nice visually.
    view.scale(1, -1);

    trajectoryPath = scene.addPath(QPainterPath(), QPen(Qt::blue, 0));

    mDecawaveCircle = scene.addEllipse(0, 0, 0, 0, QPen(Qt::black, 0));

    /*
    TrajectoryPlanner mPlanner({0,0}, std::polar(M_PI/2.0, 1.0), {1.0,6.0});
    mPlanner.run(2000);

    int id = 0;
    DrawExploreChild(mPlanner.rootNode.get(), mPlanner.rootNode->childs.front().get(), id);
    */

    QGridLayout *layout = new QGridLayout;
    layout->addWidget(&view, 0, 0);
    setLayout(layout);

}

void MapOverview::ReceiveDecawaveReading(double distance)
{
    mDecawaveCircle->setRect(-distance, -distance, distance*2, distance*2);
}

void MapOverview::ReceiveObstacleMap(std::vector<Eigen::Vector2i> points)
{
    if(mCore != nullptr)
        delete mCore;

    mCore = new QGraphicsItemGroup();

    for(const auto& imagePt : points)
    {
        Eigen::Vector2f newPt(imagePt.y() * 5.0 / 512.0 + 0.6, imagePt.x() * 5.0 / 512.0 - 2.5);

        auto rect = new QGraphicsRectItem(newPt.x(), newPt.y(), 5.0 / 512.0, 5.0 / 512.0);
        rect->setPen(QPen(Qt::red, 0));
        mCore->addToGroup(rect);
    }

    mCore->setPos(mRobotInstance->pos());
    mCore->setRotation(mRobotInstance->rotation());
    
    scene.addItem(mCore);
}

/*
void MapOverview::ReadLocation(const Robot::LocationDataPoint &historyPoint)
{
    QPointF lastPos = mRobotInstance->pos();
    QPointF nextPos(historyPoint.mPosition.x(), historyPoint.mPosition.y());

    mRobotInstance->setRotation(historyPoint.mRotation * 180.0 / M_PI);

    if(std::abs((lastPos - nextPos).manhattanLength()) > 0.01)
    {
        scene.addLine(nextPos.x(), nextPos.y(), lastPos.x(), lastPos.y(), QPen(Qt::red, 0));

        //Update our robot position
        mRobotInstance->setPos(nextPos);
    }

    //Show the positions
    QString statusStr;
    std::ostringstream stringStream;
    stringStream << historyPoint.mPosition.head<2>().transpose();
    QTextStream(&statusStr) << "Position = " << stringStream.str().c_str() << "\n" << "rotation = " << historyPoint.mRotation;
    //mStatusText->setText(statusStr);
}
*/

void MapOverview::DrawExploreChild(TrajectoryTreeNode* parent, TrajectoryTreeNode* node, int &id)
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

void MapOverview::ReceivePath(std::vector<Eigen::Vector2d> points)
{
    for(auto &line : mLines)
    {
        delete line;
    }

    mLines.clear();

    for(std::size_t i = 1; i < points.size(); i++)
    {
        QColor color;
        color.setHsvF(static_cast<double>(i/points.size()), 1.0, 1.0);

        auto p1 = points[i-1];
        auto p2 = points[i];

        auto line = scene.addLine(p1.x(), p1.y(), p2.x(), p2.y(), QPen(color, 0));
        mLines.push_back(line);
    }
}

void MapOverview::UpdateWalkabilityMap(DepthViewerTab::PclPointCloud::Ptr pointCloud)
{
    if(mCore != nullptr)
        delete mCore;

    mCore = new QGraphicsItemGroup();
    //mCore->setScale(5.0 / 512.0);

    //We want to transform the points from x=-2.5...2.5 and z=0...5 
    //To a 2d image, 512x512 
    Eigen::MatrixXi walkabilityMap = Eigen::MatrixXi::Zero(512, 512);
    Eigen::Affine2f toImageTransform = Eigen::Scaling(512.0f / 5.0f, 512.0f / 5.0f) * Eigen::Translation2f(2.5f, 0.0f);

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