#include "telemetry.h"
#include <pcl/geometry/polygon_operations.h>
#include <iostream>

#include <QGridLayout>
#include <QTextStream>
#include <QGraphicsRectItem>
#include <QGraphicsItemGroup>


WheelUsageRect::WheelUsageRect(qreal x, qreal y, qreal w, qreal h, QGraphicsItem *parent ) : 
    QGraphicsRectItem(x, y, w, h, parent), mProgress(0.0)
{
    setPen(QPen(Qt::black, 0));
    setBrush(QBrush(Qt::white, Qt::SolidPattern));
}

void WheelUsageRect::paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget)
{
    QGraphicsRectItem::paint(painter, option, widget);

    auto rect = boundingRect();
    rect.setWidth( rect.width() * mProgress );

    painter->setBrush(QBrush(Qt::green, Qt::SolidPattern));
    painter->drawRect(rect);
}


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


    mLeftWheel = new WheelUsageRect(-0.1, 0.3, 0.5, 0.15, mRobotInstance);
    mRightWheel = new WheelUsageRect(-0.1, -0.3 - 0.15, 0.5, 0.15, mRobotInstance);
   


    //Make the whole world visible
    view.setSceneRect(-5, -5, 10, 10);
    view.fitInView(-5, -5, 10, 10);

    //Scale the axis to fit nice visually.
    view.scale(1, -1);

    trajectoryPath = scene.addPath(QPainterPath(), QPen(Qt::blue, 0));
    mPlannedTrajectory = scene.addPath(QPainterPath(), QPen(Qt::red, 0));

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

void MapOverview::ReceiveControlStatus(const std::vector<double> &velocities)
{
    mLeftWheel->mProgress = velocities[0] / 0.66;
    mRightWheel->mProgress = velocities[1] / 0.66;
}

void MapOverview::ReceiveObstacleMap(std::vector<Eigen::Vector2d> points)
{
    if(mCore != nullptr)
        delete mCore;

    mCore = new QGraphicsItemGroup();

    int id = 0;

    for(const auto& imagePt : points)
    {
        QColor color;
        color.setHsvF( (static_cast<double>(id)/points.size()) , 1.0, 1.0);

        auto rect = new QGraphicsRectItem(imagePt.x(), imagePt.y(), 5.0 / 512.0, 5.0 / 512.0);
        rect->setPen(QPen(color, 0));
        mCore->addToGroup(rect);

        id++;
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
    /*
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
    */

    if(points.size() == 0){
        mPlannedTrajectory->setPath(QPainterPath());
        return;
    }

    QPainterPath path({points[0].x(), points[0].y()});

    for(std::size_t i = 1; i < points.size(); i++)
    {
        auto pt = points[i];
        path.lineTo(pt.x(), pt.y());

        path.lineTo(pt.x() + 0.1, pt.y() + 0.1); 
        path.lineTo(pt.x() - 0.1, pt.y() + 0.1); 
        path.lineTo(pt.x() + 0.0, pt.y() - 0.1);
        path.lineTo(pt.x() + 0.1, pt.y() + 0.1);  

        path.lineTo(pt.x(), pt.y());
    }

    mPlannedTrajectory->setPath(path);
    mPlannedTrajectory->setPos(mRobotInstance->pos());
    mPlannedTrajectory->setRotation(mRobotInstance->rotation());

}
