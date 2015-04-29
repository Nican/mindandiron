#include "telemetry.h"
#include <pcl/geometry/polygon_operations.h>
#include <iostream>

#include <QGridLayout>
#include <QTextStream>
#include <QGraphicsRectItem>
#include <QGraphicsItemGroup>

#include <QFormLayout>
#include <QGroupBox>

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




LabelWithPlot::LabelWithPlot(QWidget *parent)
    : QLabel("N/A", parent)
{
}


void LabelWithPlot::paintEvent(QPaintEvent * event)
{
    //http://stackoverflow.com/questions/7942996/qt-drawrect-in-background
    QLabel::paintEvent(event);
}


ValuesGrid::ValuesGrid(QWidget *parent) : QWidget(parent)
{
    QGroupBox *groupBox = new QGroupBox(tr("Teensy 1"));
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(tr("&Left:"), mLeftWheel = new QLabel("NA"));
    formLayout->addRow(tr("&Right:"), mRightWheel = new QLabel("NA"));
    formLayout->addRow(tr("&LeftVel:"), mLeftVel = new QLabel("NA"));
    formLayout->addRow(tr("&RightVel:"), mRightVel = new QLabel("NA"));
    formLayout->addRow(tr("&AceelX:"), mAccelX = new QLabel("NA"));
    formLayout->addRow(tr("&AceelY:"), mAccelY = new QLabel("NA"));
    formLayout->addRow(tr("&AceelZ:"), mAceelZ = new QLabel("NA"));
    formLayout->addRow(tr("&AUTO:"), mAUTO = new QLabel("NA"));
    groupBox->setLayout(formLayout);

    QGroupBox *groupBox2 = new QGroupBox(tr("Teensy 2"));
    QFormLayout *formLayout2 = new QFormLayout;
    formLayout2->addRow(tr("&Servo:"), mServo = new QLabel("NA"));
    formLayout2->addRow(tr("&Current:"), mCurrent = new QLabel("NA"));
    formLayout2->addRow(tr("&Voltage:"), mVoltage = new QLabel("NA"));
    formLayout2->addRow(tr("&Paused:"), mPaused = new QLabel("NA"));
    groupBox2->setLayout(formLayout2);

    QVBoxLayout *vbox = new QVBoxLayout;
    vbox->addWidget(groupBox);
    vbox->addWidget(groupBox2);

    setLayout(vbox);

    setMaximumWidth(200);
}

void ValuesGrid::ReceiveTeensyData(const Robot::TeenseyStatus &data)
{
    mLeftWheel->setText(QString::number(data.leftPosition));
    mRightWheel->setText(QString::number(data.rightPosition));
    mLeftVel->setText(QString::number(data.leftVelocity));
    mRightVel->setText(QString::number(data.rightVelocity));
    mAccelX->setText(QString::number(data.acceleration.x()));
    mAccelY->setText(QString::number(data.acceleration.y()));
    mAceelZ->setText(QString::number(data.acceleration.z()));
    mAUTO->setText(QString::number(data.autoFlag));
}

void ValuesGrid::ReceiveTeensy2Data(const Robot::Teensy2Status &data)
{
    mServo->setText(QString::number(data.servoAngle));
    mCurrent->setText(QString::number(data.current));
    mVoltage->setText(QString::number(data.voltage));
    mPaused->setText(QString::number(data.isPaused));
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

    mValueGrid = new ValuesGrid(this);

    QGridLayout *layout = new QGridLayout;
    layout->addWidget(&view, 0, 0, 1, 4);
    layout->addWidget(mValueGrid, 0, 5, 1, 1);
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
