#include "telemetry.h"
#include <pcl/geometry/polygon_operations.h>
#include <iostream>

#include <QGridLayout>
#include <QTextStream>
#include <QGraphicsRectItem>
#include <QGraphicsItemGroup>

#include <QFormLayout>
#include <QGroupBox>

using namespace Eigen;

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
	rect.setWidth( rect.width() * std::abs(mProgress) );

	painter->setBrush(QBrush(mProgress < 0.0 ? Qt::red : Qt::green, Qt::SolidPattern));
	painter->drawRect(rect);
}



RobotRect::RobotRect(QGraphicsItem *parent)
	: QGraphicsPolygonItem(parent)
{
	setPen(QPen(Qt::black, 0));
	setPolygon(QVector<QPointF>::fromStdVector(GetRobotPoints<QPointF>()));

	mLeftWheel = new WheelUsageRect(-0.1, 0.3, 0.5, 0.15, this);
	mRightWheel = new WheelUsageRect(-0.1, -0.3 - 0.15, 0.5, 0.15, this);
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
	
	QGroupBox *groupBox0 = new QGroupBox(tr("Total Run Time"));
	QFormLayout *formLayout0 = new QFormLayout;
	formLayout0->addRow(tr("&T:"), mTotalTime = new QLabel("NA"));
	groupBox0->setLayout(formLayout0);

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

	QGroupBox *groupBox3 = new QGroupBox(tr("Decawave"));
	QFormLayout *formLayout3 = new QFormLayout;
	formLayout3->addRow(tr("&Decawave:"), mDecawave = new QLabel("NA"));
	groupBox3->setLayout(formLayout3);

	QVBoxLayout *vbox = new QVBoxLayout;
	vbox->addWidget(groupBox0);
	vbox->addWidget(groupBox);
	vbox->addWidget(groupBox2);
	vbox->addWidget(groupBox3);

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
	mServo->setText(QString::number(data.servoAngle * 180.0 / M_PI));
	mCurrent->setText(QString::number(data.current));
	mVoltage->setText(QString::number(data.voltage));
	mPaused->setText(QString::number(data.isPaused));
}

void ValuesGrid::ReceiveDecawave(double value)
{
	mDecawave->setText(QString::number(value));
}

void ValuesGrid::ReceiveTime(double value)
{
	mTotalTime->setText(QString::number(value));
}



MapOverview::MapOverview(QWidget *parent)
	: QWidget(parent), view(&scene), mCore(nullptr)
{
	//SETUP THE GRID
	qreal windowSize = 30.0;
	for(qreal x = -windowSize; x <= windowSize; x += 1.0)
	{
		auto color = x == 0 ? Qt::black : Qt::gray;

		scene.addLine(x, -windowSize, x, windowSize, QPen(color, 0));
		scene.addLine(-windowSize, x, windowSize, x, QPen(color, 0));
	}

	//make the robot shape
	mRobotInstance = new RobotRect();
	scene.addItem(mRobotInstance);  
	SetRobot({0.0, 0.0}, 0.0);

	//Make the whole world visible
	view.setSceneRect(-15, -15, 30, 30);
	view.fitInView(-15, -15, 30, 30, Qt::KeepAspectRatio);

	//Scale the axis to fit nice visually.
	view.scale(1, -1);

	trajectoryPath = scene.addPath(QPainterPath(), QPen(Qt::blue, 0));
	mPlannedTrajectory = scene.addPath(QPainterPath(), QPen(Qt::red, 0));

	mDecawaveCircle = scene.addEllipse(0, -1.21, 0, 0, QPen(Qt::black, 0));

	mMovingLine = scene.addLine(0, 0, 0, 0, QPen(Qt::black, 0));

	mSampleDetections = new QGraphicsItemGroup(mRobotInstance);
	//scene.addItem(mSampleDetections);  

	mValueGrid = new ValuesGrid(this);

	mTagDetections = new QGraphicsItemGroup();
	scene.addItem(mTagDetections);  

	QGridLayout *layout = new QGridLayout;
	layout->addWidget(&view, 0, 0, 1, 4);
	layout->addWidget(mValueGrid, 0, 5, 1, 1);
	setLayout(layout);


	QPainterPath baseStationPath;
	baseStationPath.addRect(-1.0, -1.0, 2.0, 2.0); //Base rect
	baseStationPath.addRect(-0.75, -1.0, 1.5, 1.5); //Inner starting position

	//Outside ramp
	baseStationPath.addRect(-1.423, -1, 2+0.423*2, 2);
	baseStationPath.addRect(-1, 1, 2, 0.423);
	baseStationPath.moveTo(-1.423, 1);
	baseStationPath.lineTo(-1, 1.423);
	baseStationPath.moveTo(1.423, 1);
	baseStationPath.lineTo(1, 1.423);
	
	//Home beacon
	baseStationPath.addRect(-1.0, -1.423, 2, 0.423);
	baseStationPath.moveTo(-1, -1.0 - 0.423/2);
	baseStationPath.lineTo(0, -1.0);
	baseStationPath.lineTo(1, -1.0 - 0.423/2);
	baseStationPath.lineTo(0, -1.423);
	baseStationPath.closeSubpath();

	auto baseStation = scene.addPath(baseStationPath, QPen(Qt::red, 0));
	baseStation->setRotation(-90);
}

void MapOverview::SetRobot(Eigen::Vector2d pos, double angle)
{
	//std::cout << "Set pos" << pos.tran << "("<< angle << ")\n";
 
	//mRobotInstance->setPos(pos.x(), pos.y());
	//mRobotInstance->setRotation(angle * 180 / M_PI);
}

void MapOverview::ReceiveDecawaveReading(double distance)
{
	mDecawaveCircle->setRect(-distance- 1.2, -distance, distance*2, distance*2);
}

void MapOverview::ReceiveControlStatus(const std::vector<double> &velocities)
{
	mRobotInstance->mLeftWheel->mProgress = velocities[0] / 0.66;
	mRobotInstance->mRightWheel->mProgress = velocities[1] / 0.66;
}

void MapOverview::ReceiveObstacleMap(std::vector<Eigen::Vector2d> points)
{
	if(mCore != nullptr)
		delete mCore;

	mCore = new QGraphicsItemGroup();
	mCore->setPos(mRobotInstance->pos());
	mCore->setRotation(mRobotInstance->rotation());

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

	//std::cout << "Receiving obstacle list with " << id << " items.\n";

	//mCore->setPos(mRobotInstance->pos());
	//mCore->setRotation(mRobotInstance->rotation());
	
	scene.addItem(mCore);
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

void MapOverview::RobotTagLocation(Eigen::Affine2d location)
{
	/*
	qDeleteAll(mTagDetections->childItems());

	Eigen::Rotation2Dd rotation(0);
	rotation.fromRotationMatrix(location.linear());

	auto robot = new RobotRect();
	robot->setPos(location.translation().x(), location.translation().y());
	robot->setRotation(rotation.angle() * 180 / M_PI);

	mTagDetections->addToGroup(robot);
	*/
}

void MapOverview::ShowSamples(const QList<Robot::DetectedSample> &samples)
{
	static const QFont font("Times", 3, QFont::Bold);

	qDeleteAll(mSampleDetections->childItems());

	for(auto& sample : samples)
	{
		Vector2d loc = sample.location;
		auto circle = new QGraphicsEllipseItem(loc.x()-0.2, loc.y()-0.2, 0.4, 0.4, mSampleDetections);
		//circle->setPos(circle->pos() + mRobotInstance->pos());
		circle->setPen(QPen(Qt::red, 0));

		//mSampleDetections->addToGroup(circle);

		//std::cout << "Sample at " << loc.transpose() << "\n";

		/*
		auto text = new QGraphicsTextItem(sample.name);
		text->setPos(loc.x(), loc.y());
		text->setFont(font);
		text->setDefaultTextColor(Qt::red);

		mTagDetections->addToGroup(text);
		*/
	}

}

void MapOverview::ShowPositionHistory(QVector<Eigen::Affine2d> positions)
{
	if(positions.isEmpty()){
		mPlannedTrajectory->setPath(QPainterPath());
		return;
	}

	QPainterPath path({positions[0].translation().x(), positions[0].translation().y()});

	for(int i = 1; i < positions.size(); i++)
	{
		auto pt = positions[i];

		path.lineTo(pt.translation().x(), pt.translation().y());
	}

	trajectoryPath->setPath(path);

	auto& last = positions.last();
	Rotation2Dd rotation2D(0);
	rotation2D.fromRotationMatrix(last.linear());

	mRobotInstance->setPos(last.translation().x(), last.translation().y());
	mRobotInstance->setRotation(rotation2D.angle() * 180 / M_PI);

	//mSampleDetections->setPos(mRobotInstance->pos());
	//mSampleDetections->setRotation(mRobotInstance->rotation());

}

void MapOverview::ReadTags(const QList<Robot::AprilTagDetectionItem> &tags)
{
	//qDeleteAll(mTagDetections->childItems());

	/*
	for(auto& tag : tags)
	{
		if(tag.detection.id != 0)
			continue;

		double euler = tag.euler.y() + (6.4 / 180 * M_PI);
		Vector2d translation = tag.translation.head<2>();

		translation = Rotation2Dd(euler) * translation;
		translation -= Vector2d(1.1, -0.5);

		euler -= tag.cameraRotation;

		auto robot = new RobotRect();
		robot->setPos(translation.x(), translation.y());
		robot->setRotation(euler * 180 / M_PI);


		mTagDetections->addToGroup(robot);
	}
	*/
}