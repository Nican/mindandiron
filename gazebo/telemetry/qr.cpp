#include "qr.h"

#include <QGraphicsTextItem>
#include <QTextStream>

AprilTagLabel::AprilTagLabel(QWidget *parent) : 
	QWidget(parent), mView(&mScene), mMatches(nullptr)
{
	mImage = mScene.addRect(0, 0, 1.0, 1.0);

	QGridLayout *layout = new QGridLayout;
	layout->addWidget(&mView, 0, 0);
	setLayout(layout);
}

void AprilTagLabel::UpdateImage(QImage &original)
{
	mImage->setRect(0, 0, original.width(), original.height());
	mImage->setBrush(QBrush(original));

	mView.fitInView(mImage, Qt::KeepAspectRatio);
}

void AprilTagLabel::ReadTags(QList<Robot::AprilTagDetectionItem> tags)
{
	static const QFont font("Times", 20, QFont::Bold);

	if(mMatches != nullptr)
		delete mMatches;

	QList<QGraphicsItem*> items;

	for(auto& tag : tags)
	{
		QVector<QPointF> polygon;
		for(auto& p : tag.detection.p)
			polygon.append({p.first, p.second});

		auto polygonItem = new QGraphicsPolygonItem();
		polygonItem->setPen(QPen(Qt::red, 0));
		polygonItem->setPolygon(polygon);

		QString descriptionText;
		QTextStream stream(&descriptionText);
		stream.setRealNumberPrecision(2);
		stream << "Tag id(" << tag.detection.id << ")\n"
			<< "P: " << tag.translation.x() << ", " << tag.translation.y() << ", " << tag.translation.z() << " ("<< tag.translation.norm() <<")\n"
			<< "R: " << (tag.euler / M_PI * 180.0).y(); 

		auto text = new QGraphicsTextItem(descriptionText);
		text->setPos(polygon[3]);
		text->setFont(font);
		text->setDefaultTextColor(Qt::red);

		items.append(text);
		items.append(polygonItem);
	}

	mMatches = mScene.createItemGroup(items);
}
