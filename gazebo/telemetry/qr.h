#pragma once

#include <QLabel>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGridLayout>
#include "../april.h"

class AprilTagLabel : public QWidget
{
	QGraphicsScene mScene;
	QGraphicsView mView;

	QGraphicsRectItem* mImage;
	QGraphicsItemGroup* mMatches;

public:
	AprilTagLabel(QWidget *parent = 0);

	void UpdateImage(QImage &original);
	void ReadTags(QList<Robot::AprilTagDetectionItem> tags);
};
