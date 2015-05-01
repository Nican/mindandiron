#include <QLabel>
#include <QGraphicsScene>
#include <QGridLayout>
#include "../april.h"


class AprilTagLabel : public QWidget
{
	QGraphicsScene mScene;
	QGraphicsView mView;

	QGraphicsRectItem* mImage;

public:
	AprilTagLabel(QWidget *parent = 0) : 
		QWidget(parent), mView(&mScene)
	{
		mImage = mScene.addRect(0, 0, 1.0, 1.0);

		QGridLayout *layout = new QGridLayout;
	    layout->addWidget(&mView, 0, 0);
	    setLayout(layout);
	}

	void UpdateImage(QImage &original)
	{
		mImage->setRect(0, 0, original.width(), original.height());
		mImage->setBrush(QBrush(original));

		mView.fitInView(mImage, Qt::KeepAspectRatio);
	}

};

