#pragma once

#include <QDataStream>
#include <QFutureWatcher>
#include <QImage>
#include <eigen3/Eigen/Dense>
#include "AprilTags/TagDetector.h"
#include <memory>
#include <QDateTime>

namespace Robot {

struct AprilTagDetectionItem
{
	Eigen::Vector3d translation;
	Eigen::Matrix3d rotation;
	Eigen::Vector3d euler;
	qint64 time;

	AprilTags::TagDetection detection;
};

QDataStream &operator<<(QDataStream &out, const AprilTagDetectionItem &item);
QDataStream &operator>>(QDataStream &in, AprilTagDetectionItem &item);


class AprilTagCamera : public QObject
{
	Q_OBJECT

public:
	std::shared_ptr<AprilTags::TagDetector> m_tagDetector;
	QFutureWatcher<std::vector<AprilTags::TagDetection>> mDetectionFutureWatcher;
	QDateTime lastFrameTime;

	double mTagSize;
	double mFx;
	double mFy;
	double mPx;
	double mPy;

	AprilTagCamera(QObject* parent);

	virtual void RequestFrame() = 0;

	bool IsProcessing();

public slots:
	void ReadFrame(QImage image);
	virtual void finishedProcessing();

signals:
	void tagsDetected(QList<AprilTagDetectionItem>);
	void ReceiveFrame(QImage image);
};

};
