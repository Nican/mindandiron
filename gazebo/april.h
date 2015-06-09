#pragma once

#include <QDataStream>
#include <QFutureWatcher>
#include <QImage>
#include <eigen3/Eigen/Dense>
#include "AprilTags/TagDetector.h"
#include <memory>
#include <QDateTime>

void DebugImage(cv::Mat &input);

namespace Robot {

class AprilOffset
{
public:
	int mId;
	double mSize; //Size of the tag in meters
	Eigen::Vector2d mOffset;
	double mOrientation; 

	AprilOffset(int id, double size, Eigen::Vector2d offset, double orientation)
		: mId(id), mSize(size), mOffset(offset), mOrientation(orientation)
	{
	}
};

const QVector<AprilOffset> &GetOffsets();
const AprilOffset* GetTagById(int id);

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
	QFutureWatcher<void> mDetectionFutureWatcher;
	QDateTime lastFrameTime;

	double mFx;
	double mFy;
	double mPx;
	double mPy;

	AprilTagCamera(double fx, double fy, double px, double py, QObject* parent);

	bool IsProcessing();

public slots:
	void ReadFrame(QImage image);
	virtual void finishedProcessing(std::vector<AprilTags::TagDetection> detections);

signals:
	void tagsDetected(QList<AprilTagDetectionItem>);
	void ReceiveFrame(QImage image);
};

};
