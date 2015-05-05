#include "april.h"
#include "AprilTags/Tag25h9.h"

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
#include <QtCore>
#else
#include <QtConcurrent>
#endif


using namespace Robot;

namespace Robot{
QDataStream &operator<<(QDataStream &out, const AprilTagDetectionItem &item)
{
	out.writeRawData(reinterpret_cast<const char*>(&item), sizeof(AprilTagDetectionItem));

	return out;
}

QDataStream &operator>>(QDataStream &in, AprilTagDetectionItem &item)
{
	in.readRawData(reinterpret_cast<char*>(&item), sizeof(AprilTagDetectionItem));

	return in;
}
}

//////////////////////////
/// AprilTagCamera
//////////////////////////
/*
static void DebugImage(cv::Mat &input)
{
	cv::Mat output;
	cv::resize(input, output, {960, 540});

	cv::imshow("AA", output);
}
*/
inline double standardRad(double t) {
	if (t >= 0.) {
		t = fmod(t+M_PI, M_PI * 2) - M_PI;
	} else {
		t = fmod(t-M_PI, M_PI * -2) + M_PI;
	}
	return t;
}

static void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
	yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
	double c = cos(yaw);
	double s = sin(yaw);
	pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
	roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

AprilTagCamera::AprilTagCamera(QObject* parent) : 
	QObject(parent),
	mTagSize(0.829), //0.829 -- 0.159
	mFx(1315), mFy(1315),
	mPx(1920/2), mPy(1080/2)
{
	m_tagDetector.reset(new AprilTags::TagDetector(AprilTags::tagCodes25h9));

	connect(&mDetectionFutureWatcher, SIGNAL(finished()), this, SLOT(finishedProcessing()));
	connect(this, SIGNAL(ReceiveFrame(QImage)), this, SLOT(ReadFrame(QImage)));
	//connect(this, &AprilTagCamera::ReceiveFrame, this, &AprilTagCamera::ReadFrame);
}

bool AprilTagCamera::IsProcessing()
{
	return mDetectionFutureWatcher.future().isRunning();
}

void AprilTagCamera::ReadFrame(QImage image)
{
	if(IsProcessing())
	{
		std::cout << "April tags is still processing. not starting a new one\n";
		return;
	}

	auto future = QtConcurrent::run([image, this]()
	{
		if(image.format() != QImage::Format_RGB888)
		{
			std::cout << "Warning! Image is not of Format_RGB888 format.\n";
			std::cout << "Double check that April Tag Detection is working.\n";
		}

		cv::Mat cvImage(image.height(), image.width(), CV_8UC3, (uchar*)image.bits(), image.bytesPerLine());
		cv::Mat image_gray;
		std::vector<AprilTags::TagDetection> detections;

		cv::cvtColor(cvImage, image_gray, CV_BGR2GRAY);
		//DebugImage(image_gray);

		detections = m_tagDetector->extractTags(image_gray);

		return detections;
	});

	mDetectionFutureWatcher.setFuture(future);

}

void AprilTagCamera::finishedProcessing()
{
	auto detections = mDetectionFutureWatcher.future().result();
	QList<AprilTagDetectionItem> detectionsItems;

	for(auto &tag : detections)
	{
		AprilTagDetectionItem item;

		tag.getRelativeTranslationRotation(mTagSize, mFx, mFy, mPx, mPy, item.translation, item.rotation);
		item.detection = tag;

		Eigen::Matrix3d F;
		F <<
		1, 0,  0,
		0,  -1,  0,
		0,  0,  1;
		item.rotation = F * item.rotation;

		double yaw, pitch, roll;
		wRo_to_euler(item.rotation, yaw, pitch, roll);

		item.euler = {yaw, pitch, roll};
			
		//std::cout << "\tTag id " << item.detection.id << "\n"; 
		//std::cout << "\t\tT " << item.translation.transpose() << " ("<< item.translation.norm() <<")\n"; 
		//std::cout << "\t\tR " << (item.euler / M_PI * 180.0).transpose() << "\n"; 

		detectionsItems.append(item);
	}

	emit tagsDetected(detectionsItems);
}
