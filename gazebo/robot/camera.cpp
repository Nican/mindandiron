#include "camera.h"
#include <QDir>
#include <QFile>

KratosCamera::KratosCamera(QString name, int width, int height, QObject* parent)
	: QObject(parent), mWidth(width), mHeight(height), mDeviceName(name)
{
	openCamera();
}

void KratosCamera::openCamera()
{
	mDeviceId = getCameraByName(mDeviceName);
	mCapture = cv::VideoCapture(mDeviceId);

	if(!mCapture.isOpened()) {
    	std::cerr << "ERROR: Can't find video device " << mDeviceId << "\n";
    	return;
    }

    mCapture.set(CV_CAP_PROP_FRAME_WIDTH, mWidth);
    mCapture.set(CV_CAP_PROP_FRAME_HEIGHT, mHeight);
}

bool KratosCamera::read(cv::Mat &cv_image)
{
	bool result = mCapture.read(cv_image);

	if(!result)
		return result;		

	cv::Mat dest;
    cvtColor(cv_image, dest,CV_BGR2RGB);
    QImage image((uchar*)dest.data, dest.cols, dest.rows,QImage::Format_RGB888);

    emit CameraFrame(image);

	return result;
}

int KratosCamera::getCameraByName(QString name)
{
	QDir cameraDir("/dev/v4l/by-id");

	for(auto& filePath : cameraDir.entryList(QDir::Files))
	{
		if(filePath.contains(name))
		{
			auto symPath = QFile("/dev/v4l/by-id/" + filePath).symLinkTarget();

			//std::cout << "sympath: " << filePath.toStdString() << " = " << symPath.toStdString() << "\n";
			
			if(symPath == ""){
				return -1;
			}

			bool ok;
			auto id = symPath.right(1).toInt(&ok);

			if(!ok)
				return -1;

			return id;
		}
	}

	return -1;
}

