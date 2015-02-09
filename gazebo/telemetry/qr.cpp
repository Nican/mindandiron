#include <QLabel>

#include "../april.h"

#include "opencv2/opencv.hpp"
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag36h11.h"

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

static inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
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

class AprilTagLabel : public QLabel
{
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  QRTagFinder qrFinder;

public:
	AprilTagLabel(QWidget *parent = 0) : QLabel("C", parent),
		m_tagSize(0.704),
	    m_fx(1100),
    	m_fy(1100),
   		m_px(1920/2),
   		m_py(1080/2)
	{

	}

	void ReadAprilTags(QImage &original)
	{
		auto image = cv::Mat(original.height(), original.width(), CV_8UC3, original.bits(), original.bytesPerLine()).clone();

		if(qrFinder.Update(image))
    	{
			std::vector<AprilTags::TagDetection> detections = qrFinder.lastProccessed;

			std::cout << "Finished calcualting QR code. Found " << detections.size() << " entries.\n";

			for (int i=0; i<detections.size(); i++) {

				for( double principal = 1000; principal<= 2000; principal += 100 )
				{


				Eigen::Vector3d translation;
	    		Eigen::Matrix3d rotation;
	    		detections[i].getRelativeTranslationRotation(m_tagSize, principal, principal, m_px, m_py, translation, rotation);

	    		cout << principal << endl;
	    		cout << "\tdistance=" << translation.norm()
			         << "m, x=" << translation(0)
			         << ", y=" << translation(1)
			         << ", z=" << translation(2) << endl;

			    }

		      //print_detection(detections[i]);
		    }
		}

		setPixmap(QPixmap::fromImage(original.scaled(1280, 800)));
	}

	  void print_detection(AprilTags::TagDetection& detection) const {
	    cout << "  Id: " << detection.id
	         << " (Hamming: " << detection.hammingDistance << ")";

	    // recovering the relative pose of a tag:

	    // NOTE: for this to be accurate, it is necessary to use the
	    // actual camera parameters here as well as the actual tag size
	    // (m_fx, m_fy, m_px, m_py, m_tagSize)

	    Eigen::Vector3d translation;
	    Eigen::Matrix3d rotation;
	    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
	                                             translation, rotation);

	    Eigen::Matrix3d F;
	    F <<
	      1, 0,  0,
	      0,  -1,  0,
	      0,  0,  1;
	    Eigen::Matrix3d fixed_rot = F*rotation;
	    double yaw, pitch, roll;
	    wRo_to_euler(fixed_rot, yaw, pitch, roll);

	    cout << "  distance=" << translation.norm()
	         << "m, x=" << translation(0)
	         << ", y=" << translation(1)
	         << ", z=" << translation(2)
	         << ", yaw=" << yaw
	         << ", pitch=" << pitch
	         << ", roll=" << roll
	         << endl;

	    // Also note that for SLAM/multi-view application it is better to
	    // use reprojection error of corner points, because the noise in
	    // this relative pose is very non-Gaussian; see iSAM source code
	    // for suitable factors.
	  }


};

