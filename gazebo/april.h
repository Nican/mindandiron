#pragma once

#include "util.h"
#include "robot.h"

#include "opencv2/opencv.hpp"
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag36h11.h"

struct QRTagFinder : public BaseGroundProcessor<cv::Mat, std::vector<AprilTags::TagDetection>>
{
	AprilTags::TagDetector tagDetector;

	QRTagFinder();

	//Run on a different thread
	virtual std::vector<AprilTags::TagDetection> AsyncronousUpdate(cv::Mat input) override;
};

namespace Robot {

class Kratos;

struct BaseStationTagInfo
{
	int mId;

	//Matrix that transforms a point from the center of the base station
	//to the tag location
	Eigen::Affine3d affine;

	BaseStationTagInfo(int id) : mId(id), affine(Eigen::Affine3d::Identity())
	{
	}
};

/*
	Detect the base station through april tags
*/
struct BaseStationDetector
{
	Kratos* mRobot;
	QRTagFinder finder;

	std::vector<BaseStationTagInfo> tags;

	double m_tagSize; // April tag side length in meters of square black frame
	double m_fx; // camera focal length in pixels
	double m_fy;
	double m_px; // camera principal point
	double m_py;

	double lastDetectionTime;
	Eigen::Affine3d mBaseTransformation;

	BaseStationDetector(Kratos* robot);

	void Update(cv::Mat input);

	Eigen::Affine3d DetectionToAffine(const AprilTags::TagDetection &detection);

};

};
