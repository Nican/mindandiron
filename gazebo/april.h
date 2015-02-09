#pragma once

#include "util.h"

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