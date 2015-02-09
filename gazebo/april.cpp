#include "april.h"


QRTagFinder::QRTagFinder() : tagDetector(AprilTags::tagCodes36h11)
{
}


//Run on a different thread
std::vector<AprilTags::TagDetection> QRTagFinder::AsyncronousUpdate(cv::Mat input)
{
	cv::Mat image_gray;

	cv::cvtColor(input, image_gray, CV_BGR2GRAY);		

	//cv::imwrite ("mali.png",input);
	//imshow("AprilDemo", image_gray);
	auto extracted = tagDetector.extractTags(image_gray);

	std::cout << "Thread QR codes. Found " << extracted.size() << " entries.\n";

	return extracted;
}
