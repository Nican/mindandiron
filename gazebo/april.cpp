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

	//std::cout << "Thread QR codes. Found " << extracted.size() << " entries.\n";

	return extracted;
}


namespace Robot {


void BaseStationDetector::Update(cv::Mat input)
{
	using namespace Eigen;

	if(finder.Update(input))
   	{
   		std::vector<AprilTags::TagDetection> detections = finder.lastProccessed;

		//std::cout << "Finished calcualting QR code. Found " << detections.size() << " entries.\n";

		std::vector<Eigen::Affine3d> foundPositions;

		for(auto& detection : detections)
		{
			for(auto& tag : this->tags)
			{
				if(tag.mId != detection.id)
					continue;

				double rotation = mRobot->mMotion.mAprilServo->GetPosition();
				std::cout << "Found tag (" << tag.mId << "): \n";
				std::cout << "\t Rotation of camera at: " << rotation << "\n";

				Affine3d cameraTransformation;
				cameraTransformation.linear() = AngleAxisd(rotation, Vector3d::UnitZ()).toRotationMatrix();
				cameraTransformation.translation() = Vector3d(0.6, 0.0, 0.6);

				Affine3d affine = DetectionToAffine(detection);
				std::cout << "\t Input:\t" << affine.translation().transpose() << "\n";
				affine.linear().setIdentity();
				affine = cameraTransformation * affine;

				std::cout << "\t Wheel:\t" << affine.translation().transpose() << "\n";
				auto affine2 =  tag.affine.inverse() * affine;
				std::cout << "\t Plataform\t: " << affine2.translation().transpose() << "("<< affine2.translation().norm() <<")" "\n";


				/*
				for(double fx = 1000; fx <= 1200; fx+= 1.0)
				{
					for(double fy = 1000; fy <= 1200; fy+= 1.0)
					{
						this->m_fx = fx;
						this->m_fy = fy;

						Affine3d affine = DetectionToAffine(detection);
						affine.linear().setIdentity();
						affine = cameraTransformation * affine;
						auto affine2 =  tag.affine.inverse() * affine;

						std::cout << fx << "\t" << fy << "\t" << affine2.translation().norm() << "\n";
					}
				}
				*/

				//foundPositions.push_back(affine2);
			}
		}

		//assert(0);


    }	
}

}