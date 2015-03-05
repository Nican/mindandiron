#include "april.h"


QRTagFinder::QRTagFinder() : tagDetector(AprilTags::tagCodes36h11)
{
}


//Run on a different thread
std::vector<AprilTags::TagDetection> QRTagFinder::AsyncronousUpdate(cv::Mat input)
{
	cv::Mat image_gray;

	cv::cvtColor(input, image_gray, CV_BGR2GRAY);		

	cv::imwrite ("mali.png",input);
	imshow("AprilDemo", image_gray);
	auto extracted = tagDetector.extractTags(image_gray);

	//std::cout << "Thread QR codes. Found " << extracted.size() << " entries.\n";

	return extracted;
}


namespace Robot {

BaseStationDetector::BaseStationDetector(Kratos* robot) : mRobot(robot),
	m_tagSize(0.704),
	m_fx(1358),
	m_fy(1076),
	m_px(1920/2),
	m_py(1080/2)
{
	//Local normal of the tag plane
	Eigen::Vector3d normal(0.9936, 0.1121, 0); 
	Eigen::Vector3d forward(1, 0, 0);

	BaseStationTagInfo leftTag(0);
	leftTag.affine.linear() = Eigen::Quaterniond::FromTwoVectors(normal, forward).toRotationMatrix();
	leftTag.affine.translation() = Eigen::Vector3d(-1.0565, 0.5, 1.068);
	tags.push_back(leftTag);

	normal.y() *= -1;

	BaseStationTagInfo rightTag(13);
	rightTag.affine.linear() = Eigen::Quaterniond::FromTwoVectors(normal, forward).toRotationMatrix();
	rightTag.affine.translation() = Eigen::Vector3d(-1.0565, -0.5, 1.068);
	tags.push_back(rightTag);
}



void BaseStationDetector::Update(cv::Mat input)
{
	using namespace Eigen;

	if(finder.Update(input))
   	{
   		static double lastServoPosition = 0.0;

   		if(lastServoPosition == 0.0)
   			lastServoPosition = mRobot->mMotion.mAprilServo->GetPosition();

   		std::vector<AprilTags::TagDetection> detections = finder.lastProccessed;
   		Eigen::Vector2d solutionSum(0,0);
   		int solutionCount = 0;

		//std::cout << "Finished calcualting QR code. Found " << detections.size() << " entries.\n";

		//std::cout << "R: \t" << mRobot->mSensors.mTRS->GetPosition().transpose()  << " ("<< mRobot->mSensors.mTRS->GetOrientation() <<")\n";


		std::vector<Eigen::Affine3d> foundPositions;

		Affine3d bestAffine;

		for(auto& detection : detections)
		{
			for(auto& tag : this->tags)
			{
				if(tag.mId != detection.id)
					continue;

				double rotation = lastServoPosition;


				std::cout << "Found tag (" << tag.mId << "): \n";
				//std::cout << "\t Rotation of camera at: " << (rotation/M_PI*180.0) << "\n";

				Affine3d affine = DetectionToAffine(detection);

				//Oh boy, we are dealing with euler angles
				auto direction = affine.linear() * Vector3d(1,0,0);
				double euler = std::atan2(direction.z(), direction.x());


				auto direction2 = tag.affine.linear() * Vector3d(1,0,0);
				double euler2 = std::atan2(direction2.y(), direction2.x());

				std::cout << "Euler: " << (euler*180/M_PI) << "/" << (euler2*180/M_PI) << "\n";
				std::cout << "Sum: " << ((euler+euler2+rotation)*180/M_PI) << "\n";
				std::cout << "Input translation: " << affine.translation().head<2>().transpose()<< "\n";

				Vector2d fromRoot = (Rotation2Dd(rotation) * affine.translation().head<2>()) + Vector2d(0.6, 0);
				std::cout << "From root: " << fromRoot.transpose()<< "\n";

				Vector2d fix = Rotation2Dd(euler+euler2+rotation) * tag.affine.translation().head<2>();
				std::cout << "Fix: " << fix.transpose()<< "\n";
				std::cout << "\tFix2: " << Vector2d(fromRoot+fix).transpose()<< "\n";

				solutionSum = solutionSum + Vector2d(fromRoot+fix);
				solutionCount++;

				/*
				Vector2d signLocation = Rotation2Dd(rotation) * affine.translation().head<2>();
				std::cout << "Relative: " << signLocation.transpose()<< "\n";

				Vector2d adjusted = signLocation - tag.affine.translation().head<2>();
				std::cout << "Adjusted: " << adjusted.transpose()<< "\n";
				adjusted = adjusted + Vector2d(0.6, 0);
				std::cout << "Adjusted2: " << adjusted.transpose()<< "\n";

				*/
				
				//std::cout << "Read: \n" << affine.matrix() << "\n\n";
				//std::cout << "Euler: " << (affine.linear().eulerAngles(1, 2, 0) / M_PI * 180.0).transpose() << "\n";
				//std::cout << "Euler: " << (euler*180/M_PI) << "\n";


				/*
				auto affine2 =  tag.affine * affine;

				std::cout << "ToBase: \n" << affine2.matrix() << "\n";
				std::cout << "Euler: " << (affine2.linear().eulerAngles(2, 1, 0) / M_PI * 180.0).transpose() << "\n";

				Affine3d cameraTransformation = Affine3d::Identity();
				cameraTransformation.linear() = AngleAxisd(rotation, Vector3d::UnitZ()).toRotationMatrix();
				cameraTransformation.translation() = cameraTransformation.linear() * Vector3d(-0.6, 0.0, -0.6);

				std::cout << "Rotation: \n" << cameraTransformation.matrix() << "\n";

				auto affine3 = cameraTransformation * affine2;
				std::cout << "Final: \n" << affine3.matrix() << "\n";
				std::cout << "Euler: " << (affine3.linear().eulerAngles(2, 1, 0) / M_PI * 180.0).transpose() << "\n";
*/

				//auto q1 = affine.linear() * Eigen::Vector3d(1,0,0);
				//auto q2 = affine.linear() * Eigen::Vector3d(0,0,1);
				//std::cout << "\t Input:\t" << affine.translation().transpose() << " ("<< std::atan2(q1.y(), q1.x()) <<", "<< std::atan2(q2.y(), q2.x()) <<")\n";
/*
				Affine3d cameraTransformation = Affine3d::Identity();
				cameraTransformation.linear() = AngleAxisd(rotation, Vector3d::UnitZ()).toRotationMatrix();
				cameraTransformation.translation() = Vector3d(-0.6, 0.0, -0.6);

				std::cout << "Camera: \n" << cameraTransformation.matrix() << "\n\n";


				affine = affine * cameraTransformation;
				std::cout << "Axle: \n" << affine.matrix() << "\n";
				std::cout << (affine.linear().eulerAngles(2, 1, 0) / M_PI * 180.0).transpose() << "\n";

				std::cout << "Tag: \n" << tag.affine.matrix() << "\n\n";


				auto affine2 =  tag.affine * affine;

				std::cout << "Final: \n" << affine2.inverse().matrix() << "\n\n";
				*/


				/*
				Affine3d cameraTransformation = Affine3d::Identity();
				cameraTransformation.linear() = AngleAxisd(rotation, Vector3d::UnitZ()).toRotationMatrix();
				cameraTransformation.translation() = Vector3d(-0.6, 0.0, -0.6);

				Affine3d affine = DetectionToAffine(detection);
				std::cout << "\t Input:\t" << affine.translation().transpose() << "\n";
				//affine.linear().setIdentity();
				affine = affine * cameraTransformation;

				std::cout << "\t Wheel:\t" << affine.translation().transpose() << "\n";
				


				auto affine2 =  tag.affine.inverse() * affine;
				std::cout << "\t Plataform\t: " << affine2.translation().transpose() << "("<< affine2.translation().norm() <<")" "\n";
				//std::cout << "\t\t" << Quaterniond(affine2.linear()).vec().transpose() << "\n";

				if(bestAffine.translation().norm() < 0.0001 || affine2.translation().norm() < bestAffine.translation().norm())
				{
					bestAffine = affine2;
					break;
				}
				

			
				for(double fx = 800; fx <= 1400; fx+= 1.0)
				{
					for(double fy = 800; fy <= 1400; fy+= 1.0)
					{
						this->m_fx = fx;
						this->m_fy = fy;

						Affine3d affineX = DetectionToAffine(detection);
						Vector2d xytranslation = affineX.translation().head<2>();
						//affine.linear().setIdentity();
						//affine = cameraTransformation * affine;
						//auto affine2 =  tag.affine.inverse() * affine;

						if(std::abs(xytranslation.x() - 6.6) > 0.1)
							continue;

						if(std::abs(xytranslation.y() - 0.5) > 0.1)
							continue;

						auto direction = affineX.linear() * Vector3d(1,0,0);
						double euler = std::atan2(direction.z(), direction.x());


						std::cout << fx << "\t" << fy << "\t" << xytranslation.x() << "\t" << xytranslation.y() << "\t" << euler << "\t" << xytranslation.norm() << "\n";
					}
				}

				
				exit(0);
				*/
				

				//foundPositions.push_back(affine2);
			}
		}

		mBaseTransformation = bestAffine;
		lastServoPosition = mRobot->mMotion.mAprilServo->GetPosition();
		mSolution = solutionSum / solutionCount;
		

		//assert(0);


    }	

    //std::cout << "\n\n";
}

Eigen::Affine3d BaseStationDetector::DetectionToAffine(const AprilTags::TagDetection &detection)
{
	Eigen::Vector3d translation;
	Eigen::Matrix3d rotation;
	detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py, translation, rotation);

	/*
	Eigen::Matrix3d F;
	F <<
		1, 0,  0,
		0,  0,  -1,
		0,  -1,  0;

	Eigen::Matrix3d F2;
	F2 <<
		1, 0,  0,
		0,  0,  1,
		0,  1,  0;

	Eigen::Matrix3d fixed_rot = F2*rotation*F;

	auto ea = fixed_rot.eulerAngles(2, 1, 0) / M_PI * 180.0; 
	std::cout << "Euler: " << ea.transpose() << "\n";
	
	
	
	Eigen::Matrix3d fixed_rot2 = fixed_rot; // * Eigen::Quaterniond::FromTwoVectors( fixed_rot * Eigen::Vector3d(0,0,1), Eigen::Vector3d(0,0,1)).toRotationMatrix();
	*/
	Eigen::Affine3d affine;
	affine.translation() = translation;
	affine.linear() = rotation;

	return affine;	
}


}