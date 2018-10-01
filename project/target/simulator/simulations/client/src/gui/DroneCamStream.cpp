//
// Created by phil on 04/06/2018.
//

#include "DroneCamStream.h"
#include <iostream>
DroneCamStream::DroneCamStream(Simulation *sim, mav::Logger *logger):CamStream(sim) {
    this->logger = logger;
}

void DroneCamStream::draw(std::string winname){
	cv::Mat black = cv::Mat(480, 640 * 2, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat maskOut = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat camMat = getImage();

    if(logger->getLogSize()>0){
		mav::LogEntry entry = logger->peekLastEntry();
        cv::Mat mask,ycrcb;
    //if (entry.newVision){
			mav::Frame frame(camMat);
            cv::cvtColor(camMat.clone(), ycrcb, cv::COLOR_BGR2YCrCb);
			frame.drawPolygon(entry.bestGate,cv::Scalar(0,0,255));
			camMat = frame.getMat();
		//}
        cv::inRange(ycrcb,cv::Scalar(Y_MIN,Cr_MIN,Cb_MIN), cv::Scalar(Y_MAX,Cr_MAX,Cb_MAX),mask);
        cv::cvtColor(mask, maskOut, cv::COLOR_GRAY2BGR);
        //cv::imshow("ycrcb", ycrcb);


    }
    camMat.copyTo(black(cv::Rect(camMat.cols, 0, camMat.cols, camMat.rows)));
    maskOut.copyTo(black(cv::Rect(0, 0, camMat.cols, camMat.rows)));

    for (int i=640; i< 2*640; i+= 80){
        for (int j=0; j< 480; j+= 80){
            cv::circle(black,cv::Point(i,j),2,cv::Scalar(255,0,0));
        }
    }
    //maskOut.copyTo(black(cv::Rect(0, 0, camMat.cols, camMat.rows)));
    //annotate(black);
    //cv::imshow("mask", maskOut);
    cv::imshow(winname, black);
    cv::waitKey(30);
}

void DroneCamStream::annotate(cv::Mat &mat) {
//	static mav::LogEntry entry;
//	std::ostringstream title;
//    title << "Estimates:";
//
//    if(logger->getLogSize()>0) {
//        entry = logger->peekLastEntry();
//
//    }
//
//	title << " " << entry.timestamp / 1000 << ":" << entry.timestamp % 1000;
//	mav::MotionRead motionRead = entry.motionRead;
//	mav::Pose pose = entry.pose;
//	std::string accelAng = valueStr("Acc-Ang: ", std::vector<float>(
//		{ (float)motionRead.getA_x(), (float)motionRead.getA_y(), (float)motionRead.getA_z() }));
//	std::string velAng = valueStr("Vel-Ang: ", std::vector<float>({ (float)entry.motionRead.getP(), (float)entry.motionRead.getQ(), (float)entry.motionRead.getR() }));
//	std::string posAng = valueStr("Pos    : ", std::vector<float>(
//		{ (float)pose.getX(), (float)pose.getY(), (float)pose.getZ() }));
//	std::string posLin = valueStr("Orient : ", std::vector<float>(
//		{ (float)motionRead.getPhiDegree(), (float)motionRead.getThetaDegree(), (float)motionRead.getPsiDegree() }));
//
//	annotateValue(mat, cv::Point(5, 50), accelAng);
//	annotateValue(mat, cv::Point(5, 80), velAng);
//	annotateValue(mat, cv::Point(5, 110), posAng);
//	annotateValue(mat, cv::Point(5, 140), posLin);
//
//    std::ostringstream titleTruth;
//    titleTruth << "Ground Truth:" ;
//
//    std::string accelAngTruth = valueStr(oracle->getAngularAccelerations());
//    //std::string accelLinTruth = valueStr(oracle->getLinearAccelerations());
//    std::string velAngTruth = valueStr(oracle->getAngularVelocities());
//    //std::string velLinTruth = valueStr(oracle->getLinearVelocities());
//    std::string posAngTruth = valueStr(oracle->getPosition());
//    std::string posLinTruth = valueStr(oracle->getOrientation());
//
//    cv::putText(mat,
//                title.str(),
//                cv::Point(5, 20), // Coordinates
//                cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
//                0.8, // Scale. 2.0 = 2x bigger
//                cv::Scalar(255, 255, 255), // BGR Color
//                1, // Line Thickness
//                CV_AA); // Anti-alias
//
//
//
//    cv::putText(mat,
//                titleTruth.str(),
//                cv::Point(320, 20), // Coordinates
//                cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
//                0.8, // Scale. 2.0 = 2x bigger
//                cv::Scalar(255, 255, 255), // BGR Color
//                1, // Line Thickness
//                CV_AA); // Anti-alias
//
//    annotateValue(mat,cv::Point(320, 50),accelAngTruth);
//    annotateValue(mat,cv::Point(320, 80),velAngTruth);
//    annotateValue(mat,cv::Point(320, 110),posAngTruth);
//    annotateValue(mat,cv::Point(320, 140),posLinTruth);
    //annotateValue(mat,cv::Point(320, 170),accelLinTruth);
    //annotateValue(mat,cv::Point(320, 200),velLinTruth);

}