//
// Created by phil on 03/06/2018.
//

#include "CamStream.h"

CamStream::CamStream(Simulation *sim) {
    running = false;
    this->sim=sim;
}

void CamStream::start() {
    running = true;
    while (running) {
        draw("Stream");

    }
    cv::destroyWindow("Stream");

}

cv::Mat CamStream::getImage() {
    Image image = sim->getImage();
    cv::Mat mat;

    if(!image.empty()){
        mat = cv::Mat(image.getHeight(), image.getWidth(), CV_8UC4);
        memcpy(mat.data, image.getData().data(), image.getData().size() * sizeof(uint8_t));
    }else{
        mat = cv::Mat(10, 10, CV_8UC4);

    }
    cv::Mat mat_bgr;
    cv::cvtColor(mat, mat_bgr, cv::COLOR_RGB2BGR);
    return mat_bgr;
}

double CamStream::round3(float value) {
    return round(value * 1000.0) / 1000.0;
}

void CamStream::annotate(cv::Mat &mat) {
//    std::ostringstream title;
//    title << "Estimates:";
//    std::string accelAng = valueStr("Acc-Ang: ",sim->getAngularAccelerations());
//    std::string velAng = valueStr("Vel-Ang: ",autopilot->getAngularVelocities());
//    std::string posAng = valueStr("Pos    : ",autopilot->getPosition());
//    std::string posLin = valueStr("Orient : ",autopilot->getOrientation());
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
//    annotateValue(mat,cv::Point(5, 50),accelAng);
//    annotateValue(mat,cv::Point(5, 80),velAng);
//    annotateValue(mat,cv::Point(5, 110),posAng);
//    annotateValue(mat,cv::Point(5, 140),posLin);
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

void CamStream::stop() {
    running = false;
}

bool CamStream::isRunning() {
    return bool(running);
}

CamStream::CamStream(const CamStream &that) {
    this->sim = that.sim;
}

std::string CamStream::valueStr(std::vector<float> value) {
    std::ostringstream msg;
    msg  << this->round3(value[0]) << " | " << this->round3(value[1]) << " | " << this->round3(value[2])<< " |";
    return msg.str();
}

std::string CamStream::valueStr(std::string title, std::vector<float> value) {
    std::ostringstream msg;
    msg  << title << this->round3(value[0]) << " | " << this->round3(value[1]) << " | " << this->round3(value[2])<< " |";
    return msg.str();
}

void CamStream::annotateValue(cv::Mat &mat, cv::Point location, std::string text) {
    cv::putText(mat,
                text,
                location, // Coordinates
                cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                0.8, // Scale. 2.0 = 2x bigger
                cv::Scalar(255, 255, 255), // BGR Color
                1, // Line Thickness
                CV_AA); // Anti-alias
}

void CamStream::draw(std::string winname) {
    cv::Mat camMat = getImage();
    cv::Mat black = cv::Mat(camMat.rows, camMat.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    camMat.copyTo(black(cv::Rect(0, 0, camMat.cols, camMat.rows)));
    cv::imshow(winname, black);
    cv::waitKey(1);
}

CamStream::CamStream() = default;
