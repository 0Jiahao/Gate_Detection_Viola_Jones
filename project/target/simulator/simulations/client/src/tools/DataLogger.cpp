#include "DataLogger.h"
#include <iostream>
#include <string>
#include <fstream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace mav {

	DataLogger::DataLogger(std::string path,std::string filename,bool logImages) {
		logfile = filename;
		logpath = path;
		this->logImages = logImages;
	}

void DataLogger::log(std::string logpath, std::string logfile, std::string line) {
	std::ofstream myfile;
	myfile.open(logpath + logfile, std::ios_base::app);
	myfile << line << EOL_CSV;
	myfile.close();

}

void DataLogger::append(std::vector<LogEntry> entries) {
	std::ofstream myfile;
	myfile.open(logpath + logfile, std::ios_base::app);
	for (int i = 0; i < entries.size(); i++) {


		/*
		long long timestamp;
		MotionRead motionRead;
		Attitude motionCommand;
		State state;
		Frame frame;
		Pose pose;
		Polygon bestGate;
		bool newVision;
		*/

		LogEntry entry = entries[i];
		//TODO extend this with missing stuff (Attitude set points, Gate polyong)
		Pose pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		//We log the pose only when it was recently detected.
		if (entry.newVision) {
			pose = entry.pose;
		}
		myfile 	<< entry.timestamp << ','
			   	<< entry.motionRead.getA_x() << ',' << entry.motionRead.getA_y() << ',' << entry.motionRead.getA_z() << ","
				<< entry.motionRead.getP() << "," << entry.motionRead.getQ() << "," << entry.motionRead.getR() << ","
				<< entry.motionRead.getPhi() << ',' << entry.motionRead.getTheta() << ',' << entry.motionRead.getPsi() << ","
				<< pose.getX() << "," << pose.getY() << "," << pose.getZ()<< ","
			 	<< entry.state.getX() << "," << entry.state.getY() << "," << entry.state.getZ() << ","
				<< entry.state.getV_z_B() << ","
				<< entry.state.getBx()  << ","<< entry.state.getBy() << ","<< entry.state.getBz()  <<EOL_CSV;
		if(logImages){
			std::ostringstream imgFileName;
			imgFileName << logpath << entry.timestamp << ".jpg";
			cv::imwrite(imgFileName.str(), entry.frame.getMat());
			entry.frame.drawPolygon(entry.bestGate,cv::Scalar(0,0,255));
			std::ostringstream imgFileName2;
			imgFileName2 << logpath << entry.timestamp << "_label.jpg";
			cv::imwrite(imgFileName2.str(), entry.frame.getMat());
			//std::cout<<"Writing frame.." << std::endl;
		}
	}
	myfile.close();

}

	DataLogger::DataLogger(DataLogger &old) {
		this->logImages = old.logImages;
		this->logpath = old.logpath;
		this->logfile = old.logfile;

	}

void DataLogger::log(std::string logpath, std::string logfile, const std::vector<std::string> &line) {
	std::ofstream myfile;
	myfile.open(logpath + logfile, std::ios_base::app);
	for (int i=0; i<line.size(); i++){
		myfile << line[i] << EOL_CSV;

	}
	myfile.close();
}

void DataLogger::log_buffered(std::string logpath, std::string logfile, std::string line) {
	static std::vector<std::string> buffer;
	buffer.push_back(line);
	if (buffer.size() > (int) 0.5*LOG_BUFFER_SIZE){
		mav::DataLogger::log(logpath, logfile, buffer);
		buffer.clear();
	}
}
}