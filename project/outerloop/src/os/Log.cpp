//
// Created by phil on 28/08/18.
//

#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <Config.h>
#include <Comm.h>
#include <filter.h>
#include "Log.h"
#include "Scheduler.h"

#define EOL_CSV "\n"
namespace mav{
void
Log::
append(mav::MotionRead &motionRead, mav::Attitude &motionCommand, mav::State &state, mav::Pose &pose,
       mav::Polygon &bestGate,mav::Control &control)
{
    std::ostringstream lineBuilder;
    lineBuilder << Scheduler::getStartupTimeMillis()<<','
         <<  motionRead.getA_x() << ','
         << motionRead.getA_y() << ','
         << motionRead.getA_z() << ','
         << motionRead.getP() << ','
         << motionRead.getQ() << ','
         << motionRead.getR() << ','
         << motionRead.getPhi() << ','
         << motionRead.getTheta() << ','
         << motionRead.getPsi() << ','
         << pose.getX() << ','
         << pose.getY() << ','
         << pose.getZ()<< ','
         << state.getX() << ','
         << state.getY() << ','
         << state.getZ() << ','
         << state.getV_z_B() << ','
         << state.getBx()  << ','
         << state.getBy() << ','
         << state.getBz() << ','
         << bestGate.get_vertex(0).col << ','
         << bestGate.get_vertex(0).row << ','
         << bestGate.get_vertex(1).col << ','
         << bestGate.get_vertex(1).row << ','
         << bestGate.get_vertex(2).col << ','
         << bestGate.get_vertex(2).row << ','
         << bestGate.get_vertex(3).col << ','
         << bestGate.get_vertex(3).row << ','
         << motionCommand.roll << ','
         << motionCommand.pitch << ','
         << motionCommand.yaw << ','
         << motionCommand.altitude<< ','
                <<control.getArcTargetHeading()<<','
            <<control.getHighLevelGuidanceState() << ','
            << control.getLowLevelGuidanceState() << ','
            << control.getGateNumber() << ','
            << filteredX << ','
            << filteredY;

    std::string line = lineBuilder.str();
    append(line);
}

void
Log::
append(const std::string &line)
{

    if(n_entries == MAX_LOG){
        Comm::print("Log dangerously big. Not Logging.");
    }

    if(n_entries >= MAX_LOG){
        return;
    }

    std::ofstream myfile;
    myfile.open(filename, std::ios_base::app);
    myfile << line << EOL_CSV;
    myfile.close();
    n_entries++;
}

Log::
Log(const std::string &filename):
        n_entries(0),
        n_frames(0)
{
    this->filename = filename;
    std::ofstream myfile;
    myfile.open(filename);
    myfile  << "timestamp,"
            << "A_x,A_y,A_z,"
            << "P,Q,R,"
            << "Phi,Theta,Psi,"
            << "poseX,poseY,poseZ,"
            << "state_X,state_Y,state_Z,state_VzB,state_Bx,state_By,state_Bz,"
            << "p1_x,p1_y,p2_x,p2_y,p3_x,p3_y,p4_x,p4_y,"
            << "roll,pitch,yaw,alt," << "targetHeading,"
            << "highLevelState,lowLevelState,gateNumber,"
            << "filtered_x,filtered_y"
           << EOL_CSV;
    myfile.close();
}

void
Log::
append(Frame &frame) {

    if(n_frames == MAX_LOG_FRAMES){
        Comm::print("Frame log dangerously big. Not Logging.");
    }

    if(n_frames >= MAX_LOG_FRAMES){
        return;
    }
    cv::Mat mat;
    cv::cvtColor(frame.getMat(),mat,CV_YUV2BGR_YUYV);
    cv::resize(mat,mat,cv::Size(LOG_FRAME_W,LOG_FRAME_H));
    std::stringstream filename;
    filename << this->filename << "_" <<std::to_string(n_frames);
    filename << ".bmp";
    cv::imwrite(filename.str(),mat);
    n_frames++;


}

Log::
Log(const Log &old) :
        n_entries(0),
        n_frames(0),
        filename(old.filename)
{
}

Log::
~Log()
{

    if(writeThread.joinable()){
        writeThread.join();
    }
}

Log::
Log(const std::string &filename, const std::string &header):
    n_entries(0),
    n_frames(0)
{
    this->filename = filename;
    std::ofstream myfile;
    myfile.open(filename);
    myfile  << header
    << EOL_CSV;
    myfile.close();
    Comm::print("Log file created.");
}
}
