//
// Created by phil on 28/08/18.
//

#ifndef MAVPILOTBASE_LOG_H
#define MAVPILOTBASE_LOG_H

#include <string>
#include <MotionRead.h>
#include <Attitude.h>
#include <State.h>
#include <Control.h>
#include <Pose.h>
#include <thread>
#include <atomic>
#include <mutex>

namespace mav{
class Log {
public:
    Log(const std::string &filename);
    Log(const std::string &filename, const std::string &header);
    Log(const Log &old);
    ~Log();
    void append(MotionRead &motionRead,
                Attitude &motionCommand,
                State &state,
                Pose &pose,
                Polygon &bestGate,
                Control &control);
    void append(Frame &frame);
    void append(const std::string &line);
protected:
    std::atomic<int> n_entries;
    std::atomic<int> n_frames;
    std::thread writeThread;
    std::mutex writing;
    std::string filename;
};
}

#endif //MAVPILOTBASE_LOG_H
