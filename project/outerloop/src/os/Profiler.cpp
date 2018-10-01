//
// Created by phil on 02/08/18.
//

#include "Profiler.h"
#include "Scheduler.h"

namespace mav{

int Profiler::nTimers = -1;
long long Profiler::timers[MAX_TIMERS] = ARRAY_INIT;
long long Profiler::periods[MAX_TIMERS] = ARRAY_INIT;
long long Profiler::iterations[MAX_TIMERS] = ARRAY_INIT;
float Profiler::means_time[MAX_TIMERS] = ARRAY_INIT;
float Profiler::stds_time[MAX_TIMERS] = ARRAY_INIT;
float Profiler::means_period[MAX_TIMERS] = ARRAY_INIT;
float Profiler::stds_period[MAX_TIMERS] = ARRAY_INIT;
std::string Profiler::timerNames[MAX_TIMERS] = STRING_ARRAY_INIT;
long long Profiler::startupTime = getCurrentTimeMillis();
std::mutex Profiler::guard;

int
Profiler::
createTimer(const std::string &name) {
    int idx=0;
    guard.lock();
    nTimers++;
    if(nTimers > MAX_TIMERS){
        return -1;
    }
    timerNames[nTimers] = name;
    timers[nTimers] = 0;
    means_time[nTimers] = 0.0f;
    stds_time[nTimers] = 1.0f;
    iterations[nTimers] = 0;
    idx = nTimers;
    guard.unlock();
    return idx;

}

void
Profiler::
startTimer(const int index) {

    if(index < 0 || index > MAX_TIMERS){return;}
    auto t = getStartupTimeMillis();
    guard.lock();
    auto t_last = timers[index];
    periods[index] = t - t_last;
    timers[index] = t;
    guard.unlock();
}

void
Profiler::
stopTimer(const int index) {
    if(index < 0 || index > MAX_TIMERS){return;}
    auto t2 = getStartupTimeMillis();
    guard.lock();
    auto t1 = timers[index];
    float n = (float)iterations[index];
    auto std_time = stds_time[index];
    auto mean_time = means_time[index];
    auto std_period = stds_period[index];
    auto mean_period = means_period[index];
    auto x_time = (float)(t2-t1);
    auto x_period = (float)periods[index];
    if(n > 0){
        means_time[index] += (x_time-mean_time)/n;
        means_period[index] += (x_period-mean_period)/n;

    }
    if (mean_time > 0 && n > 2){
        stds_time[index] = (n-2)/(n-1)*std_time+((x_time-mean_time)*(x_time-mean_time))/n;
        stds_period[index] = (n-2)/(n-1)*std_period+((x_period-mean_period)*(x_period-mean_period))/n;
    }

    iterations[index]++;
    guard.unlock();

}

void
Profiler::
print() {
    guard.lock();
    std::ostringstream header;
    header << "| Profiling::Execution time mean +- standard deviation [ms]| Call period mean +- standard deviation [ms] |" << std::endl;
    Comm::print(header.str());

    for(int i = 0; i <= nTimers; i++){
        std::ostringstream line;
        line << "|";
        line << timerNames[i] << ": " << means_time[i] << " +- " << sqrt(stds_time[i])
             << "|" << means_period[i] << " +- " << sqrt(stds_period[i]);
        line << " | " << std::endl;
        Comm::print(line.str());
    }


    guard.unlock();

}



long long
Profiler::
getStartupTimeMillis()
{
    guard.lock();
    long long t = startupTime;
    guard.unlock();
    return getCurrentTimeMillis() - t;
}
}

