//
// Created by phil on 02/08/18.
// Class to do speed measurements
/**
 * Example Usage:
 * #ifdef PROFILING
 * int timer0 = createTimer("TestTimer");
 * #endif PROFILING
 * ...
 * void dummy(){
 *  #ifdef PROFILING
 *  startTimer(timer0);
 *  #endif PROFILING
 *
 *  normal code that happens in this function
 *
 *  #ifdef PROFILING
 *  stopTimer(timer0);
 *  #endif PROFILING
 * }
 *
 */

#ifndef PROFILER_H
#define PROFILER_H

#include <string>
#include <mutex>


#define PRINT_WAIT_MS 5000 //Time interval to print profiling report
#define MAX_TIMERS 50 //Max numbers of timers possible to create
//Some init stuff:
#define ARRAY_INIT {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,}
#define STRING_ARRAY_INIT {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0",}


namespace mav{
class Profiler {
protected:
    static long long timers[MAX_TIMERS];
    static long long periods[MAX_TIMERS];
    static long long iterations[MAX_TIMERS];
    static float means_time[MAX_TIMERS];
    static float stds_time[MAX_TIMERS];
    static float means_period[MAX_TIMERS];
    static float stds_period[MAX_TIMERS];
    static std::string timerNames[MAX_TIMERS];

    static int nTimers;
    static long long startupTime;

    static long long getStartupTimeMillis();
    static std::mutex guard;
public:
    /**
     * Create a new timer;
     * @param name string that is printed with the timer value in the report
     * @return index of the created timer to be used with the other functions
     */
    static int createTimer(const std::string &name);

    /**
     * Start a timer
     * @param index of timer to be used. Index is returned by createTimer
     */
    static void startTimer(int index);

    /**
     * Stop a timer. Will stop the respective timer and update the mean and standard deviation estimate.
     * @param index of timer to be used. Index is returned by createTimer
     */
    static void stopTimer(int index);

    /**
     * Prints estimates for all timers.
     */
    static void print();
};
}

#endif //OUTERLOOP_TEST_PROFILER_H
