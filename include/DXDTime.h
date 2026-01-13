#ifndef DXD_TIME_H
#define DXD_TIME_H

#include <cstdlib>
#include <sys/time.h>
#include <mutex>


extern bool diffTimes(timeval& ret, const timeval &tLater, const timeval &tEarlier);

class CStepTime
{
    static int timeVal;

public:

    static void makeStart()
    {
        timeVal = 0;
    }

    static int getTime()
    {
        return timeVal;
    }

    static void stepTime()
    {
        timeVal++;
    }
};

class CStopWatch
{
    timeval timeStart;          // 本次开始时间
    long long elapsedAccum = 0; // 累计时间（微秒）
    bool running = false;       // 是否正在计时
    long int timeBound = 0;     // 时间上限（秒）
    mutable std::mutex mtx;     // 互斥锁，保证多线程安全

public:
    CStopWatch() {}
    ~CStopWatch() {}

    // 开始或继续计时
    bool markStartTime()
    {
        // std::lock_guard<std::mutex> lock(mtx);
        if (!running) {
            gettimeofday(&timeStart, nullptr);
            running = true;
        }
        return true;
    }

    // 暂停计时
    bool markStopTime()
    {
        // std::lock_guard<std::mutex> lock(mtx);
        if (running) {
            timeval now, r;
            gettimeofday(&now, nullptr);
            diffTimes(r, now, timeStart);
            elapsedAccum += r.tv_sec * 1000000LL + r.tv_usec;
            running = false;
        }
        return true;
    }

    void reset()
    {
        // std::lock_guard<std::mutex> lock(mtx);
        elapsedAccum = 0;
        running = false;
    }

    void setTimeBound(long int seconds)
    {
        // std::lock_guard<std::mutex> lock(mtx);
        timeBound = seconds;
    }

    long int getTimeBound()
    {
        // std::lock_guard<std::mutex> lock(mtx);
        return timeBound;
    }

    bool timeBoundBroken()
    {
        // std::lock_guard<std::mutex> lock(mtx);
        return getElapsedTime() > (double)timeBound;
    }

    // 获取累计秒数
    double getElapsedTime() const
    {
        // std::lock_guard<std::mutex> lock(mtx);
        long long total = elapsedAccum;
        if (running) {
            timeval now, r;
            gettimeofday(&now, nullptr);
            diffTimes(r, now, timeStart);
            total += r.tv_sec * 1000000LL + r.tv_usec;
        }
        return (double)total / 1e6;
    }
};

#endif
