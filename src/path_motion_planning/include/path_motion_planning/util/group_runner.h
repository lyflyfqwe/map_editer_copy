#ifndef PATH_MOTION_PLANNING_UTIL_GROUP_RUNNER_H
#define PATH_MOTION_PLANNING_UTIL_GROUP_RUNNER_H

#include <mutex>
#include <condition_variable>
#include <thread>
#include <memory>
#include <functional>

namespace path_motion_planning{
namespace util{
struct GroupRun{
    std::mutex mtx;
    std::condition_variable cv;
    bool is_paused{false}, is_stopped{false};
    std::function<void()> work;
    std::thread thread_run;
    int sleep_run;

    void setPauseState(bool state){
        {
            std::unique_lock<std::mutex> lock(mtx);
            is_paused = state;
        }
        if(!state) cv.notify_one();
    }

    void run(){
        thread_run = std::thread(std::bind(&GroupRun::threadWork, this));
    }

    void stop(){
        {
            std::unique_lock<std::mutex> lock(mtx);
            is_stopped = true;
        }
        cv.notify_all();
        if(thread_run.joinable()) thread_run.join();
    }
    
    inline void threadWork(){
        while(true){
            std::unique_lock<std::mutex> lock(mtx);
            if(is_stopped) break;
            if(is_paused){
                cv.wait(lock, [this]() {return !is_paused || is_stopped; });
                if(is_stopped) break;
            }
            lock.unlock();
            work();
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_run));
        }
    }
};
} //namespace util
} //namespace path_motion_planning

#endif