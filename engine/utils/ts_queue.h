#ifndef TS_QUEUE_H
#define TS_QUEUE_H

#include "SDL2/SDL.h"
#include "threading.h"
#include <queue>


template< typename T>
class TSQueue {
    SDL_mutex* mutex_;
    std::queue<T> job_list_;
    threading::Event has_job_;
public:
    TSQueue() {
        mutex_ = SDL_CreateMutex();
    }

    ~TSQueue() {
        SDL_DestroyMutex(mutex_);
    }

    void push(T job) {
        SDL_LockMutex(mutex_);
        job_list_.push(job);
        SDL_UnlockMutex(mutex_);
        has_job_.Signal();
    }

    T pop() {
        T pjob = 0;
        SDL_LockMutex(mutex_);
        if(!job_list_.empty()) {
            pjob = job_list_.front();
            job_list_.pop();
        }
        SDL_UnlockMutex(mutex_);
        return pjob;
    }

    void wait_for_job() {
        has_job_.Wait();
    }

    int size() { 
        return (int)job_list_.size();
    }
};

#endif // TS_QUEUE_H
