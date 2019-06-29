#include "SDL2/SDL.h"
#include <queue>


template< typename T>
class TSQueue {
    SDL_mutex* mutex_;
    std::queue<T> job_list_;

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

    int size() { 
        return job_list_.size();
    }
};

