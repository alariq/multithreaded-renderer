#include "threading.h"
#include "SDL2/SDL.h"
#include "logging.h"
#include "profiler/profiler.h"

struct MutexWrapper {
    //MutexWrapper(SDL_mutex* m):m_(m) {}
    SDL_mutex* m_;
    void lock() {
        SCOPED_ZONE_N(Lock, 0)
        SDL_LockMutex(m_);
    }
    void unlock() { 
        SCOPED_ZONE_N(UnLock, 0)
        SDL_UnlockMutex(m_);
    }
    operator SDL_mutex* () { return m_; } 
    void operator=(SDL_mutex* m) { m_ = m; }
};

namespace threading {

    // current implementation: auto resets when Wait succeded

    class Event::PimplEvent {

        friend class Event;

        // everything is private on purpose
        bool raised;
        //TracyLockable(MutexWrapper, mutex_);
        MutexWrapper mutex_;
        SDL_cond* condition_;

        PimplEvent() {
            mutex_ = SDL_CreateMutex();
            condition_ = SDL_CreateCond();
            raised = false;
        }

        ~PimplEvent() {
            SDL_DestroyCond(condition_);
            SDL_DestroyMutex(mutex_);
        }
        
        int Signal() {

            int rv = 0;
            mutex_.lock();
            {
                raised = true;
                SCOPED_ZONE_N(Signal, flags)
                rv = SDL_CondSignal(condition_);
                if(rv)
                    log_error("SDL_CondSignal failed rv: %d : %s\n", rv, SDL_GetError());
            }
            mutex_.unlock();
            return rv;
        }

        int Wait() {
            int rv = 0;
            mutex_.lock();
            while (!raised)
            {
                SCOPED_ZONE_N(WaitCond, flags)
                rv = SDL_CondWait(condition_, mutex_);
                if(rv)
                    log_error("SDL_CondWait failed rv: %d : %s\n", rv, SDL_GetError());
            }
            raised = false;
            mutex_.unlock();
            return rv;
        }
    }; // PimplEvent


    Event::Event() {
        pev_ = new PimplEvent();
    };

    Event::~Event() {
        delete pev_;
    }

    int Event::Signal() {
        return pev_->Signal();
    }

    int Event::Wait() {
        return pev_->Wait();
    }


////////////////////////////////////////////////////////////////////////////////

    class Thread::PimplThread {
        friend class Thread;
        SDL_Thread* thread_;

        PimplThread(Thread::thread_func fn, const char* name, void* data) {
            thread_ = SDL_CreateThread(fn, name, data);   
        }

        ~PimplThread() {
            if(thread_)
                Wait();
        }

        int Wait() {
            int status;
            SDL_WaitThread(thread_, &status);
            thread_ = nullptr;
            return status;
        }
    };

    Thread::Thread(thread_func fn, const char* name, void* data) {
        pthread_ = new PimplThread(fn, name, data);
    }

    Thread::~Thread() {
        delete pthread_;
    }

    int Thread::Wait() { 
        return pthread_->Wait();
    }
};

