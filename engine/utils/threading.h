#include <cstdint>

namespace threading {

class Thread {
    class PimplThread;
    PimplThread* pthread_;
    public:

    typedef int (*thread_func)(void*);

    Thread(thread_func fn, const char* name, void* data);
    ~Thread();

    int Wait();

};

class Event {
    class PimplEvent;
    PimplEvent* pev_;
public:

    Event();
    ~Event();

    int Signal();
    int Wait();

};

};
