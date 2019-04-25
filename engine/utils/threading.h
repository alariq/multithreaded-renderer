
namespace threading {



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
