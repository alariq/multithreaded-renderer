#include "Time.h"
#include "engine/utils/timing.h"
#include <assert.h>

struct GameClock {
    uint64_t time_ = 0;
    //uint64_t last_time_ = 0;
    //bool b_is_paused_ = false;
};
GameClock g_game_clock;

uint64_t TimerGetGameTime() {
    return g_game_clock.time_;
}

void TimerGameTimeInit() {
    g_game_clock.time_ = 0;
    //g_game_clock.last_time_ = timing::gettickcount();
}

#if 0
void TimerGameTimeContinue() {
    assert(g_game_clock.b_is_paused_);
    g_game_clock.b_is_paused_ = false;
}

void TimerGameTimeResume() {
    assert(!g_game_clock.b_is_paused_);
    g_game_clock.b_is_paused_ = true;
}
#endif

void TimerGameTimeUpdate(uint64_t dtick) {
#if 0
    uint64_t cur_time = timing::gettickcount();
    if(!g_game_clock.b_is_paused_) {
        g_game_clock.time_ += timing::gettickcount() - g_game_clock.last_time_;
    }
    g_game_clock.last_time_ = cur_time;
#else
    g_game_clock.time_ += dtick;
#endif
}

