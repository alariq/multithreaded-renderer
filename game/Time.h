#pragma once

#include <stdint.h>

uint64_t TimerGetGameTime();
void TimerGameTimeInit();
//void TimerGameTimeContinue(); 
//void TimerGameTimeResume();
void TimerGameTimeUpdate(uint64_t dtick);
