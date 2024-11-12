#ifndef __BASE_TIME_H_
#define __BASE_TIME_H_

#include "stdint.h"

extern uint32_t gMS_TimeCNT;

#define MS_TIMEBASE()     gMS_TimeCNT++
#define MS_GETTIME(void)  gMS_TimeCNT 
	
#define RUN_BLOCK(ms, BLOCK)           \
{                                               \
    static uint32_t last_exec_ms = 0;           \
    uint32_t cur_ms_tick = MS_GETTIME();   \
    if (cur_ms_tick >= (last_exec_ms + ms))     \
    {                                           \
        BLOCK;                                  \
        last_exec_ms = cur_ms_tick;             \
    }                                           \
}


#endif

