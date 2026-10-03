// Bare-metal time sources for the bundled libmicroros.a.
//
// The library references clock_gettime()/gettimeofday(); on STM32 these come from
// the SysTick millisecond counter driven by HAL_IncTick().

#include <stdint.h>
#include <time.h>
#include <sys/time.h>

#include "main.h"

int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    (void)clk_id;
    if (tp == NULL)
    {
        return -1;
    }
    uint32_t ms = HAL_GetTick();
    tp->tv_sec = (time_t)(ms / 1000u);
    tp->tv_nsec = (long)((ms % 1000u) * 1000000u);
    return 0;
}

int gettimeofday(struct timeval *tv, void *tz)
{
    (void)tz;
    if (tv == NULL)
    {
        return -1;
    }
    uint32_t ms = HAL_GetTick();
    tv->tv_sec = (time_t)(ms / 1000u);
    tv->tv_usec = (suseconds_t)((ms % 1000u) * 1000u);
    return 0;
}
