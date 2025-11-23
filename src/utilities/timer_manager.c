/**
 * @file    timer_manager.c
 * @brief   Timer management implementation
 * @details Implements timer lifecycle management using Raspberry Pi Pico
 *          hardware timer peripherals with microsecond precision
 */

#include "timer_manager.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include <stddef.h>

#define MAX_TIMERS 10

static Timer timers[MAX_TIMERS];
static int timer_count = 0;

void timer_manager_init(void)
{
    for (int i = 0; i < MAX_TIMERS; i++)
    {
        timers[i].active = false;
    }

    timer_count = 0;
}

Timer *timer_start(uint32_t duration_ms)
{
    if (timer_count >= MAX_TIMERS)
    {
        return NULL;
    }

    Timer *timer = &timers[timer_count++];
    timer->start_time = time_us_64();
    timer->duration_us = (uint64_t)duration_ms * 1000;
    timer->active = true;

    return timer;
}

bool timer_is_expired(Timer *timer)
{
    if (timer == NULL || !timer->active)
    {
        return true;
    }

    uint64_t elapsed = time_us_64() - timer->start_time;

    if (elapsed >= timer->duration_us)
    {
        timer->active = false;
        return true;
    }

    return false;
}

uint32_t timer_get_remaining(Timer *timer)
{
    if (timer == NULL || !timer->active)
    {
        return 0;
    }

    uint64_t elapsed = time_us_64() - timer->start_time;

    if (elapsed >= timer->duration_us)
    {
        timer->active = false;
        return 0;
    }

    return (uint32_t)((timer->duration_us - elapsed) / 1000);
}

void timer_stop(Timer *timer)
{
    if (timer != NULL)
    {
        timer->active = false;
    }
}

void timer_wait_ms(uint32_t duration_ms)
{
    uint64_t start_time = time_us_64();
    uint64_t duration_us = (uint64_t)duration_ms * 1000;

    while ((time_us_64() - start_time) < duration_us)
    {
        tight_loop_contents();
    }
}
