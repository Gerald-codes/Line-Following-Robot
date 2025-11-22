#ifndef TIMER_MANAGER_H
#define TIMER_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint64_t start_time;
    uint64_t duration_us;
    bool active;
} Timer;

/**
 * Initialize timer manager (sets up hardware timer)
 */
void timer_manager_init(void);

/**
 * Create and start a new timer
 * @param duration_ms Duration in milliseconds
 * @return Timer handle (pointer to Timer structure)
 */
Timer* timer_start(uint32_t duration_ms);

/**
 * Check if timer has expired
 * @param timer Timer handle
 * @return true if timer has expired, false otherwise
 */
bool timer_is_expired(Timer* timer);

/**
 * Get remaining time on timer
 * @param timer Timer handle
 * @return Remaining time in milliseconds, 0 if expired
 */
uint32_t timer_get_remaining(Timer* timer);

/**
 * Stop and free a timer
 * @param timer Timer handle
 */
void timer_stop(Timer* timer);

/**
 * Poll-based wait until timer expires
 * (Non-blocking alternative to sleep_ms)
 * @param duration_ms Duration in milliseconds
 */
void timer_wait_ms(uint32_t duration_ms);

#endif
