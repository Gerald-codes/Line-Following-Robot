/**
 * @file    timer_manager.h
 * @brief   Timer management interface
 * @details Provides timer creation, expiration checking, and non-blocking
 *          wait functionality for embedded applications using Raspberry Pi
 *          Pico hardware timers
 */

#ifndef TIMER_MANAGER_H
#define TIMER_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint64_t start_time;
    uint64_t duration_us;
    bool active;
} Timer;

/**
 * @brief Initialize timer manager
 * @details Sets up timer tracking structures
 */
void timer_manager_init(void);

/**
 * @brief Create and start a new timer
 * @param duration_ms Duration in milliseconds
 * @return Pointer to Timer structure, NULL if no timers available
 */
Timer *timer_start(uint32_t duration_ms);

/**
 * @brief Check if timer has expired
 * @param timer Pointer to Timer structure
 * @return true if timer has expired, false otherwise
 */
bool timer_is_expired(Timer *timer);

/**
 * @brief Get remaining time on timer
 * @param timer Pointer to Timer structure
 * @return Remaining time in milliseconds, 0 if expired
 */
uint32_t timer_get_remaining(Timer *timer);

/**
 * @brief Stop and deactivate a timer
 * @param timer Pointer to Timer structure
 */
void timer_stop(Timer *timer);

/**
 * @brief Non-blocking wait until duration expires
 * @details Poll-based wait, alternative to sleep_ms
 * @param duration_ms Duration in milliseconds
 */
void timer_wait_ms(uint32_t duration_ms);

#endif /* TIMER_MANAGER_H */
