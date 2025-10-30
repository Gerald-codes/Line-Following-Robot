/**
 * filters.h
 * Signal filtering algorithms
 */

#ifndef FILTERS_H
#define FILTERS_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Maximum window size for moving average
#define MAX_FILTER_WINDOW 20

// Complementary filter structure
// Used for sensor fusion (e.g., gyro + accelerometer)
typedef struct {
    float alpha;              // Filter coefficient (0-1)
                             // High alpha = trust high_freq more
                             // Low alpha = trust low_freq more
                             // Typical: 0.96-0.98
    float filtered_value;     // Current filtered output
    bool initialized;         // First update flag
} ComplementaryFilter;

// Moving average filter structure
// Used for smoothing noisy signals
typedef struct {
    float buffer[MAX_FILTER_WINDOW];  // Circular buffer
    uint8_t window_size;              // Number of samples to average
    uint8_t index;                    // Current buffer position
    uint8_t count;                    // Number of samples in buffer
    float sum;                        // Running sum for efficiency
} MovingAverageFilter;

// Complementary filter functions
void complementary_filter_init(ComplementaryFilter *filter, float alpha);
float complementary_filter_update(ComplementaryFilter *filter, 
                                  float high_freq_value, 
                                  float low_freq_value);
void complementary_filter_reset(ComplementaryFilter *filter);
float complementary_filter_get_value(ComplementaryFilter *filter);

// Moving average filter functions
void moving_average_init(MovingAverageFilter *filter, uint8_t window_size);
float moving_average_update(MovingAverageFilter *filter, float new_value);
void moving_average_reset(MovingAverageFilter *filter);
float moving_average_get_value(MovingAverageFilter *filter);

#endif // FILTERS_H