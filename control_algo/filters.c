#include "filters.h"
#include <math.h>

void complementary_filter_init(ComplementaryFilter *filter, float alpha) {
    filter->alpha = alpha;
    filter->filtered_value = 0.0f;
    filter->initialized = false;
}

float complementary_filter_update(ComplementaryFilter *filter, 
                                  float high_freq_value, 
                                  float low_freq_value) {
    // On first call, initialize with low frequency value
    if (!filter->initialized) {
        filter->filtered_value = low_freq_value;
        filter->initialized = true;
        return filter->filtered_value;
    }
    
    // Complementary filter: blend high-frequency and low-frequency signals
    // high_freq: fast response but drifts (gyroscope)
    // low_freq: stable but noisy (accelerometer/magnetometer)
    filter->filtered_value = filter->alpha * high_freq_value + 
                            (1.0f - filter->alpha) * low_freq_value;
    
    return filter->filtered_value;
}

void complementary_filter_reset(ComplementaryFilter *filter) {
    filter->filtered_value = 0.0f;
    filter->initialized = false;
}

float complementary_filter_get_value(ComplementaryFilter *filter) {
    return filter->filtered_value;
}

// Moving average filter for smoothing noisy signals
void moving_average_init(MovingAverageFilter *filter, uint8_t window_size) {
    filter->window_size = window_size;
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0.0f;
    
    // Clear buffer
    for (int i = 0; i < MAX_FILTER_WINDOW; i++) {
        filter->buffer[i] = 0.0f;
    }
}

float moving_average_update(MovingAverageFilter *filter, float new_value) {
    // Remove oldest value from sum
    filter->sum -= filter->buffer[filter->index];
    
    // Add new value
    filter->buffer[filter->index] = new_value;
    filter->sum += new_value;
    
    // Update index
    filter->index = (filter->index + 1) % filter->window_size;
    
    // Update count (until buffer is full)
    if (filter->count < filter->window_size) {
        filter->count++;
    }
    
    // Return average
    return filter->sum / filter->count;
}

void moving_average_reset(MovingAverageFilter *filter) {
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0.0f;
    
    for (int i = 0; i < MAX_FILTER_WINDOW; i++) {
        filter->buffer[i] = 0.0f;
    }
}

float moving_average_get_value(MovingAverageFilter *filter) {
    if (filter->count == 0) {
        return 0.0f;
    }
    return filter->sum / filter->count;
}