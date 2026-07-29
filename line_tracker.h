#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t error;
    int16_t last_error;
    int16_t correction;
    uint8_t active_bits;
    bool valid;
    bool has_seen_line;
} LineTracker;

void line_tracker_init(LineTracker *tracker);
void line_tracker_reset(LineTracker *tracker);
int16_t line_tracker_update(LineTracker *tracker, uint8_t gray_raw,
                            float kp, float kd, int16_t corr_limit);

#endif
