#include "line_tracker.h"

#include "app_config.h"

static int16_t clamp_i16(int32_t value, int16_t limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return (int16_t)-limit;
    }
    return (int16_t)value;
}

void line_tracker_init(LineTracker *tracker)
{
    line_tracker_reset(tracker);
}

void line_tracker_reset(LineTracker *tracker)
{
    tracker->error = 0;
    tracker->last_error = 0;
    tracker->correction = 0;
    tracker->active_bits = 0U;
    tracker->valid = false;
    tracker->has_seen_line = false;
}

int16_t line_tracker_update(LineTracker *tracker, uint8_t gray_raw,
                            float kp, float kd, int16_t corr_limit)
{
    static const int16_t weights[8] = {
        -35, -25, -15, -5, 5, 15, 25, 35
    };
    int32_t weighted_sum = 0;
    int32_t count = 0;
    int16_t derivative;
    int32_t correction;
    uint8_t active_bits = gray_raw;
    uint8_t i;

    tracker->active_bits = active_bits;
    for (i = 0U; i < 8U; i++) {
        if ((active_bits & (uint8_t)(1U << i)) != 0U) {
            weighted_sum += weights[i];
            count++;
        }
    }

    if (count > 0) {
        tracker->valid = true;
        tracker->has_seen_line = true;
        tracker->error = (int16_t)(weighted_sum / count);
    } else {
        tracker->valid = false;
        if (tracker->has_seen_line) {
            tracker->error = tracker->last_error;
        } else {
            tracker->error = 0;
        }
    }

    derivative = (int16_t)(tracker->error - tracker->last_error);
    correction = (int32_t)((kp * (float)tracker->error) +
                           (kd * (float)derivative));
    correction *= LINE_CORRECTION_SIGN;

    tracker->correction = clamp_i16(correction, corr_limit);
    tracker->last_error = tracker->error;
    return tracker->correction;
}
