#ifndef MOTION_STATE_H
#define MOTION_STATE_H

#include <stdint.h>

typedef struct {
    int32_t left_speed_cps;
    int32_t right_speed_cps;
    int32_t speed_cps;
    int32_t accel_cps2;
    int32_t prev_speed_cps;
    int32_t filtered_accel_cps2;
} MotionState;

static inline int32_t motion_abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static inline void motion_state_init(MotionState *state)
{
    state->left_speed_cps = 0;
    state->right_speed_cps = 0;
    state->speed_cps = 0;
    state->accel_cps2 = 0;
    state->prev_speed_cps = 0;
    state->filtered_accel_cps2 = 0;
}

static inline void motion_state_update(MotionState *state,
                                       int32_t left_delta,
                                       int32_t right_delta,
                                       uint32_t period_ms)
{
    int32_t raw_accel;
    int32_t left_abs = motion_abs_i32(left_delta);
    int32_t right_abs = motion_abs_i32(right_delta);

    if (period_ms == 0U) {
        return;
    }

    state->left_speed_cps = (left_abs * 1000) / (int32_t)period_ms;
    state->right_speed_cps = (right_abs * 1000) / (int32_t)period_ms;
    state->speed_cps = (state->left_speed_cps + state->right_speed_cps) / 2;

    raw_accel = ((state->speed_cps - state->prev_speed_cps) * 1000) /
                (int32_t)period_ms;
    state->filtered_accel_cps2 =
        ((state->filtered_accel_cps2 * 3) + raw_accel) / 4;
    state->accel_cps2 = state->filtered_accel_cps2;
    state->prev_speed_cps = state->speed_cps;
}

#endif
