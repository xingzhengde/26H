#ifndef BALL_BALANCE_H
#define BALL_BALANCE_H

#include "k230_ball.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BALL_MODE_IDLE = 0,
    BALL_MODE_HOLD_TARGET,
    BALL_MODE_SEQ_PLUS,
    BALL_MODE_SEQ_ZERO,
    BALL_MODE_SEQ_MINUS,
    BALL_MODE_DONE,
    BALL_MODE_ERROR
} BallMode;

typedef struct {
    BallMode mode;
    float target_mm;
    float position_mm;
    float velocity_mm_s;
    float feedforward_deg;
    float feedback_deg;
    float theta_deg;
    uint16_t settle_count;
    bool sample_ok;
} BallBalanceState;

void ball_balance_init(void);
void ball_balance_start_sequence(void);
void ball_balance_hold_target(float target_mm);
void ball_balance_hold_q4(float target_mm);
void ball_balance_set_feedforward(float pipe_angle_deg);
void ball_balance_stop(void);
void ball_balance_update(uint32_t now_ms, bool sample_ok,
                         const K230BallSample *sample);
BallBalanceState ball_balance_get_state(void);

#endif
