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

typedef struct {
    float angle_deg[3];
    uint32_t time_ms[3];
} Q3OpenLoopParams;

void ball_balance_init(void);
void ball_balance_start_sequence(void);
void ball_balance_hold_target(float target_mm);
void ball_balance_hold_q4(float target_mm);
void ball_balance_hold_q6(float target_mm);
void ball_balance_hold_q7(float target_mm);
bool ball_balance_start_q3(uint32_t now_ms);
bool ball_balance_set_q3_open_loop_params(const Q3OpenLoopParams *params);
bool ball_balance_set_q67_target_biases(float q6_straight_mm,
                                        float q6_curve_mm,
                                        float q7_straight_mm,
                                        float q7_curve_mm);
void ball_balance_set_feedforward(float pipe_angle_deg);
void ball_balance_set_curve_feedforward(float pipe_angle_deg);
void ball_balance_set_feedback_scale(float scale);
void ball_balance_set_initial_feedforward(float pipe_angle_deg);
void ball_balance_stop(void);
void ball_balance_update(uint32_t now_ms, bool sample_ok,
                         const K230BallSample *sample);
BallBalanceState ball_balance_get_state(void);
uint8_t ball_balance_get_q3_report_phase(void);

#endif
