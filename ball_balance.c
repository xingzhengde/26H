#include "ball_balance.h"

#include "app_config.h"
#include "stepper_arm.h"

static BallBalanceState g_state;
static uint32_t g_last_ms;
static float g_last_pos_mm;
static float g_speed_integral;
static float g_last_pos_error;

static float clamp_f32_ball(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float abs_f32(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void reset_loop_state(void)
{
    g_last_ms = 0U;
    g_last_pos_mm = 0.0f;
    g_speed_integral = 0.0f;
    g_last_pos_error = 0.0f;
    g_state.settle_count = 0U;
    g_state.theta_deg = 0.0f;
}

void ball_balance_init(void)
{
    g_state.mode = BALL_MODE_IDLE;
    g_state.target_mm = 0.0f;
    g_state.position_mm = 0.0f;
    g_state.velocity_mm_s = 0.0f;
    g_state.theta_deg = 0.0f;
    g_state.sample_ok = false;
    reset_loop_state();
}

void ball_balance_start_sequence(void)
{
    g_state.mode = BALL_MODE_SEQ_PLUS;
    g_state.target_mm = BALL_TARGET_PLUS_MM;
    reset_loop_state();
}

void ball_balance_hold_target(float target_mm)
{
    g_state.mode = BALL_MODE_HOLD_TARGET;
    g_state.target_mm = target_mm;
    reset_loop_state();
}

void ball_balance_stop(void)
{
    g_state.mode = BALL_MODE_IDLE;
    stepper_arm_set_pipe_angle(0.0f, BALL_THETA_RATE_DEG);
    reset_loop_state();
}

static void update_sequence_stage(void)
{
    if ((abs_f32(g_state.target_mm - g_state.position_mm) <=
            BALL_SETTLE_ERR_MM) &&
        (abs_f32(g_state.velocity_mm_s) <= BALL_SETTLE_SPEED_MM_S)) {
        if (g_state.settle_count < BALL_SETTLE_TICKS) {
            g_state.settle_count++;
        }
    } else {
        g_state.settle_count = 0U;
    }

    if (g_state.settle_count < BALL_SETTLE_TICKS) {
        return;
    }

    g_state.settle_count = 0U;
    if (g_state.mode == BALL_MODE_SEQ_PLUS) {
        g_state.mode = BALL_MODE_SEQ_ZERO;
        g_state.target_mm = 0.0f;
    } else if (g_state.mode == BALL_MODE_SEQ_ZERO) {
        g_state.mode = BALL_MODE_SEQ_MINUS;
        g_state.target_mm = BALL_TARGET_MINUS_MM;
    } else if (g_state.mode == BALL_MODE_SEQ_MINUS) {
        g_state.mode = BALL_MODE_DONE;
        g_state.target_mm = BALL_TARGET_MINUS_MM;
    }
}

void ball_balance_update(uint32_t now_ms, bool sample_ok,
                         const K230BallSample *sample)
{
    float dt_s;
    float raw_velocity;
    float pos_error;
    float pos_derivative;
    float target_velocity;
    float speed_error;

    g_state.sample_ok = sample_ok;
    if (g_state.mode == BALL_MODE_IDLE || g_state.mode == BALL_MODE_DONE) {
        return;
    }
    if (!sample_ok || (sample == 0)) {
        g_state.mode = BALL_MODE_ERROR;
        stepper_arm_set_pipe_angle(0.0f, BALL_THETA_RATE_DEG);
        return;
    }
    if (g_last_ms == 0U) {
        g_last_ms = now_ms;
        g_last_pos_mm = sample->x_mm;
        g_state.position_mm = sample->x_mm;
        return;
    }
    if ((now_ms - g_last_ms) < BALL_CTRL_PERIOD_MS) {
        return;
    }

    dt_s = (float)(now_ms - g_last_ms) / 1000.0f;
    raw_velocity = (sample->x_mm - g_last_pos_mm) / dt_s;
    g_state.position_mm = (g_state.position_mm * 0.70f) +
        (sample->x_mm * 0.30f);
    g_state.velocity_mm_s = (g_state.velocity_mm_s * 0.65f) +
        (raw_velocity * 0.35f);
    g_last_pos_mm = sample->x_mm;
    g_last_ms = now_ms;

    pos_error = g_state.target_mm - g_state.position_mm;
    pos_derivative = pos_error - g_last_pos_error;
    target_velocity = (BALL_POS_KP * pos_error) +
        (BALL_POS_KD * pos_derivative);
    target_velocity = clamp_f32_ball(target_velocity,
        -BALL_POS_MAX_SPEED, BALL_POS_MAX_SPEED);

    speed_error = target_velocity - g_state.velocity_mm_s;
    g_speed_integral += speed_error;
    g_speed_integral = clamp_f32_ball(g_speed_integral, -3000.0f, 3000.0f);
    g_state.theta_deg = (BALL_SPEED_KP * speed_error) +
        (BALL_SPEED_KI * g_speed_integral);
    g_state.theta_deg = clamp_f32_ball(g_state.theta_deg,
        -BALL_SPEED_MAX_THETA, BALL_SPEED_MAX_THETA);
    stepper_arm_set_pipe_angle(g_state.theta_deg, BALL_THETA_RATE_DEG);
    g_last_pos_error = pos_error;

    if ((g_state.mode == BALL_MODE_SEQ_PLUS) ||
        (g_state.mode == BALL_MODE_SEQ_ZERO) ||
        (g_state.mode == BALL_MODE_SEQ_MINUS)) {
        update_sequence_stage();
    }
}

BallBalanceState ball_balance_get_state(void)
{
    return g_state;
}
