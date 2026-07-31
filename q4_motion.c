#include "q4_motion.h"

#include "app_config.h"

static bool g_q4_active;
static uint32_t g_q4_start_ms;

static float q4_clamp_f32(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

void q4_motion_init(void)
{
    g_q4_active = false;
    g_q4_start_ms = 0U;
}

void q4_motion_start(uint32_t now_ms)
{
    g_q4_start_ms = now_ms;
    g_q4_active = true;
}

void q4_motion_stop(void)
{
    g_q4_active = false;
}

bool q4_motion_is_active(void)
{
    return g_q4_active;
}

float q4_motion_get_start_feedforward_deg(void)
{
    float feedforward_deg =
        Q4_FEEDFORWARD_SIGN * Q4_INITIAL_ANGLE_DEG;

    return q4_clamp_f32(feedforward_deg,
        -Q4_MAX_FEEDFORWARD_DEG, Q4_MAX_FEEDFORWARD_DEG);
}

void q4_motion_get_setpoint(uint32_t now_ms, int32_t measured_speed_cps,
                            int32_t measured_accel_cps2,
                            Q4MotionSetpoint *setpoint)
{
    uint32_t elapsed_ms;
    float time_ratio;
    float planned_accel_mps2;

    (void)measured_speed_cps;
    (void)measured_accel_cps2;

    if (setpoint == 0) {
        return;
    }

    if (!g_q4_active) {
        setpoint->speed_target_counts = 0.0f;
        setpoint->base_pwm = 0;
        setpoint->ramp_ratio = 0.0f;
        setpoint->forward_accel_mps2 = 0.0f;
        setpoint->pipe_feedforward_deg = 0.0f;
        setpoint->elapsed_ms = 0U;
        setpoint->passed_b_time = false;
        return;
    }

    elapsed_ms = now_ms - g_q4_start_ms;
    if (elapsed_ms >= Q4_ACCEL_TIME_MS) {
        time_ratio = 1.0f;
        planned_accel_mps2 = 0.0f;
    } else {
        time_ratio = (float)elapsed_ms / (float)Q4_ACCEL_TIME_MS;
        planned_accel_mps2 = Q4_EST_CRUISE_SPEED_MPS /
            ((float)Q4_ACCEL_TIME_MS / 1000.0f);
    }

#if Q4_FEEDFORWARD_ENABLE
    /* 固定目标角保持期内不变；保持结束后一次清零，由 PID 接管。 */
    if (elapsed_ms < Q4_INITIAL_ANGLE_HOLD_MS) {
        setpoint->pipe_feedforward_deg =
            q4_motion_get_start_feedforward_deg();
    } else {
        setpoint->pipe_feedforward_deg = 0.0f;
    }
#else
    setpoint->pipe_feedforward_deg = 0.0f;
#endif

    setpoint->speed_target_counts =
        Q4_CRUISE_TARGET_COUNTS * time_ratio;
    setpoint->base_pwm = (int16_t)((float)Q4_START_PWM +
        ((float)(Q4_CRUISE_BASE_PWM - Q4_START_PWM) * time_ratio));
    setpoint->ramp_ratio = time_ratio;
    setpoint->forward_accel_mps2 = planned_accel_mps2;
    setpoint->elapsed_ms = elapsed_ms;
    setpoint->passed_b_time = elapsed_ms >= Q4_AB_TARGET_TIME_MS;
}
