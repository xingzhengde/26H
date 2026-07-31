#include "q4_motion.h"

#include "app_config.h"

static bool g_q4_active;
static uint32_t g_q4_start_ms;
static Q4MotionProfile g_motion_profile;

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
    g_motion_profile = Q4_MOTION_PROFILE_CENTER;
}

void q4_motion_select_profile(Q4MotionProfile profile)
{
    g_motion_profile = profile;
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
    bool arbitrary =
        (g_motion_profile == Q4_MOTION_PROFILE_ARBITRARY);
    float feedforward_deg = arbitrary ?
        (Q6_FEEDFORWARD_SIGN * Q6_INITIAL_ANGLE_DEG) :
        (Q4_FEEDFORWARD_SIGN * Q4_INITIAL_ANGLE_DEG);
    float max_deg = arbitrary ? Q6_MAX_FEEDFORWARD_DEG :
        Q4_MAX_FEEDFORWARD_DEG;

    return q4_clamp_f32(feedforward_deg, -max_deg, max_deg);
}

uint32_t q4_motion_get_preload_lead_ms(void)
{
    return (g_motion_profile == Q4_MOTION_PROFILE_ARBITRARY) ?
        Q6_PRELOAD_LEAD_MS : Q4_PRELOAD_LEAD_MS;
}

void q4_motion_get_setpoint(uint32_t now_ms, int32_t measured_speed_cps,
                            int32_t measured_accel_cps2,
                            Q4MotionSetpoint *setpoint)
{
    uint32_t elapsed_ms;
    float time_ratio;
    float planned_accel_mps2;
    bool arbitrary;
    uint32_t accel_time_ms;
    uint32_t initial_hold_ms;
    float cruise_target;
    int16_t start_pwm;
    int16_t cruise_pwm;
    float cruise_speed_mps;

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

    arbitrary = (g_motion_profile == Q4_MOTION_PROFILE_ARBITRARY);
    accel_time_ms = arbitrary ? Q6_ACCEL_TIME_MS : Q4_ACCEL_TIME_MS;
    initial_hold_ms = arbitrary ? Q6_INITIAL_ANGLE_HOLD_MS :
        Q4_INITIAL_ANGLE_HOLD_MS;
    cruise_target = arbitrary ? Q6_CRUISE_TARGET_COUNTS :
        Q4_CRUISE_TARGET_COUNTS;
    start_pwm = arbitrary ? Q6_START_PWM : Q4_START_PWM;
    cruise_pwm = arbitrary ? Q6_CRUISE_BASE_PWM : Q4_CRUISE_BASE_PWM;
    cruise_speed_mps = arbitrary ? Q6_EST_CRUISE_SPEED_MPS :
        Q4_EST_CRUISE_SPEED_MPS;

    elapsed_ms = now_ms - g_q4_start_ms;
    if (elapsed_ms >= accel_time_ms) {
        time_ratio = 1.0f;
        planned_accel_mps2 = 0.0f;
    } else {
        time_ratio = (float)elapsed_ms / (float)accel_time_ms;
        planned_accel_mps2 = cruise_speed_mps /
            ((float)accel_time_ms / 1000.0f);
    }

    if (((arbitrary && Q6_FEEDFORWARD_ENABLE) ||
         (!arbitrary && Q4_FEEDFORWARD_ENABLE)) &&
        (elapsed_ms < initial_hold_ms)) {
        setpoint->pipe_feedforward_deg =
            q4_motion_get_start_feedforward_deg();
    } else {
        setpoint->pipe_feedforward_deg = 0.0f;
    }

    setpoint->speed_target_counts = cruise_target * time_ratio;
    setpoint->base_pwm = (int16_t)((float)start_pwm +
        ((float)(cruise_pwm - start_pwm) * time_ratio));
    setpoint->ramp_ratio = time_ratio;
    setpoint->forward_accel_mps2 = planned_accel_mps2;
    setpoint->elapsed_ms = elapsed_ms;
    setpoint->passed_b_time = elapsed_ms >= (arbitrary ?
        Q6_LAP_TARGET_TIME_MS : Q4_AB_TARGET_TIME_MS);
}
