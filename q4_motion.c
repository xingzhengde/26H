#include "q4_motion.h"

#include "app_config.h"

static bool g_q4_active;
static uint32_t g_q4_start_ms;

/**
 * @brief 浮点限幅，防止编码器尖峰产生危险倾角。
 */
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
    float planned_accel_mps2 = Q4_EST_CRUISE_SPEED_MPS /
        ((float)Q4_ACCEL_TIME_MS / 1000.0f);
    float feedforward_deg = Q4_FEEDFORWARD_SIGN * Q4_FEEDFORWARD_GAIN *
        planned_accel_mps2 * 57.2957795f / Q4_GRAVITY_MPS2;

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
    float compensation_accel_mps2;
    float feedforward_deg;
    float feedforward_envelope = 0.0f;

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
        /*
         * 模式二唯一的运动规律：越过死区后 PWM 与时间成线性关系。
         * 没有速度 PID，也没有第二套匀速控制器。
         */
        time_ratio = (float)elapsed_ms / (float)Q4_ACCEL_TIME_MS;
        planned_accel_mps2 = Q4_EST_CRUISE_SPEED_MPS /
            ((float)Q4_ACCEL_TIME_MS / 1000.0f);
    }

    /* 编码器不控制车速，只负责可靠检测实车起步并修正补偿量。 */
    compensation_accel_mps2 = planned_accel_mps2;
    if (measured_speed_cps >= Q4_MOVING_SPEED_CPS) {
        float measured_accel_mps2 =
            (float)measured_accel_cps2 * Q4_ENCODER_METERS_PER_COUNT;

        measured_accel_mps2 = q4_clamp_f32(measured_accel_mps2,
            0.0f, Q4_ACCEL_LIMIT_MPS2);
        compensation_accel_mps2 =
            (Q4_MEASURED_ACCEL_WEIGHT * measured_accel_mps2) +
            ((1.0f - Q4_MEASURED_ACCEL_WEIGHT) *
             planned_accel_mps2);
    }

    /*
     * 移动补偿不在按键瞬间介入：等待车辆越过死区后再渐入，并在开环
     * 加速结束前渐出。巡航阶段前馈为 0，只保留 K230 球位置反馈微调。
     */
#if Q4_FEEDFORWARD_ENABLE
    if (elapsed_ms < Q4_ACCEL_TIME_MS) {
        /*
         * B15 触发时已经置入完整计划前馈，此处仅在加速结束前渐出。
         * 编码器不参与触发，只在车辆运动后修正加速度估计值。
         */
        uint32_t accel_remaining_ms =
            Q4_ACCEL_TIME_MS - elapsed_ms;
        float fade_out = (float)accel_remaining_ms /
            (float)Q4_FEEDFORWARD_FADE_MS;

        fade_out = q4_clamp_f32(fade_out, 0.0f, 1.0f);
        feedforward_envelope = fade_out;
    }
    feedforward_deg = Q4_FEEDFORWARD_SIGN * Q4_FEEDFORWARD_GAIN *
        feedforward_envelope * compensation_accel_mps2 *
        57.2957795f / Q4_GRAVITY_MPS2;
    feedforward_deg = q4_clamp_f32(feedforward_deg,
        -Q4_MAX_FEEDFORWARD_DEG, Q4_MAX_FEEDFORWARD_DEG);
#else
    feedforward_deg = 0.0f;
#endif

    setpoint->speed_target_counts =
        Q4_CRUISE_TARGET_COUNTS * time_ratio;
    /*
     * 低于起步 PWM 的区域无法克服静摩擦。目标从起步值开始，再线性增加
     * 到巡航值；输出层仍以与模式一相同的变化率平滑到达起步值。
     */
    setpoint->base_pwm = (int16_t)((float)Q4_START_PWM +
        ((float)(Q4_CRUISE_BASE_PWM - Q4_START_PWM) * time_ratio));
    setpoint->ramp_ratio = time_ratio;
    setpoint->forward_accel_mps2 = compensation_accel_mps2;
    setpoint->pipe_feedforward_deg = feedforward_deg;
    setpoint->elapsed_ms = elapsed_ms;
    setpoint->passed_b_time = elapsed_ms >= Q4_AB_TARGET_TIME_MS;
}
