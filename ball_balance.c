#include "ball_balance.h"

#include "app_config.h"
#include "stepper_arm.h"

static BallBalanceState g_state;
static uint32_t g_last_sample_timestamp_ms;
static uint16_t g_last_frame_id;
static uint16_t g_last_frame_time_ms;
static bool g_have_frame_meta;
static float g_last_raw_pos_mm;
static float g_speed_integral;
static float g_q4_position_integral;
static float g_q4_speed_integral;
static float g_q4_last_speed_error;
static float g_last_raw_velocity_mm_s;
static float g_filtered_accel_mm_s2;
static float g_feedforward_deg;
static float g_feedback_deg;
static bool g_q4_hold_active;

/**
 * @brief 浮点限幅，防止视觉异常值或积分累积产生危险倾角。
 */
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

/**
 * @brief 把物理水管角转换为当前执行机构命令。
 *
 * 物理角正方向统一定义为“车尾合页端高、车头电机端低”。如果实机
 * 第一次测试方向相反，只改 BALL_ACTUATOR_ANGLE_SIGN，不改控制公式。
 */
static void apply_pipe_command_with_rate(float motor_rate_deg_s)
{
    float total_angle = g_feedforward_deg + g_feedback_deg;

    total_angle = clamp_f32_ball(total_angle,
        -BALL_SPEED_MAX_THETA, BALL_SPEED_MAX_THETA);
    g_state.feedforward_deg = g_feedforward_deg;
    g_state.feedback_deg = g_feedback_deg;
    g_state.theta_deg = total_angle;
    stepper_arm_set_pipe_angle(
        BALL_ACTUATOR_ANGLE_SIGN * total_angle,
        motor_rate_deg_s);
}

static void apply_pipe_command(void)
{
    apply_pipe_command_with_rate(BALL_THETA_RATE_DEG);
}

/**
 * @brief 清除与一次控制任务相关的滤波和积分状态。
 *
 * 每次切换目标都重新初始化，防止上一任务的速度积分把球推向端部。
 */
static void reset_loop_state(void)
{
    g_last_sample_timestamp_ms = 0U;
    g_last_frame_id = 0U;
    g_last_frame_time_ms = 0U;
    g_have_frame_meta = false;
    g_last_raw_pos_mm = 0.0f;
    g_speed_integral = 0.0f;
    g_q4_position_integral = 0.0f;
    g_q4_speed_integral = 0.0f;
    g_q4_last_speed_error = 0.0f;
    g_last_raw_velocity_mm_s = 0.0f;
    g_filtered_accel_mm_s2 = 0.0f;
    g_feedback_deg = 0.0f;
    g_state.position_mm = 0.0f;
    g_state.velocity_mm_s = 0.0f;
    g_state.feedforward_deg = g_feedforward_deg;
    g_state.feedback_deg = 0.0f;
    g_state.settle_count = 0U;
    g_state.theta_deg = g_feedforward_deg;
}

void ball_balance_init(void)
{
    g_feedforward_deg = 0.0f;
    g_q4_hold_active = false;
    g_state.mode = BALL_MODE_IDLE;
    g_state.target_mm = 0.0f;
    g_state.sample_ok = false;
    reset_loop_state();
}

void ball_balance_start_sequence(void)
{
    g_q4_hold_active = false;
    g_state.mode = BALL_MODE_SEQ_PLUS;
    g_state.target_mm = BALL_TARGET_PLUS_MM;
    reset_loop_state();
}

void ball_balance_hold_target(float target_mm)
{
    g_q4_hold_active = false;
    g_state.mode = BALL_MODE_HOLD_TARGET;
    g_state.target_mm = target_mm;
    reset_loop_state();
}

void ball_balance_hold_q4(float target_mm)
{
    g_q4_hold_active = true;
    g_state.mode = BALL_MODE_HOLD_TARGET;
    g_state.target_mm = target_mm;
    reset_loop_state();
}

void ball_balance_set_feedforward(float pipe_angle_deg)
{
    g_feedforward_deg = clamp_f32_ball(pipe_angle_deg,
        -Q4_MAX_FEEDFORWARD_DEG, Q4_MAX_FEEDFORWARD_DEG);
    if (g_q4_hold_active) {
        /* B15 启动时立即把前馈目标提交给 X42S，不等待下一帧视觉数据。 */
        apply_pipe_command();
    }
}

void ball_balance_set_initial_feedforward(float pipe_angle_deg)
{
    g_feedforward_deg = clamp_f32_ball(pipe_angle_deg,
        -Q4_MAX_FEEDFORWARD_DEG, Q4_MAX_FEEDFORWARD_DEG);
    if (g_q4_hold_active) {
        /* 启动补偿直接高速到达目标角，正常 PID 接管后仍使用原来的微调速度。 */
        apply_pipe_command_with_rate(Q4_INITIAL_ANGLE_RATE_DEG_S);
    }
}

void ball_balance_stop(void)
{
    g_state.mode = BALL_MODE_IDLE;
    g_q4_hold_active = false;
    g_feedforward_deg = 0.0f;
    reset_loop_state();
    apply_pipe_command();
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
    g_speed_integral = 0.0f;
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
    float raw_accel;
    float pos_error;
    float control_position_mm;
    float control_velocity_mm_s;
    float lookahead_s;
    float predicted_position_delta;
    float predicted_speed_delta;
    float target_velocity;
    float speed_error;
    uint32_t sample_dt_ms;

    (void)now_ms;
    g_state.sample_ok = sample_ok;
    if ((g_state.mode == BALL_MODE_IDLE) ||
        (g_state.mode == BALL_MODE_DONE)) {
        return;
    }

    /*
     * k230_ball_get_sample() 在数据超过 180ms 或置信度不足时返回 false。
     * 此时撤掉闭环修正但保留小车加速度前馈，避免继续沿错误方向积分。
     */
    if (!sample_ok || (sample == 0)) {
        g_last_sample_timestamp_ms = 0U;
        g_have_frame_meta = false;
        g_speed_integral = 0.0f;
        g_q4_position_integral = 0.0f;
        g_q4_speed_integral = 0.0f;
        g_q4_last_speed_error = 0.0f;
        g_last_raw_velocity_mm_s = 0.0f;
        g_filtered_accel_mm_s2 = 0.0f;
        g_feedback_deg = 0.0f;
        g_state.velocity_mm_s = 0.0f;
        apply_pipe_command();
        return;
    }

    /*
     * K230 约每 40ms 发送一次位置。主循环比它快，必须忽略重复时间戳，
     * 否则同一位置会被误算成多次零速度样本，削弱速度阻尼。
     */
    if ((sample->frame_meta_valid && g_have_frame_meta &&
         (sample->frame_id == g_last_frame_id)) ||
        (!sample->frame_meta_valid &&
         (sample->timestamp_ms == g_last_sample_timestamp_ms))) {
        apply_pipe_command();
        return;
    }

    if (g_last_sample_timestamp_ms == 0U) {
        g_last_sample_timestamp_ms = sample->timestamp_ms;
        g_last_frame_id = sample->frame_id;
        g_last_frame_time_ms = sample->frame_time_ms;
        g_have_frame_meta = sample->frame_meta_valid;
        g_last_raw_pos_mm = sample->x_mm;
        g_state.position_mm = sample->x_mm;
        g_state.velocity_mm_s = 0.0f;
        g_last_raw_velocity_mm_s = 0.0f;
        g_filtered_accel_mm_s2 = 0.0f;
        apply_pipe_command();
        return;
    }

    if (sample->frame_meta_valid && g_have_frame_meta) {
        /* uint16_t 相减天然兼容 K230 毫秒计数器约 65.5s 的回绕。 */
        sample_dt_ms = (uint16_t)(sample->frame_time_ms -
            g_last_frame_time_ms);
    } else {
        sample_dt_ms = sample->timestamp_ms -
            g_last_sample_timestamp_ms;
    }
    dt_s = (float)sample_dt_ms / 1000.0f;
    if ((dt_s < 0.010f) || (dt_s > 0.250f)) {
        g_last_sample_timestamp_ms = sample->timestamp_ms;
        g_last_frame_id = sample->frame_id;
        g_last_frame_time_ms = sample->frame_time_ms;
        g_have_frame_meta = sample->frame_meta_valid;
        g_last_raw_pos_mm = sample->x_mm;
        g_state.position_mm = sample->x_mm;
        g_state.velocity_mm_s = 0.0f;
        g_speed_integral = 0.0f;
        g_q4_position_integral = 0.0f;
        g_q4_speed_integral = 0.0f;
        g_q4_last_speed_error = 0.0f;
        g_last_raw_velocity_mm_s = 0.0f;
        g_filtered_accel_mm_s2 = 0.0f;
        g_feedback_deg = 0.0f;
        apply_pipe_command();
        return;
    }

    raw_velocity = (sample->x_mm - g_last_raw_pos_mm) / dt_s;
    raw_accel = (raw_velocity - g_last_raw_velocity_mm_s) / dt_s;
    raw_accel = clamp_f32_ball(raw_accel,
        -Q4_BALL_ACCEL_LIMIT_MM_S2, Q4_BALL_ACCEL_LIMIT_MM_S2);
    g_filtered_accel_mm_s2 =
        ((1.0f - Q4_BALL_ACCEL_FILTER_NEW) *
         g_filtered_accel_mm_s2) +
        (Q4_BALL_ACCEL_FILTER_NEW * raw_accel);
    g_state.position_mm =
        ((1.0f - Q4_BALL_POSITION_FILTER_NEW) * g_state.position_mm) +
        (Q4_BALL_POSITION_FILTER_NEW * sample->x_mm);
    g_state.velocity_mm_s =
        ((1.0f - Q4_BALL_VELOCITY_FILTER_NEW) *
         g_state.velocity_mm_s) +
        (Q4_BALL_VELOCITY_FILTER_NEW * raw_velocity);
    g_last_raw_velocity_mm_s = raw_velocity;
    g_last_raw_pos_mm = sample->x_mm;
    g_last_sample_timestamp_ms = sample->timestamp_ms;
    g_last_frame_id = sample->frame_id;
    g_last_frame_time_ms = sample->frame_time_ms;
    g_have_frame_meta = sample->frame_meta_valid;

    /*
     * +x 指向车尾合页。球偏向车尾时 pos_error<0，外环要求向车头的
     * 负速度；要产生该加速度，物理水管角应为正（车尾端高）。
     * 因此速度 PI 输出前有负号。
     */
    {
        uint32_t lookahead_ms = Q4_BALL_ACTUATOR_LOOKAHEAD_MS;

        if (sample->frame_meta_valid) {
            lookahead_ms += sample_dt_ms * Q4_BALL_MEDIAN_DELAY_FRAMES;
        }
        if (sample->vision_latency_valid) {
            lookahead_ms += sample->vision_latency_ms;
        }
        if (lookahead_ms > Q4_BALL_LOOKAHEAD_MAX_MS) {
            lookahead_ms = Q4_BALL_LOOKAHEAD_MAX_MS;
        }
        lookahead_s = (float)lookahead_ms / 1000.0f;
    }

    predicted_position_delta = clamp_f32_ball(
        g_state.velocity_mm_s * lookahead_s,
        -Q4_BALL_PREDICT_POS_MAX_MM, Q4_BALL_PREDICT_POS_MAX_MM);
    predicted_speed_delta = clamp_f32_ball(
        g_filtered_accel_mm_s2 * lookahead_s,
        -Q4_BALL_PREDICT_SPEED_DELTA_MAX,
        Q4_BALL_PREDICT_SPEED_DELTA_MAX);
    control_position_mm = g_state.position_mm + predicted_position_delta;
    control_velocity_mm_s = g_state.velocity_mm_s + predicted_speed_delta;
    pos_error = g_state.target_mm - control_position_mm;
    if (g_q4_hold_active) {
        float position_output;
        float speed_output;
        float speed_derivative;

        /*
         * 固定启动倾角期间只更新视觉位置和速度状态，不叠加 PID 输出。
         * 保持结束后前馈直接归零，稳定版位置/速度 PID 从最新状态接管。
         */
        if (abs_f32(g_feedforward_deg) > 0.001f) {
            g_q4_position_integral = 0.0f;
            g_q4_speed_integral = 0.0f;
            g_q4_last_speed_error = 0.0f;
            g_feedback_deg = 0.0f;
            apply_pipe_command();
            return;
        }

        /*
         * 模式二直接对球位置做小角度 PID。速度即位置误差的负导数，
         * 因此 D 项使用实测球速提供阻尼，避免步进电机反复推拉。
         */
        if (abs_f32(pos_error) <= Q4_BALL_PID_DEADBAND_MM) {
            pos_error = 0.0f;
            g_q4_position_integral *= 0.85f;
        } else {
            g_q4_position_integral += pos_error * dt_s;
            g_q4_position_integral = clamp_f32_ball(
                g_q4_position_integral,
                -Q4_BALL_PID_INT_LIMIT, Q4_BALL_PID_INT_LIMIT);
        }
        /*
         * 位置 PID 把球拉回中心；速度 PID 的目标速度固定为 0，专门抑制
         * 球继续滚动。两路先各自限幅再加权，避免视觉跳点独占全部倾角。
         */
        position_output = (Q4_BALL_POS_KP * pos_error) +
            (Q4_BALL_POS_KI * g_q4_position_integral) -
            (Q4_BALL_POS_KD * control_velocity_mm_s);
        position_output = clamp_f32_ball(position_output,
            -Q4_BALL_POS_OUT_MAX_DEG, Q4_BALL_POS_OUT_MAX_DEG);

        speed_error = -control_velocity_mm_s;
        g_q4_speed_integral += speed_error * dt_s;
        g_q4_speed_integral = clamp_f32_ball(g_q4_speed_integral,
            -Q4_BALL_SPEED_INT_LIMIT, Q4_BALL_SPEED_INT_LIMIT);
        speed_derivative = (speed_error - g_q4_last_speed_error) / dt_s;
        speed_derivative = clamp_f32_ball(speed_derivative,
            -Q4_BALL_ACCEL_LIMIT_MM_S2,
            Q4_BALL_ACCEL_LIMIT_MM_S2);
        g_q4_last_speed_error = speed_error;
        speed_output = (Q4_BALL_SPEED_KP * speed_error) +
            (Q4_BALL_SPEED_KI * g_q4_speed_integral) +
            (Q4_BALL_SPEED_KD * speed_derivative);
        speed_output = clamp_f32_ball(speed_output,
            -Q4_BALL_SPEED_OUT_MAX_DEG, Q4_BALL_SPEED_OUT_MAX_DEG);

        g_feedback_deg = -((Q4_BALL_POS_WEIGHT * position_output) +
            (Q4_BALL_SPEED_WEIGHT * speed_output));
        g_feedback_deg = clamp_f32_ball(g_feedback_deg,
            -Q4_BALL_PID_MAX_THETA, Q4_BALL_PID_MAX_THETA);
        apply_pipe_command();
        return;
    }

    target_velocity = (BALL_POS_KP * pos_error) -
        (BALL_POS_KD * g_state.velocity_mm_s);
    target_velocity = clamp_f32_ball(target_velocity,
        -BALL_POS_MAX_SPEED, BALL_POS_MAX_SPEED);

    speed_error = target_velocity - g_state.velocity_mm_s;
    g_speed_integral += speed_error * dt_s;
    g_speed_integral = clamp_f32_ball(g_speed_integral,
        -BALL_SPEED_INT_MAX, BALL_SPEED_INT_MAX);
    g_feedback_deg = -((BALL_SPEED_KP * speed_error) +
        (BALL_SPEED_KI * g_speed_integral));
    g_feedback_deg = clamp_f32_ball(g_feedback_deg,
        -BALL_SPEED_MAX_THETA, BALL_SPEED_MAX_THETA);
    apply_pipe_command();

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
