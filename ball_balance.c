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
static float g_q6_curve_position_integral;
static float g_q6_curve_speed_integral;
static float g_q6_curve_last_speed_error;
static bool g_q6_curve_cascade_active;
static float g_last_raw_velocity_mm_s;
static float g_filtered_accel_mm_s2;
static float g_feedforward_deg;
static float g_curve_feedforward_deg;
static float g_feedback_deg;
static float g_feedback_scale;
static bool g_q4_hold_active;
static bool g_q6_hold_active;

typedef struct {
    float pos_kp, pos_ki, pos_kd, pos_weight;
    float speed_kp, speed_ki, speed_kd, speed_weight;
    float deadband_mm, pos_int_limit, speed_int_limit;
    float accel_limit, position_filter, velocity_filter, accel_filter;
    uint8_t median_delay_frames;
    uint32_t actuator_lookahead_ms, lookahead_max_ms;
    float predict_pos_max, predict_speed_max;
    float pos_out_max_deg, speed_out_max_deg, max_theta;
} BallPidConfig;

static const BallPidConfig g_q4_pid = {
    Q4_BALL_POS_KP, Q4_BALL_POS_KI, Q4_BALL_POS_KD, Q4_BALL_POS_WEIGHT,
    Q4_BALL_SPEED_KP, Q4_BALL_SPEED_KI, Q4_BALL_SPEED_KD,
    Q4_BALL_SPEED_WEIGHT, Q4_BALL_PID_DEADBAND_MM,
    Q4_BALL_PID_INT_LIMIT, Q4_BALL_SPEED_INT_LIMIT,
    Q4_BALL_ACCEL_LIMIT_MM_S2, Q4_BALL_POSITION_FILTER_NEW,
    Q4_BALL_VELOCITY_FILTER_NEW, Q4_BALL_ACCEL_FILTER_NEW,
    Q4_BALL_MEDIAN_DELAY_FRAMES, Q4_BALL_ACTUATOR_LOOKAHEAD_MS,
    Q4_BALL_LOOKAHEAD_MAX_MS, Q4_BALL_PREDICT_POS_MAX_MM,
    Q4_BALL_PREDICT_SPEED_DELTA_MAX, Q4_BALL_POS_OUT_MAX_DEG,
    Q4_BALL_SPEED_OUT_MAX_DEG, Q4_BALL_PID_MAX_THETA
};

static const BallPidConfig g_q6_pid = {
    Q6_BALL_POS_KP, Q6_BALL_POS_KI, Q6_BALL_POS_KD, Q6_BALL_POS_WEIGHT,
    Q6_BALL_SPEED_KP, Q6_BALL_SPEED_KI, Q6_BALL_SPEED_KD,
    Q6_BALL_SPEED_WEIGHT, Q6_BALL_PID_DEADBAND_MM,
    Q6_BALL_PID_INT_LIMIT, Q6_BALL_SPEED_INT_LIMIT,
    Q6_BALL_ACCEL_LIMIT_MM_S2, Q6_BALL_POSITION_FILTER_NEW,
    Q6_BALL_VELOCITY_FILTER_NEW, Q6_BALL_ACCEL_FILTER_NEW,
    Q6_BALL_MEDIAN_DELAY_FRAMES, Q6_BALL_ACTUATOR_LOOKAHEAD_MS,
    Q6_BALL_LOOKAHEAD_MAX_MS, Q6_BALL_PREDICT_POS_MAX_MM,
    Q6_BALL_PREDICT_SPEED_DELTA_MAX, Q6_BALL_POS_OUT_MAX_DEG,
    Q6_BALL_SPEED_OUT_MAX_DEG, Q6_BALL_PID_MAX_THETA
};

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
    float applied_feedback_deg = g_feedback_deg * g_feedback_scale;
    float total_angle = g_feedforward_deg + g_curve_feedforward_deg +
        applied_feedback_deg;

    total_angle = clamp_f32_ball(total_angle,
        -BALL_SPEED_MAX_THETA, BALL_SPEED_MAX_THETA);
    g_state.feedforward_deg = g_feedforward_deg + g_curve_feedforward_deg;
    g_state.feedback_deg = applied_feedback_deg;
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
    g_q6_curve_position_integral = 0.0f;
    g_q6_curve_speed_integral = 0.0f;
    g_q6_curve_last_speed_error = 0.0f;
    g_q6_curve_cascade_active = false;
    g_last_raw_velocity_mm_s = 0.0f;
    g_filtered_accel_mm_s2 = 0.0f;
    g_feedback_deg = 0.0f;
    g_state.position_mm = 0.0f;
    g_state.velocity_mm_s = 0.0f;
    g_state.feedforward_deg = g_feedforward_deg + g_curve_feedforward_deg;
    g_state.feedback_deg = 0.0f;
    g_state.settle_count = 0U;
    g_state.theta_deg = g_feedforward_deg + g_curve_feedforward_deg;
}

void ball_balance_init(void)
{
    g_feedforward_deg = 0.0f;
    g_curve_feedforward_deg = 0.0f;
    g_feedback_scale = 1.0f;
    g_q4_hold_active = false;
    g_q6_hold_active = false;
    g_state.mode = BALL_MODE_IDLE;
    g_state.target_mm = 0.0f;
    g_state.sample_ok = false;
    reset_loop_state();
}

void ball_balance_start_sequence(void)
{
    g_q4_hold_active = false;
    g_q6_hold_active = false;
    g_curve_feedforward_deg = 0.0f;
    g_feedback_scale = 1.0f;
    g_state.mode = BALL_MODE_SEQ_PLUS;
    g_state.target_mm = BALL_TARGET_PLUS_MM;
    reset_loop_state();
}

void ball_balance_hold_target(float target_mm)
{
    g_q4_hold_active = false;
    g_q6_hold_active = false;
    g_curve_feedforward_deg = 0.0f;
    g_feedback_scale = 1.0f;
    g_state.mode = BALL_MODE_HOLD_TARGET;
    g_state.target_mm = target_mm;
    reset_loop_state();
}

void ball_balance_hold_q4(float target_mm)
{
    g_q4_hold_active = true;
    g_q6_hold_active = false;
    g_curve_feedforward_deg = 0.0f;
    g_feedback_scale = 1.0f;
    g_state.mode = BALL_MODE_HOLD_TARGET;
    g_state.target_mm = target_mm;
    reset_loop_state();
}

void ball_balance_hold_q6(float target_mm)
{
    g_q4_hold_active = true;
    g_q6_hold_active = true;
    g_curve_feedforward_deg = 0.0f;
    g_feedback_scale = 0.0f;
    g_state.mode = BALL_MODE_HOLD_TARGET;
    g_state.target_mm = target_mm;
    reset_loop_state();
}

void ball_balance_set_feedforward(float pipe_angle_deg)
{
    float max_deg = g_q6_hold_active ? Q6_MAX_FEEDFORWARD_DEG :
        Q4_MAX_FEEDFORWARD_DEG;

    g_feedforward_deg = clamp_f32_ball(pipe_angle_deg, -max_deg, max_deg);
    if (g_q4_hold_active) {
        /* B15 启动时立即把前馈目标提交给 X42S，不等待下一帧视觉数据。 */
        apply_pipe_command();
    }
}

void ball_balance_set_curve_feedforward(float pipe_angle_deg)
{
    bool cascade_active;

    if (!g_q6_hold_active) {
        g_curve_feedforward_deg = 0.0f;
        g_q6_curve_cascade_active = false;
        return;
    }
    g_curve_feedforward_deg = clamp_f32_ball(pipe_angle_deg,
        -Q6_CURVE_FF_MAX_DEG, Q6_CURVE_FF_MAX_DEG);
    cascade_active = abs_f32(g_curve_feedforward_deg) > 0.02f;
    if (cascade_active != g_q6_curve_cascade_active) {
        /* Reset both controllers at curve transitions to avoid stale I terms. */
        g_q4_position_integral = 0.0f;
        g_q4_speed_integral = 0.0f;
        g_q4_last_speed_error = 0.0f;
        g_q6_curve_position_integral = 0.0f;
        g_q6_curve_speed_integral = 0.0f;
        g_q6_curve_last_speed_error = 0.0f;
        g_q6_curve_cascade_active = cascade_active;
    }
    /* 弯道前馈独立叠加，不触发启动阶段的 PID 冻结。 */
    apply_pipe_command();
}

void ball_balance_set_feedback_scale(float scale)
{
    g_feedback_scale = clamp_f32_ball(scale, 0.0f, 1.0f);
    if (g_q4_hold_active) {
        apply_pipe_command();
    }
}

void ball_balance_set_initial_feedforward(float pipe_angle_deg)
{
    float max_deg = g_q6_hold_active ? Q6_MAX_FEEDFORWARD_DEG :
        Q4_MAX_FEEDFORWARD_DEG;
    float rate_deg_s = g_q6_hold_active ?
        Q6_INITIAL_ANGLE_RATE_DEG_S : Q4_INITIAL_ANGLE_RATE_DEG_S;

    g_feedforward_deg = clamp_f32_ball(pipe_angle_deg, -max_deg, max_deg);
    if (g_q4_hold_active) {
        /* 启动补偿直接高速到达目标角，正常 PID 接管后仍使用原来的微调速度。 */
        apply_pipe_command_with_rate(rate_deg_s);
    }
}

void ball_balance_stop(void)
{
    g_state.mode = BALL_MODE_IDLE;
    g_q4_hold_active = false;
    g_q6_hold_active = false;
    g_feedforward_deg = 0.0f;
    g_curve_feedforward_deg = 0.0f;
    g_feedback_scale = 1.0f;
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
    const BallPidConfig *pid_cfg = g_q6_hold_active ?
        &g_q6_pid : &g_q4_pid;

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
        -pid_cfg->accel_limit, pid_cfg->accel_limit);
    g_filtered_accel_mm_s2 =
        ((1.0f - pid_cfg->accel_filter) *
         g_filtered_accel_mm_s2) +
        (pid_cfg->accel_filter * raw_accel);
    g_state.position_mm =
        ((1.0f - pid_cfg->position_filter) * g_state.position_mm) +
        (pid_cfg->position_filter * sample->x_mm);
    g_state.velocity_mm_s =
        ((1.0f - pid_cfg->velocity_filter) *
         g_state.velocity_mm_s) +
        (pid_cfg->velocity_filter * raw_velocity);
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
        uint32_t lookahead_ms = pid_cfg->actuator_lookahead_ms;

        if (sample->frame_meta_valid) {
            lookahead_ms += sample_dt_ms * pid_cfg->median_delay_frames;
        }
        if (sample->vision_latency_valid) {
            lookahead_ms += sample->vision_latency_ms;
        }
        if (lookahead_ms > pid_cfg->lookahead_max_ms) {
            lookahead_ms = pid_cfg->lookahead_max_ms;
        }
        lookahead_s = (float)lookahead_ms / 1000.0f;
    }

    predicted_position_delta = clamp_f32_ball(
        g_state.velocity_mm_s * lookahead_s,
        -pid_cfg->predict_pos_max, pid_cfg->predict_pos_max);
    predicted_speed_delta = clamp_f32_ball(
        g_filtered_accel_mm_s2 * lookahead_s,
        -pid_cfg->predict_speed_max, pid_cfg->predict_speed_max);
    control_position_mm = g_state.position_mm + predicted_position_delta;
    control_velocity_mm_s = g_state.velocity_mm_s + predicted_speed_delta;
    pos_error = g_state.target_mm - control_position_mm;
    if (g_q4_hold_active) {
        float position_output;
        float speed_output;
        float speed_derivative;
        bool q6_curve_control = g_q6_hold_active &&
            g_q6_curve_cascade_active;

        /*
         * 固定启动倾角期间只更新视觉位置和速度状态，不叠加 PID 输出。
         * 保持结束后前馈直接归零，稳定版位置/速度 PID 从最新状态接管。
         */
        if (abs_f32(g_feedforward_deg) > 0.001f) {
            g_q4_position_integral = 0.0f;
            g_q4_speed_integral = 0.0f;
            g_q4_last_speed_error = 0.0f;
            g_q6_curve_position_integral = 0.0f;
            g_q6_curve_speed_integral = 0.0f;
            g_q6_curve_last_speed_error = 0.0f;
            g_feedback_deg = 0.0f;
            apply_pipe_command();
            return;
        }

        if (q6_curve_control) {
            float target_velocity_mm_s;

            /* Compensate the measured curve equilibrium bias toward motor end. */
            pos_error += Q6_CURVE_TARGET_BIAS_MM;

            /* Position outer loop: position error becomes target ball speed. */
            if (abs_f32(pos_error) <= Q6_CURVE_CASCADE_DEADBAND_MM) {
                pos_error = 0.0f;
                g_q6_curve_position_integral *= 0.85f;
            } else {
                g_q6_curve_position_integral += pos_error * dt_s;
                g_q6_curve_position_integral = clamp_f32_ball(
                    g_q6_curve_position_integral,
                    -Q6_CURVE_CASCADE_POS_INT_LIMIT,
                    Q6_CURVE_CASCADE_POS_INT_LIMIT);
            }
            target_velocity_mm_s =
                (Q6_CURVE_CASCADE_POS_KP * pos_error) +
                (Q6_CURVE_CASCADE_POS_KI *
                    g_q6_curve_position_integral) -
                (Q6_CURVE_CASCADE_POS_KD * control_velocity_mm_s);
            target_velocity_mm_s = clamp_f32_ball(target_velocity_mm_s,
                -Q6_CURVE_CASCADE_TARGET_SPEED_MAX_MM_S,
                Q6_CURVE_CASCADE_TARGET_SPEED_MAX_MM_S);

            /* Speed inner loop: target speed becomes the pipe angle command. */
            speed_error = target_velocity_mm_s - control_velocity_mm_s;
            g_q6_curve_speed_integral += speed_error * dt_s;
            g_q6_curve_speed_integral = clamp_f32_ball(
                g_q6_curve_speed_integral,
                -Q6_CURVE_CASCADE_SPEED_INT_LIMIT,
                Q6_CURVE_CASCADE_SPEED_INT_LIMIT);
            speed_derivative =
                (speed_error - g_q6_curve_last_speed_error) / dt_s;
            speed_derivative = clamp_f32_ball(speed_derivative,
                -pid_cfg->accel_limit, pid_cfg->accel_limit);
            g_q6_curve_last_speed_error = speed_error;
            g_feedback_deg = -(
                (Q6_CURVE_CASCADE_SPEED_KP * speed_error) +
                (Q6_CURVE_CASCADE_SPEED_KI *
                    g_q6_curve_speed_integral) +
                (Q6_CURVE_CASCADE_SPEED_KD * speed_derivative));
            g_feedback_deg = clamp_f32_ball(g_feedback_deg,
                -Q6_CURVE_CASCADE_ANGLE_MAX_DEG,
                Q6_CURVE_CASCADE_ANGLE_MAX_DEG);
            apply_pipe_command();
            return;
        }

        /*
         * 模式二直接对球位置做小角度 PID。速度即位置误差的负导数，
         * 因此 D 项使用实测球速提供阻尼，避免步进电机反复推拉。
         */
        if (abs_f32(pos_error) <= pid_cfg->deadband_mm) {
            pos_error = 0.0f;
            g_q4_position_integral *= 0.85f;
        } else {
            g_q4_position_integral += pos_error * dt_s;
            g_q4_position_integral = clamp_f32_ball(
                g_q4_position_integral,
                -pid_cfg->pos_int_limit, pid_cfg->pos_int_limit);
        }
        /*
         * 位置 PID 把球拉回中心；速度 PID 的目标速度固定为 0，专门抑制
         * 球继续滚动。两路先各自限幅再加权，避免视觉跳点独占全部倾角。
         */
        position_output = (pid_cfg->pos_kp * pos_error) +
            (pid_cfg->pos_ki * g_q4_position_integral) -
            (pid_cfg->pos_kd * control_velocity_mm_s);
        position_output = clamp_f32_ball(position_output,
            -pid_cfg->pos_out_max_deg, pid_cfg->pos_out_max_deg);

        speed_error = -control_velocity_mm_s;
        g_q4_speed_integral += speed_error * dt_s;
        g_q4_speed_integral = clamp_f32_ball(g_q4_speed_integral,
            -pid_cfg->speed_int_limit, pid_cfg->speed_int_limit);
        speed_derivative = (speed_error - g_q4_last_speed_error) / dt_s;
        speed_derivative = clamp_f32_ball(speed_derivative,
            -pid_cfg->accel_limit, pid_cfg->accel_limit);
        g_q4_last_speed_error = speed_error;
        speed_output = (pid_cfg->speed_kp * speed_error) +
            (pid_cfg->speed_ki * g_q4_speed_integral) +
            (pid_cfg->speed_kd * speed_derivative);
        speed_output = clamp_f32_ball(speed_output,
            -pid_cfg->speed_out_max_deg, pid_cfg->speed_out_max_deg);

        g_feedback_deg = -((pid_cfg->pos_weight * position_output) +
            (pid_cfg->speed_weight * speed_output));
        g_feedback_deg = clamp_f32_ball(g_feedback_deg,
            -pid_cfg->max_theta, pid_cfg->max_theta);
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
