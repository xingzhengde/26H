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
static float g_q6_straight_position_integral;
static float g_q6_straight_speed_integral;
static float g_q6_straight_last_speed_error;
static float g_last_raw_velocity_mm_s;
static float g_filtered_accel_mm_s2;
static float g_feedforward_deg;
static float g_curve_feedforward_deg;
static float g_feedback_deg;
static float g_feedback_scale;
static bool g_q4_hold_active;
static bool g_q6_hold_active;
static bool g_q7_hold_active;
static bool g_q3_mode5_active;
static float g_q6_straight_target_bias_mm;
static float g_q6_curve_target_bias_mm;
static float g_q7_straight_target_bias_mm;
static float g_q7_curve_target_bias_mm;
static bool g_q67_drive_accel_active;
static float g_q67_drive_accel_feedforward_deg;
static float g_q67_fast_accel_mm_s2;
static float g_q67_fast_jerk_mm_s3;
static uint8_t g_q67_accel_observer_samples;

typedef enum {
    Q67_STICTION_IDLE = 0,
    Q67_STICTION_QUALIFY,
    Q67_STICTION_PULSE,
    Q67_STICTION_COOLDOWN
} Q67StictionPhase;

static Q67StictionPhase g_q67_stiction_phase;
static uint32_t g_q67_stiction_phase_start_ms;
static int8_t g_q67_stiction_error_sign;

typedef struct {
    float mean_error_mm;
    float learned_bias_mm;
    uint32_t qualify_start_ms;
    bool qualifying;
} Q67AdaptiveBiasState;

/* Straight/curve and mode-3/mode-4 learners are intentionally independent. */
static Q67AdaptiveBiasState g_q6_straight_adaptive;
static Q67AdaptiveBiasState g_q6_curve_adaptive;
static Q67AdaptiveBiasState g_q7_straight_adaptive;
static Q67AdaptiveBiasState g_q7_curve_adaptive;

/* Mode-5 fixed sequence is private and never selected by modes 1-4. */
typedef enum {
    Q3_MODE5_PHASE_IDLE = 0,
    Q3_MODE5_PHASE_OUT_ACCEL,
    Q3_MODE5_PHASE_TRANSFER,
    Q3_MODE5_PHASE_FINAL_BRAKE,
    Q3_MODE5_PHASE_HOLD_MINUS
} Q3Mode5Phase;

static Q3Mode5Phase g_q3_mode5_phase;
static uint32_t g_q3_mode5_phase_start_ms;
static float g_q3_mode5_hold_integral_deg;
static Q3OpenLoopParams g_q3_mode5_pending_params;
static Q3OpenLoopParams g_q3_mode5_run_params;

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

static const BallPidConfig g_q7_pid = {
    Q7_BALL_POS_KP, Q7_BALL_POS_KI, Q7_BALL_POS_KD, Q7_BALL_POS_WEIGHT,
    Q7_BALL_SPEED_KP, Q7_BALL_SPEED_KI, Q7_BALL_SPEED_KD,
    Q7_BALL_SPEED_WEIGHT, Q7_BALL_PID_DEADBAND_MM,
    Q7_BALL_PID_INT_LIMIT, Q7_BALL_SPEED_INT_LIMIT,
    Q7_BALL_ACCEL_LIMIT_MM_S2, Q7_BALL_POSITION_FILTER_NEW,
    Q7_BALL_VELOCITY_FILTER_NEW, Q7_BALL_ACCEL_FILTER_NEW,
    Q7_BALL_MEDIAN_DELAY_FRAMES, Q7_BALL_ACTUATOR_LOOKAHEAD_MS,
    Q7_BALL_LOOKAHEAD_MAX_MS, Q7_BALL_PREDICT_POS_MAX_MM,
    Q7_BALL_PREDICT_SPEED_DELTA_MAX, Q7_BALL_POS_OUT_MAX_DEG,
    Q7_BALL_SPEED_OUT_MAX_DEG, Q7_BALL_PID_MAX_THETA
};

static const BallPidConfig g_q3_mode5_pid = {
    Q3_MODE5_HOLD_KP_DEG_PER_MM, Q3_MODE5_HOLD_KI_DEG_PER_MM_S,
    Q3_MODE5_HOLD_KD_DEG_PER_MM_S, 1.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, Q3_MODE5_HOLD_INT_MAX_ANGLE_DEG,
    Q3_MODE5_ACCEL_LIMIT_MM_S2, Q3_MODE5_POSITION_FILTER_NEW,
    Q3_MODE5_VELOCITY_FILTER_NEW, Q3_MODE5_ACCEL_FILTER_NEW,
    0U, 0U, 0U, 0.0f, 0.0f,
    Q3_MODE5_HOLD_MAX_ANGLE_DEG, Q3_MODE5_HOLD_MAX_ANGLE_DEG,
    Q3_MODE5_HOLD_MAX_ANGLE_DEG
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

static float q67_effective_drive_accel_feedforward(void)
{
    /* The proven fixed startup angle has priority; never double-compensate. */
    if (!g_q6_hold_active || !g_q67_drive_accel_active ||
        g_q6_curve_cascade_active ||
        (abs_f32(g_feedforward_deg) > 0.001f)) {
        return 0.0f;
    }
    return g_q67_drive_accel_feedforward_deg;
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
    float drive_accel_ff_deg = q67_effective_drive_accel_feedforward();
    float total_angle = g_feedforward_deg + g_curve_feedforward_deg +
        drive_accel_ff_deg + applied_feedback_deg;

    total_angle = clamp_f32_ball(total_angle,
        -BALL_SPEED_MAX_THETA, BALL_SPEED_MAX_THETA);
    g_state.feedforward_deg = g_feedforward_deg +
        g_curve_feedforward_deg + drive_accel_ff_deg;
    g_state.feedback_deg = applied_feedback_deg;
    g_state.theta_deg = total_angle;
    stepper_arm_set_pipe_angle(
        BALL_ACTUATOR_ANGLE_SIGN * total_angle,
        motor_rate_deg_s);
}

static void reset_q67_stiction_state(void)
{
    g_q67_stiction_phase = Q67_STICTION_IDLE;
    g_q67_stiction_phase_start_ms = 0U;
    g_q67_stiction_error_sign = 0;
}

static void reset_one_q67_adaptive_filter(Q67AdaptiveBiasState *state)
{
    state->mean_error_mm = 0.0f;
    state->qualify_start_ms = 0U;
    state->qualifying = false;
}

static void reset_q67_adaptive_filters(void)
{
    reset_one_q67_adaptive_filter(&g_q6_straight_adaptive);
    reset_one_q67_adaptive_filter(&g_q6_curve_adaptive);
    reset_one_q67_adaptive_filter(&g_q7_straight_adaptive);
    reset_one_q67_adaptive_filter(&g_q7_curve_adaptive);
}

static void reset_q67_adaptive_all(void)
{
    reset_q67_adaptive_filters();
    g_q6_straight_adaptive.learned_bias_mm = 0.0f;
    g_q6_curve_adaptive.learned_bias_mm = 0.0f;
    g_q7_straight_adaptive.learned_bias_mm = 0.0f;
    g_q7_curve_adaptive.learned_bias_mm = 0.0f;
}

static void reset_q67_accel_observer(void)
{
    g_q67_fast_accel_mm_s2 = 0.0f;
    g_q67_fast_jerk_mm_s3 = 0.0f;
    g_q67_accel_observer_samples = 0U;
}

/**
 * Use the newest velocity difference with a high new-sample weight. The jerk
 * estimate is retained so the controller can lead the camera/actuator delay.
 */
static void update_q67_accel_observer(float raw_accel_mm_s2, float dt_s)
{
    bool q7 = g_q7_hold_active;
    float observer_new = q7 ? Q7_VISUAL_ACCEL_OBSERVER_NEW :
                              Q6_VISUAL_ACCEL_OBSERVER_NEW;
    float jerk_limit = q7 ? Q7_VISUAL_ACCEL_JERK_LIMIT_MM_S3 :
                            Q6_VISUAL_ACCEL_JERK_LIMIT_MM_S3;
    float previous_accel;

    if (!g_q6_hold_active) {
        return;
    }
    if (g_q67_accel_observer_samples < 2U) {
        g_q67_accel_observer_samples++;
        g_q67_fast_accel_mm_s2 = 0.0f;
        g_q67_fast_jerk_mm_s3 = 0.0f;
        return;
    }

    previous_accel = g_q67_fast_accel_mm_s2;
    g_q67_fast_accel_mm_s2 =
        ((1.0f - observer_new) * previous_accel) +
        (observer_new * raw_accel_mm_s2);
    g_q67_fast_jerk_mm_s3 = clamp_f32_ball(
        (g_q67_fast_accel_mm_s2 - previous_accel) / dt_s,
        -jerk_limit, jerk_limit);
}

/**
 * Acceleration target is zero relative motion in the pipe. Positive visual
 * acceleration is toward the hinge, so a positive pipe angle opposes it.
 */
static float apply_q67_visual_accel_feedback(float feedback_deg,
                                              float lookahead_s,
                                              float angle_max_deg)
{
    bool q7 = g_q7_hold_active;
    bool enabled = q7 ? (Q7_VISUAL_ACCEL_FB_ENABLE != 0) :
                        (Q6_VISUAL_ACCEL_FB_ENABLE != 0);
    float kp = q7 ? Q7_VISUAL_ACCEL_FB_KP_DEG_PER_MM_S2 :
                    Q6_VISUAL_ACCEL_FB_KP_DEG_PER_MM_S2;
    float max_deg = q7 ? Q7_VISUAL_ACCEL_FB_MAX_DEG :
                         Q6_VISUAL_ACCEL_FB_MAX_DEG;
    float sign = q7 ? Q7_VISUAL_ACCEL_FB_SIGN :
                      Q6_VISUAL_ACCEL_FB_SIGN;
    uint32_t predict_max_ms = q7 ? Q7_VISUAL_ACCEL_PREDICT_MAX_MS :
                                   Q6_VISUAL_ACCEL_PREDICT_MAX_MS;
    float input_limit = q7 ? Q7_VISUAL_ACCEL_INPUT_LIMIT_MM_S2 :
                             Q6_VISUAL_ACCEL_INPUT_LIMIT_MM_S2;
    float predict_max_s = (float)predict_max_ms / 1000.0f;
    float predicted_accel_mm_s2;
    float accel_feedback_deg;

    if (!enabled || !g_q6_hold_active || !g_q67_drive_accel_active ||
        g_q6_curve_cascade_active || (g_feedback_scale <= 0.01f) ||
        (g_q67_accel_observer_samples < 2U)) {
        return feedback_deg;
    }
    if (lookahead_s > predict_max_s) {
        lookahead_s = predict_max_s;
    }
    predicted_accel_mm_s2 = g_q67_fast_accel_mm_s2 +
        (g_q67_fast_jerk_mm_s3 * lookahead_s);
    predicted_accel_mm_s2 = clamp_f32_ball(predicted_accel_mm_s2,
        -input_limit, input_limit);
    accel_feedback_deg = clamp_f32_ball(
        sign * kp * predicted_accel_mm_s2, -max_deg, max_deg);
    return clamp_f32_ball(feedback_deg + accel_feedback_deg,
        -angle_max_deg, angle_max_deg);
}

/**
 * Learn only the persistent mean visual error. Oscillation around the target
 * averages to zero, while an off-centre oscillation slowly shifts the target.
 */
static float update_q67_adaptive_bias(uint32_t now_ms, bool curve,
                                      float true_error_mm,
                                      float velocity_mm_s, float dt_s)
{
    bool q7 = g_q7_hold_active;
    bool enabled = q7 ? (Q7_ADAPTIVE_BIAS_ENABLE != 0) :
                        (Q6_ADAPTIVE_BIAS_ENABLE != 0);
    float deadband_mm = q7 ? Q7_ADAPTIVE_BIAS_DEADBAND_MM :
                             Q6_ADAPTIVE_BIAS_DEADBAND_MM;
    float max_bias_mm = q7 ? Q7_ADAPTIVE_BIAS_MAX_MM :
                             Q6_ADAPTIVE_BIAS_MAX_MM;
    float rate_per_s = q7 ? Q7_ADAPTIVE_BIAS_RATE_PER_S :
                            Q6_ADAPTIVE_BIAS_RATE_PER_S;
    uint32_t tau_ms = q7 ? Q7_ADAPTIVE_BIAS_MEAN_TAU_MS :
                           Q6_ADAPTIVE_BIAS_MEAN_TAU_MS;
    uint32_t qualify_ms = q7 ? Q7_ADAPTIVE_BIAS_QUALIFY_MS :
                               Q6_ADAPTIVE_BIAS_QUALIFY_MS;
    float position_window_mm = q7 ?
        Q7_ADAPTIVE_BIAS_POSITION_WINDOW_MM :
        Q6_ADAPTIVE_BIAS_POSITION_WINDOW_MM;
    float speed_max_mm_s = q7 ? Q7_ADAPTIVE_BIAS_SPEED_MAX_MM_S :
                                Q6_ADAPTIVE_BIAS_SPEED_MAX_MM_S;
    Q67AdaptiveBiasState *state;
    float tau_s;
    float alpha;

    if (q7) {
        state = curve ? &g_q7_curve_adaptive : &g_q7_straight_adaptive;
    } else {
        state = curve ? &g_q6_curve_adaptive : &g_q6_straight_adaptive;
    }

    if (!enabled || !g_q6_hold_active || (g_feedback_scale < 0.95f) ||
        (g_q67_stiction_phase == Q67_STICTION_PULSE) ||
        (abs_f32(true_error_mm) > position_window_mm)) {
        reset_one_q67_adaptive_filter(state);
        return state->learned_bias_mm;
    }

    /* Keep the existing mean through fast crossings; learn near turnarounds. */
    if (abs_f32(velocity_mm_s) > speed_max_mm_s) {
        return state->learned_bias_mm;
    }

    if (!state->qualifying) {
        state->qualifying = true;
        state->qualify_start_ms = now_ms;
        state->mean_error_mm = true_error_mm;
    }

    tau_s = (float)tau_ms / 1000.0f;
    alpha = dt_s / (tau_s + dt_s);
    state->mean_error_mm +=
        alpha * (true_error_mm - state->mean_error_mm);

    if (((now_ms - state->qualify_start_ms) >= qualify_ms) &&
        (abs_f32(state->mean_error_mm) > deadband_mm)) {
        state->learned_bias_mm +=
            rate_per_s * state->mean_error_mm * dt_s;
        state->learned_bias_mm = clamp_f32_ball(
            state->learned_bias_mm, -max_bias_mm, max_bias_mm);
    }
    return state->learned_bias_mm;
}

/**
 * @brief 给模式三/四的串级PID叠加短时破静摩擦脉冲。
 *
 * 只有误差持续存在且小球速度很低时才触发。脉冲方向始终与PID回目标方向
 * 一致；一旦小球开始移动立即进入冷却，避免固定增益造成穿越中心后摆动。
 */
static float apply_q67_stiction_boost(uint32_t now_ms, float position_error_mm,
                                      float velocity_mm_s,
                                      float feedback_deg, float angle_max_deg)
{
    bool q7 = g_q7_hold_active;
    bool enabled = q7 ? (Q7_STICTION_ENABLE != 0) :
                        (Q6_STICTION_ENABLE != 0);
    float error_enter_mm = q7 ? Q7_STICTION_ERROR_ENTER_MM :
                                Q6_STICTION_ERROR_ENTER_MM;
    float error_exit_mm = q7 ? Q7_STICTION_ERROR_EXIT_MM :
                               Q6_STICTION_ERROR_EXIT_MM;
    float speed_enter_mm_s = q7 ? Q7_STICTION_SPEED_ENTER_MM_S :
                                  Q6_STICTION_SPEED_ENTER_MM_S;
    float speed_release_mm_s = q7 ? Q7_STICTION_SPEED_RELEASE_MM_S :
                                    Q6_STICTION_SPEED_RELEASE_MM_S;
    uint32_t stable_ms = q7 ? Q7_STICTION_STABLE_MS :
                              Q6_STICTION_STABLE_MS;
    uint32_t pulse_ms = q7 ? Q7_STICTION_PULSE_MS :
                             Q6_STICTION_PULSE_MS;
    uint32_t cooldown_ms = q7 ? Q7_STICTION_COOLDOWN_MS :
                                Q6_STICTION_COOLDOWN_MS;
    float boost_deg = q7 ? Q7_STICTION_BOOST_DEG :
                           Q6_STICTION_BOOST_DEG;
    float abs_error_mm = abs_f32(position_error_mm);
    float abs_velocity_mm_s = abs_f32(velocity_mm_s);
    int8_t error_sign = (position_error_mm > 0.0f) ? 1 : -1;
    bool stuck = (abs_error_mm >= error_enter_mm) &&
                 (abs_velocity_mm_s <= speed_enter_mm_s);

    if (!enabled || !g_q6_hold_active || (g_feedback_scale < 0.95f) ||
        (abs_error_mm <= error_exit_mm)) {
        reset_q67_stiction_state();
        return feedback_deg;
    }

    if (g_q67_stiction_phase == Q67_STICTION_IDLE) {
        if (stuck) {
            g_q67_stiction_phase = Q67_STICTION_QUALIFY;
            g_q67_stiction_phase_start_ms = now_ms;
            g_q67_stiction_error_sign = error_sign;
        }
    } else if (g_q67_stiction_phase == Q67_STICTION_QUALIFY) {
        if (!stuck || (error_sign != g_q67_stiction_error_sign)) {
            reset_q67_stiction_state();
        } else if ((now_ms - g_q67_stiction_phase_start_ms) >= stable_ms) {
            g_q67_stiction_phase = Q67_STICTION_PULSE;
            g_q67_stiction_phase_start_ms = now_ms;
        }
    } else if (g_q67_stiction_phase == Q67_STICTION_PULSE) {
        if ((error_sign != g_q67_stiction_error_sign) ||
            (abs_velocity_mm_s >= speed_release_mm_s) ||
            ((now_ms - g_q67_stiction_phase_start_ms) >= pulse_ms)) {
            g_q67_stiction_phase = Q67_STICTION_COOLDOWN;
            g_q67_stiction_phase_start_ms = now_ms;
        }
    } else if ((now_ms - g_q67_stiction_phase_start_ms) >= cooldown_ms) {
        reset_q67_stiction_state();
    }

    if (g_q67_stiction_phase == Q67_STICTION_PULSE) {
        /* Cascade feedback sign is opposite to position-error sign. */
        feedback_deg -= (float)g_q67_stiction_error_sign * boost_deg;
    }
    return clamp_f32_ball(feedback_deg, -angle_max_deg, angle_max_deg);
}

static void apply_pipe_command(void)
{
    apply_pipe_command_with_rate(BALL_THETA_RATE_DEG);
}

/**
 * @brief 清除模式五私有时序，防止切换到其它模式后残留固定倾角。
 */
static void reset_q3_mode5_state(void)
{
    g_q3_mode5_active = false;
    g_q3_mode5_phase = Q3_MODE5_PHASE_IDLE;
    g_q3_mode5_phase_start_ms = 0U;
    g_q3_mode5_hold_integral_deg = 0.0f;
}

static void load_default_q3_mode5_params(void)
{
    g_q3_mode5_pending_params.angle_deg[0] = Q3_MODE5_OUT_ACCEL_ANGLE_DEG;
    g_q3_mode5_pending_params.time_ms[0] = Q3_MODE5_OUT_ACCEL_TIME_MS;
    g_q3_mode5_pending_params.angle_deg[1] = Q3_MODE5_TRANSFER_ANGLE_DEG;
    g_q3_mode5_pending_params.time_ms[1] = Q3_MODE5_TRANSFER_TIME_MS;
    g_q3_mode5_pending_params.angle_deg[2] = Q3_MODE5_FINAL_BRAKE_ANGLE_DEG;
    g_q3_mode5_pending_params.time_ms[2] = Q3_MODE5_FINAL_BRAKE_TIME_MS;
    g_q3_mode5_run_params = g_q3_mode5_pending_params;
}

bool ball_balance_set_q3_open_loop_params(const Q3OpenLoopParams *params)
{
    uint8_t stage;
    uint32_t total_ms = 0U;

    if ((params == 0) || g_q3_mode5_active) {
        return false;
    }
    for (stage = 0U; stage < 3U; stage++) {
        if ((params->angle_deg[stage] < Q3_MODE5_TUNE_ANGLE_MIN_DEG) ||
            (params->angle_deg[stage] > Q3_MODE5_TUNE_ANGLE_MAX_DEG) ||
            (params->time_ms[stage] < Q3_MODE5_TUNE_TIME_MIN_MS) ||
            (params->time_ms[stage] > Q3_MODE5_TUNE_TIME_MAX_MS)) {
            return false;
        }
        total_ms += params->time_ms[stage];
    }
    if (total_ms > Q3_MODE5_TUNE_TOTAL_MAX_MS) {
        return false;
    }
    g_q3_mode5_pending_params = *params;
    return true;
}

/**
 * @brief 进入模式五的一段固定轨迹，并立即把绝对位置帧发给X42S。
 *
 * 参考工程的逻辑角先乘模式五私有方向映射，不修改全局执行机构方向，
 * 因此模式二、三、四的倾角方向和参数均保持原样。
 */
static bool enter_q3_mode5_phase(Q3Mode5Phase phase, uint32_t now_ms)
{
    float reference_angle_deg = 0.0f;

    g_q3_mode5_phase = phase;
    g_q3_mode5_phase_start_ms = now_ms;
    g_feedback_deg = 0.0f;
    g_speed_integral = 0.0f;

    switch (phase) {
    case Q3_MODE5_PHASE_OUT_ACCEL:
        g_state.target_mm = Q3_MODE5_TARGET_PLUS_MM;
        reference_angle_deg = g_q3_mode5_run_params.angle_deg[0];
        break;
    case Q3_MODE5_PHASE_TRANSFER:
        g_state.target_mm = Q3_MODE5_TARGET_MINUS_MM;
        reference_angle_deg = g_q3_mode5_run_params.angle_deg[1];
        break;
    case Q3_MODE5_PHASE_FINAL_BRAKE:
        g_state.target_mm = Q3_MODE5_TARGET_MINUS_MM;
        reference_angle_deg = g_q3_mode5_run_params.angle_deg[2];
        break;
    case Q3_MODE5_PHASE_HOLD_MINUS:
        g_state.target_mm = Q3_MODE5_TARGET_MINUS_MM;
        g_q3_mode5_hold_integral_deg = 0.0f;
        reference_angle_deg = 0.0f;
        break;
    default:
        g_state.target_mm = 0.0f;
        reference_angle_deg = 0.0f;
        break;
    }

    g_feedforward_deg = Q3_MODE5_REFERENCE_ANGLE_SIGN *
        reference_angle_deg;
    apply_pipe_command_with_rate(Q3_MODE5_OPEN_LOOP_MOTOR_DPS);
    return stepper_arm_send_pending_now(now_ms);
}

/**
 * @brief 按参考工程的800/1600/450 ms固定时序推进模式五。
 */
static void service_q3_mode5_timeline(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (!g_q3_mode5_active) {
        return;
    }
    elapsed_ms = now_ms - g_q3_mode5_phase_start_ms;
    if ((g_q3_mode5_phase == Q3_MODE5_PHASE_OUT_ACCEL) &&
        (elapsed_ms >= g_q3_mode5_run_params.time_ms[0])) {
        (void)enter_q3_mode5_phase(Q3_MODE5_PHASE_TRANSFER, now_ms);
    } else if ((g_q3_mode5_phase == Q3_MODE5_PHASE_TRANSFER) &&
               (elapsed_ms >= g_q3_mode5_run_params.time_ms[1])) {
        (void)enter_q3_mode5_phase(Q3_MODE5_PHASE_FINAL_BRAKE, now_ms);
    } else if ((g_q3_mode5_phase == Q3_MODE5_PHASE_FINAL_BRAKE) &&
               (elapsed_ms >= g_q3_mode5_run_params.time_ms[2])) {
        (void)enter_q3_mode5_phase(Q3_MODE5_PHASE_HOLD_MINUS, now_ms);
    }
}

/**
 * @brief 清除与一次控制任务相关的滤波和积分状态。
 *
 * 每次切换目标都重新初始化，防止上一任务的速度积分把球推向端部。
 */
static void reset_loop_state(void)
{
    reset_q67_stiction_state();
    reset_q67_adaptive_all();
    reset_q67_accel_observer();
    g_q67_drive_accel_active = false;
    g_q67_drive_accel_feedforward_deg = 0.0f;
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
    g_q6_straight_position_integral = 0.0f;
    g_q6_straight_speed_integral = 0.0f;
    g_q6_straight_last_speed_error = 0.0f;
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
    g_q7_hold_active = false;
    g_q6_straight_target_bias_mm = Q6_STRAIGHT_TARGET_BIAS_MM;
    g_q6_curve_target_bias_mm = Q6_CURVE_TARGET_BIAS_MM;
    g_q7_straight_target_bias_mm = Q7_STRAIGHT_TARGET_BIAS_MM;
    g_q7_curve_target_bias_mm = Q7_CURVE_TARGET_BIAS_MM;
    load_default_q3_mode5_params();
    reset_q3_mode5_state();
    g_state.mode = BALL_MODE_IDLE;
    g_state.target_mm = 0.0f;
    g_state.sample_ok = false;
    reset_loop_state();
}

void ball_balance_start_sequence(void)
{
    g_q4_hold_active = false;
    g_q6_hold_active = false;
    g_q7_hold_active = false;
    reset_q3_mode5_state();
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
    g_q7_hold_active = false;
    reset_q3_mode5_state();
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
    g_q7_hold_active = false;
    reset_q3_mode5_state();
    g_curve_feedforward_deg = 0.0f;
    g_feedback_scale = 1.0f;
    g_state.mode = BALL_MODE_HOLD_TARGET;
    g_state.target_mm = target_mm;
    reset_loop_state();
}

bool ball_balance_set_q67_target_biases(float q6_straight_mm,
                                        float q6_curve_mm,
                                        float q7_straight_mm,
                                        float q7_curve_mm)
{
    if (g_q6_hold_active || g_q7_hold_active ||
        (q6_straight_mm < Q67_TARGET_BIAS_TUNE_MIN_MM) ||
        (q6_straight_mm > Q67_TARGET_BIAS_TUNE_MAX_MM) ||
        (q6_curve_mm < Q67_TARGET_BIAS_TUNE_MIN_MM) ||
        (q6_curve_mm > Q67_TARGET_BIAS_TUNE_MAX_MM) ||
        (q7_straight_mm < Q67_TARGET_BIAS_TUNE_MIN_MM) ||
        (q7_straight_mm > Q67_TARGET_BIAS_TUNE_MAX_MM) ||
        (q7_curve_mm < Q67_TARGET_BIAS_TUNE_MIN_MM) ||
        (q7_curve_mm > Q67_TARGET_BIAS_TUNE_MAX_MM)) {
        return false;
    }
    g_q6_straight_target_bias_mm = q6_straight_mm;
    g_q6_curve_target_bias_mm = q6_curve_mm;
    g_q7_straight_target_bias_mm = q7_straight_mm;
    g_q7_curve_target_bias_mm = q7_curve_mm;
    return true;
}

void ball_balance_hold_q6(float target_mm)
{
    g_q4_hold_active = true;
    g_q6_hold_active = true;
    g_q7_hold_active = false;
    reset_q3_mode5_state();
    g_curve_feedforward_deg = 0.0f;
    g_feedback_scale = 0.0f;
    g_state.mode = BALL_MODE_HOLD_TARGET;
    g_state.target_mm = target_mm;
    reset_loop_state();
}

void ball_balance_hold_q7(float target_mm)
{
    g_q4_hold_active = true;
    g_q6_hold_active = true;
    g_q7_hold_active = true;
    reset_q3_mode5_state();
    g_curve_feedforward_deg = 0.0f;
    g_feedback_scale = 0.0f;
    g_state.mode = BALL_MODE_HOLD_TARGET;
    g_state.target_mm = target_mm;
    reset_loop_state();
}

bool ball_balance_start_q3(uint32_t now_ms)
{
    bool command_sent;

    g_q4_hold_active = false;
    g_q6_hold_active = false;
    g_q7_hold_active = false;
    /* Freeze the six LCD values for this complete three-stage run. */
    g_q3_mode5_run_params = g_q3_mode5_pending_params;
    g_q3_mode5_active = true;
    g_feedforward_deg = 0.0f;
    g_curve_feedforward_deg = 0.0f;
    g_feedback_scale = 1.0f;
    g_state.mode = BALL_MODE_HOLD_TARGET;
    g_state.target_mm = Q3_MODE5_TARGET_PLUS_MM;
    reset_loop_state();
    command_sent = enter_q3_mode5_phase(
        Q3_MODE5_PHASE_OUT_ACCEL, now_ms);
    if (!command_sent) {
        ball_balance_stop();
        return false;
    }
    return true;
}

void ball_balance_set_feedforward(float pipe_angle_deg)
{
    float max_deg = g_q7_hold_active ? Q7_MAX_FEEDFORWARD_DEG :
        (g_q6_hold_active ? Q6_MAX_FEEDFORWARD_DEG :
         Q4_MAX_FEEDFORWARD_DEG);

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
    {
        float max_deg = g_q7_hold_active ? Q7_CURVE_FF_MAX_DEG :
            Q6_CURVE_FF_MAX_DEG;
        g_curve_feedforward_deg = clamp_f32_ball(pipe_angle_deg,
            -max_deg, max_deg);
    }
    cascade_active = abs_f32(g_curve_feedforward_deg) > 0.02f;
    if (cascade_active != g_q6_curve_cascade_active) {
        /* Reset both controllers at curve transitions to avoid stale I terms. */
        g_q4_position_integral = 0.0f;
        g_q4_speed_integral = 0.0f;
        g_q4_last_speed_error = 0.0f;
        g_q6_curve_position_integral = 0.0f;
        g_q6_curve_speed_integral = 0.0f;
        g_q6_curve_last_speed_error = 0.0f;
        g_q6_straight_position_integral = 0.0f;
        g_q6_straight_speed_integral = 0.0f;
        g_q6_straight_last_speed_error = 0.0f;
        reset_q67_stiction_state();
        reset_q67_adaptive_filters();
        g_q6_curve_cascade_active = cascade_active;
    }
    /* 弯道前馈独立叠加，不触发启动阶段的 PID 冻结。 */
    apply_pipe_command();
}

void ball_balance_set_drive_accel_phase(bool active,
                                        float planned_accel_mps2,
                                        float ramp_ratio,
                                        float startup_angle_deg)
{
    bool q7 = g_q7_hold_active;
    float gain = q7 ? Q7_DRIVE_ACCEL_FF_GAIN_DEG_PER_MPS2 :
                      Q6_DRIVE_ACCEL_FF_GAIN_DEG_PER_MPS2;
    float model_max_deg = q7 ? Q7_DRIVE_ACCEL_FF_MAX_DEG :
                               Q6_DRIVE_ACCEL_FF_MAX_DEG;
    float total_max_deg = q7 ? Q7_MAX_FEEDFORWARD_DEG :
                               Q6_MAX_FEEDFORWARD_DEG;
    float sign = q7 ? Q7_DRIVE_ACCEL_FF_SIGN :
                      Q6_DRIVE_ACCEL_FF_SIGN;
    float model_feedforward_deg;
    float decay_ratio;

    g_q67_drive_accel_active = g_q6_hold_active && active;
    if (g_q67_drive_accel_active) {
        model_feedforward_deg = clamp_f32_ball(
            sign * gain * planned_accel_mps2,
            -model_max_deg, model_max_deg);
        ramp_ratio = clamp_f32_ball(ramp_ratio, 0.0f, 1.0f);
        decay_ratio = 1.0f - ramp_ratio;
        /* Strong angle disappears early; the model term remains to ramp end. */
        decay_ratio = decay_ratio * decay_ratio * decay_ratio;
        startup_angle_deg = clamp_f32_ball(startup_angle_deg,
            -total_max_deg, total_max_deg);
        /* Continue startup compensation through the full acceleration ramp. */
        g_q67_drive_accel_feedforward_deg = model_feedforward_deg +
            ((startup_angle_deg - model_feedforward_deg) * decay_ratio);
    } else {
        g_q67_drive_accel_feedforward_deg = 0.0f;
    }
    if (g_q4_hold_active) {
        /* Apply immediately; do not wait for the next K230 vision frame. */
        apply_pipe_command();
    }
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
    float max_deg = g_q7_hold_active ? Q7_MAX_FEEDFORWARD_DEG :
        (g_q6_hold_active ? Q6_MAX_FEEDFORWARD_DEG :
         Q4_MAX_FEEDFORWARD_DEG);
    float rate_deg_s = g_q7_hold_active ? Q7_INITIAL_ANGLE_RATE_DEG_S :
        (g_q6_hold_active ? Q6_INITIAL_ANGLE_RATE_DEG_S :
         Q4_INITIAL_ANGLE_RATE_DEG_S);

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
    g_q7_hold_active = false;
    reset_q3_mode5_state();
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
    float true_target_error_mm;
    float control_position_mm;
    float control_velocity_mm_s;
    float lookahead_s;
    float predicted_position_delta;
    float predicted_speed_delta;
    float target_velocity;
    float speed_error;
    uint32_t sample_dt_ms;
    const BallPidConfig *pid_cfg = g_q3_mode5_active ? &g_q3_mode5_pid :
        (g_q7_hold_active ? &g_q7_pid :
         (g_q6_hold_active ? &g_q6_pid : &g_q4_pid));

    g_state.sample_ok = sample_ok;
    if ((g_state.mode == BALL_MODE_IDLE) ||
        (g_state.mode == BALL_MODE_DONE)) {
        return;
    }

    /* M5固定轨迹只依赖毫秒时基，不等待K230目标切换或新视觉帧。 */
    service_q3_mode5_timeline(now_ms);

    /*
     * k230_ball_get_sample() 在数据超过 180ms 或置信度不足时返回 false。
     * 此时撤掉闭环修正但保留小车加速度前馈，避免继续沿错误方向积分。
     */
    if (!sample_ok || (sample == 0)) {
        reset_q67_stiction_state();
        reset_q67_adaptive_filters();
        reset_q67_accel_observer();
        g_last_sample_timestamp_ms = 0U;
        g_have_frame_meta = false;
        g_speed_integral = 0.0f;
        g_q4_position_integral = 0.0f;
        g_q4_speed_integral = 0.0f;
        g_q4_last_speed_error = 0.0f;
        g_last_raw_velocity_mm_s = 0.0f;
        g_filtered_accel_mm_s2 = 0.0f;
        g_q3_mode5_hold_integral_deg = 0.0f;
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
        reset_q67_adaptive_filters();
        reset_q67_accel_observer();
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
        g_q3_mode5_hold_integral_deg = 0.0f;
        g_feedback_deg = 0.0f;
        apply_pipe_command();
        return;
    }

    raw_velocity = (sample->x_mm - g_last_raw_pos_mm) / dt_s;
    raw_accel = (raw_velocity - g_last_raw_velocity_mm_s) / dt_s;
    raw_accel = clamp_f32_ball(raw_accel,
        -pid_cfg->accel_limit, pid_cfg->accel_limit);
    update_q67_accel_observer(raw_accel, dt_s);
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
    true_target_error_mm = pos_error;

    if (g_q3_mode5_active) {
        float hold_error_mm;
        float reference_hold_angle_deg;

        if ((g_q3_mode5_phase != Q3_MODE5_PHASE_HOLD_MINUS) ||
            (Q3_MODE5_FINAL_PID_ENABLE == 0)) {
            /*
             * 纯开环调参版：前三段只执行固定角度；第三段结束后
             * enter_q3_mode5_phase()已把前馈清零，此处继续强制反馈为零。
             * K230位置/速度仍更新，但绝不会转化为步进电机角度。
             */
            g_feedback_deg = 0.0f;
            apply_pipe_command_with_rate(Q3_MODE5_OPEN_LOOP_MOTOR_DPS);
            return;
        }

        /*
         * 固定三段结束后才接入参考项目的最终位置PD。积分只在目标附近
         * 工作且直接按角度限幅，防止再次形成-4 cm到-7 cm的来回摆动。
         */
        hold_error_mm = Q3_MODE5_TARGET_MINUS_MM - g_state.position_mm;
        if (abs_f32(hold_error_mm) <= Q3_MODE5_HOLD_INTEGRAL_ZONE_MM) {
            g_q3_mode5_hold_integral_deg +=
                Q3_MODE5_HOLD_KI_DEG_PER_MM_S * hold_error_mm * dt_s;
            g_q3_mode5_hold_integral_deg = clamp_f32_ball(
                g_q3_mode5_hold_integral_deg,
                -Q3_MODE5_HOLD_INT_MAX_ANGLE_DEG,
                Q3_MODE5_HOLD_INT_MAX_ANGLE_DEG);
        } else {
            g_q3_mode5_hold_integral_deg = 0.0f;
        }
        reference_hold_angle_deg =
            (Q3_MODE5_HOLD_KP_DEG_PER_MM * hold_error_mm) +
            g_q3_mode5_hold_integral_deg -
            (Q3_MODE5_HOLD_KD_DEG_PER_MM_S * g_state.velocity_mm_s);
        reference_hold_angle_deg = clamp_f32_ball(
            reference_hold_angle_deg,
            -Q3_MODE5_HOLD_MAX_ANGLE_DEG,
            Q3_MODE5_HOLD_MAX_ANGLE_DEG);
        g_feedforward_deg = 0.0f;
        g_feedback_deg = Q3_MODE5_REFERENCE_ANGLE_SIGN *
            reference_hold_angle_deg;
        apply_pipe_command_with_rate(Q3_MODE5_HOLD_MOTOR_DPS);
        return;
    }

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
            g_q6_straight_position_integral = 0.0f;
            g_q6_straight_speed_integral = 0.0f;
            g_q6_straight_last_speed_error = 0.0f;
            g_feedback_deg = 0.0f;
            apply_pipe_command();
            return;
        }

        if (q6_curve_control) {
            float target_velocity_mm_s;
            float pos_kp = g_q7_hold_active ?
                Q7_CURVE_CASCADE_POS_KP : Q6_CURVE_CASCADE_POS_KP;
            float pos_ki = g_q7_hold_active ?
                Q7_CURVE_CASCADE_POS_KI : Q6_CURVE_CASCADE_POS_KI;
            float pos_kd = g_q7_hold_active ?
                Q7_CURVE_CASCADE_POS_KD : Q6_CURVE_CASCADE_POS_KD;
            float pos_int_limit = g_q7_hold_active ?
                Q7_CURVE_CASCADE_POS_INT_LIMIT :
                Q6_CURVE_CASCADE_POS_INT_LIMIT;
            float target_speed_max = g_q7_hold_active ?
                Q7_CURVE_CASCADE_TARGET_SPEED_MAX_MM_S :
                Q6_CURVE_CASCADE_TARGET_SPEED_MAX_MM_S;
            float speed_kp = g_q7_hold_active ?
                Q7_CURVE_CASCADE_SPEED_KP : Q6_CURVE_CASCADE_SPEED_KP;
            float speed_ki = g_q7_hold_active ?
                Q7_CURVE_CASCADE_SPEED_KI : Q6_CURVE_CASCADE_SPEED_KI;
            float speed_kd = g_q7_hold_active ?
                Q7_CURVE_CASCADE_SPEED_KD : Q6_CURVE_CASCADE_SPEED_KD;
            float speed_int_limit = g_q7_hold_active ?
                Q7_CURVE_CASCADE_SPEED_INT_LIMIT :
                Q6_CURVE_CASCADE_SPEED_INT_LIMIT;
            float angle_max = g_q7_hold_active ?
                Q7_CURVE_CASCADE_ANGLE_MAX_DEG :
                Q6_CURVE_CASCADE_ANGLE_MAX_DEG;
            float deadband_mm = g_q7_hold_active ?
                Q7_CURVE_CASCADE_DEADBAND_MM :
                Q6_CURVE_CASCADE_DEADBAND_MM;
            float target_bias_mm = g_q7_hold_active ?
                g_q7_curve_target_bias_mm : g_q6_curve_target_bias_mm;
            float adaptive_bias_mm = update_q67_adaptive_bias(
                now_ms, true, true_target_error_mm,
                control_velocity_mm_s, dt_s);

            /* Manual trim plus a slow learner for the measured mean offset. */
            pos_error = true_target_error_mm + target_bias_mm +
                adaptive_bias_mm;

            /* Position outer loop: position error becomes target ball speed. */
            if (abs_f32(pos_error) <= deadband_mm) {
                pos_error = 0.0f;
                g_q6_curve_position_integral *= 0.85f;
            } else {
                g_q6_curve_position_integral += pos_error * dt_s;
                g_q6_curve_position_integral = clamp_f32_ball(
                    g_q6_curve_position_integral,
                    -pos_int_limit, pos_int_limit);
            }
            target_velocity_mm_s =
                (pos_kp * pos_error) +
                (pos_ki *
                    g_q6_curve_position_integral) -
                (pos_kd * control_velocity_mm_s);
            target_velocity_mm_s = clamp_f32_ball(target_velocity_mm_s,
                -target_speed_max, target_speed_max);

            /* Speed inner loop: target speed becomes the pipe angle command. */
            speed_error = target_velocity_mm_s - control_velocity_mm_s;
            g_q6_curve_speed_integral += speed_error * dt_s;
            g_q6_curve_speed_integral = clamp_f32_ball(
                g_q6_curve_speed_integral,
                -speed_int_limit, speed_int_limit);
            speed_derivative =
                (speed_error - g_q6_curve_last_speed_error) / dt_s;
            speed_derivative = clamp_f32_ball(speed_derivative,
                -pid_cfg->accel_limit, pid_cfg->accel_limit);
            g_q6_curve_last_speed_error = speed_error;
            g_feedback_deg = -(
                (speed_kp * speed_error) +
                (speed_ki *
                    g_q6_curve_speed_integral) +
                (speed_kd * speed_derivative));
            g_feedback_deg = apply_q67_stiction_boost(
                now_ms, pos_error, control_velocity_mm_s,
                g_feedback_deg, angle_max);
            apply_pipe_command();
            return;
        }

        if (g_q6_hold_active) {
            float target_velocity_mm_s;
            float pos_kp = g_q7_hold_active ?
                Q7_STRAIGHT_CASCADE_POS_KP : Q6_STRAIGHT_CASCADE_POS_KP;
            float pos_ki = g_q7_hold_active ?
                Q7_STRAIGHT_CASCADE_POS_KI : Q6_STRAIGHT_CASCADE_POS_KI;
            float pos_kd = g_q7_hold_active ?
                Q7_STRAIGHT_CASCADE_POS_KD : Q6_STRAIGHT_CASCADE_POS_KD;
            float pos_int_limit = g_q7_hold_active ?
                Q7_STRAIGHT_CASCADE_POS_INT_LIMIT :
                Q6_STRAIGHT_CASCADE_POS_INT_LIMIT;
            float target_speed_max = g_q7_hold_active ?
                Q7_STRAIGHT_CASCADE_TARGET_SPEED_MAX_MM_S :
                Q6_STRAIGHT_CASCADE_TARGET_SPEED_MAX_MM_S;
            float speed_kp = g_q7_hold_active ?
                Q7_STRAIGHT_CASCADE_SPEED_KP : Q6_STRAIGHT_CASCADE_SPEED_KP;
            float speed_ki = g_q7_hold_active ?
                Q7_STRAIGHT_CASCADE_SPEED_KI : Q6_STRAIGHT_CASCADE_SPEED_KI;
            float speed_kd = g_q7_hold_active ?
                Q7_STRAIGHT_CASCADE_SPEED_KD : Q6_STRAIGHT_CASCADE_SPEED_KD;
            float speed_int_limit = g_q7_hold_active ?
                Q7_STRAIGHT_CASCADE_SPEED_INT_LIMIT :
                Q6_STRAIGHT_CASCADE_SPEED_INT_LIMIT;
            float angle_max = g_q7_hold_active ?
                Q7_STRAIGHT_CASCADE_ANGLE_MAX_DEG :
                Q6_STRAIGHT_CASCADE_ANGLE_MAX_DEG;
            float deadband_mm = g_q7_hold_active ?
                Q7_STRAIGHT_CASCADE_DEADBAND_MM :
                Q6_STRAIGHT_CASCADE_DEADBAND_MM;
            float target_bias_mm = g_q7_hold_active ?
                g_q7_straight_target_bias_mm :
                g_q6_straight_target_bias_mm;
            float adaptive_bias_mm = update_q67_adaptive_bias(
                now_ms, false, true_target_error_mm,
                control_velocity_mm_s, dt_s);

            /* Manual trim plus an independent straight-line mean learner. */
            pos_error = true_target_error_mm + target_bias_mm +
                adaptive_bias_mm;

            /* Mode-3 straight position outer loop: command a gentle return speed. */
            if (abs_f32(pos_error) <= deadband_mm) {
                pos_error = 0.0f;
                g_q6_straight_position_integral *= 0.85f;
            } else {
                g_q6_straight_position_integral += pos_error * dt_s;
                g_q6_straight_position_integral = clamp_f32_ball(
                    g_q6_straight_position_integral,
                    -pos_int_limit, pos_int_limit);
            }
            target_velocity_mm_s =
                (pos_kp * pos_error) +
                (pos_ki *
                    g_q6_straight_position_integral) -
                (pos_kd * control_velocity_mm_s);
            target_velocity_mm_s = clamp_f32_ball(target_velocity_mm_s,
                -target_speed_max, target_speed_max);

            /* Mode-3 straight speed inner loop: convert speed error to angle. */
            speed_error = target_velocity_mm_s - control_velocity_mm_s;
            if ((speed_error * g_q6_straight_last_speed_error) < 0.0f) {
                /* Never carry braking torque into the next half-cycle. */
                g_q6_straight_speed_integral = 0.0f;
            } else {
                /* Small leakage prevents a static speed bias storing energy. */
                g_q6_straight_speed_integral *= 0.995f;
            }
            g_q6_straight_speed_integral += speed_error * dt_s;
            g_q6_straight_speed_integral = clamp_f32_ball(
                g_q6_straight_speed_integral,
                -speed_int_limit, speed_int_limit);
            speed_derivative =
                (speed_error - g_q6_straight_last_speed_error) / dt_s;
            speed_derivative = clamp_f32_ball(speed_derivative,
                -pid_cfg->accel_limit, pid_cfg->accel_limit);
            g_q6_straight_last_speed_error = speed_error;
            g_feedback_deg = -(
                (speed_kp * speed_error) +
                (speed_ki *
                    g_q6_straight_speed_integral) +
                (speed_kd * speed_derivative));
            g_feedback_deg = apply_q67_visual_accel_feedback(
                g_feedback_deg, lookahead_s, angle_max);
            g_feedback_deg = apply_q67_stiction_boost(
                now_ms, pos_error, control_velocity_mm_s,
                g_feedback_deg, angle_max);
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

uint8_t ball_balance_get_q3_report_phase(void)
{
    switch (g_q3_mode5_phase) {
    case Q3_MODE5_PHASE_OUT_ACCEL:
        return 1U; /* K230 SEQ_TO_POSITIVE */
    case Q3_MODE5_PHASE_TRANSFER:
    case Q3_MODE5_PHASE_FINAL_BRAKE:
        return 2U; /* K230 SEQ_TO_NEGATIVE */
    case Q3_MODE5_PHASE_HOLD_MINUS:
        return 3U; /* K230 SEQ_HOLD_NEGATIVE */
    default:
        return 0U; /* K230 SEQ_WAIT_CENTER */
    }
}
