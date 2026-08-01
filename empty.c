#include "app_config.h"
#include "ball_balance.h"
#include "encoder.h"
#include "gray_sensor.h"
#include "k230_ball.h"
#include "line_tracker.h"
#include "motion_state.h"
#include "motor.h"
#include "oled.h"
#include "pid.h"
#include "q4_motion.h"
#include "stepper_arm.h"
#include "ti_msp_dl_config.h"
#include "uart_tune.h"
#include <stdbool.h>
#include <stdint.h>

static volatile uint32_t g_ms_ticks;
static volatile uint32_t g_run_time_ms;
static volatile bool g_run_timer_enabled;
static bool g_buttons_released_after_boot;

typedef enum {
    APP_MODE_LINE_TIMER = 0,
    APP_MODE_Q4_AB,
    APP_MODE_Q6_ARBITRARY,
    APP_MODE_Q7_ARBITRARY,
    APP_MODE_Q3_SEQUENCE,
    APP_MODE_COUNT,
    /*
     * 保留步进机构调试代码，但不放入 B5 的比赛模式循环。
     * B5 现场操作严格只在模式一和模式二之间切换。
     */
    APP_MODE_STEPPER_MANUAL
} AppMode;

typedef enum {
    LINE_LAP_IDLE = 0,
    LINE_LAP_WAIT_START_A,
    LINE_LAP_RUNNING,
    LINE_LAP_FINISHED
} LineLapPhase;

typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    int base_pwm;
    int left_pwm_trim;
    int right_pwm_trim;
    uint16_t brake_ms;
    float line_kp;
    float line_kd;
    int16_t line_corr_limit;
    int16_t line_search_pwm;
    uint8_t a_total_min;
    uint8_t a_span_min;
    uint8_t a_confirm_frames;
    uint16_t a_min_lap_ms;
    uint32_t a_min_lap_counts;
} LineModeParams;

typedef struct {
    int32_t jog_steps;
    float jog_speed_dps;
    uint32_t jog_interval_ms;
} StepperModeParams;

typedef struct {
    uint8_t total_count;
    uint8_t center_count;
    uint8_t span;
    bool candidate;
    bool finish_candidate;
} AMarkStats;

static const LineModeParams g_line_mode_params = {
    DEFAULT_KP,
    DEFAULT_KI,
    DEFAULT_KD,
    LINE_DEFAULT_TARGET,
    LINE_DEFAULT_BASE_PWM,
    DEFAULT_LEFT_PWM_TRIM,
    DEFAULT_RIGHT_PWM_TRIM,
    DEFAULT_BRAKE_MS,
    LINE_DEFAULT_KP,
    LINE_DEFAULT_KD,
    LINE_CORR_LIMIT,
    LINE_SEARCH_PWM,
    A_MARK_DEFAULT_TOTAL,
    A_MARK_DEFAULT_SPAN,
    A_MARK_DEFAULT_FRAMES,
    A_MARK_MIN_LAP_MS,
    A_MARK_MIN_LAP_COUNTS
};

/*
 * 第四问使用独立参数副本。即使后续对 Q4 的速度、基础 PWM 或循迹
 * 参数进行修改，也不会改变模式一的一圈计时参数。
 */
static const LineModeParams g_q4_mode_params = {
    0.0f, /* 兼容参数结构的占位值；模式二不调用速度 PID。 */
    0.0f,
    0.0f,
    Q4_CRUISE_TARGET_COUNTS,
    Q4_CRUISE_BASE_PWM,
    Q4_LEFT_PWM_TRIM,
    Q4_RIGHT_PWM_TRIM,
    Q4_BRAKE_MS,
    Q4_LINE_KP,
    Q4_LINE_KD,
    Q4_LINE_CORR_LIMIT,
    Q4_LINE_SEARCH_PWM,
    Q4_A_MARK_TOTAL,
    Q4_A_MARK_SPAN,
    Q4_A_MARK_FRAMES,
    Q4_A_MIN_LAP_MS,
    Q4_A_MIN_LAP_COUNTS
};

static const LineModeParams g_q6_mode_params = {
    0.0f,
    0.0f,
    0.0f,
    Q6_CRUISE_TARGET_COUNTS,
    Q6_CRUISE_BASE_PWM,
    Q6_LEFT_PWM_TRIM,
    Q6_RIGHT_PWM_TRIM,
    Q6_BRAKE_MS,
    Q6_LINE_KP,
    Q6_LINE_KD,
    Q6_LINE_CORR_LIMIT,
    Q6_LINE_SEARCH_PWM,
    Q6_A_MARK_TOTAL,
    Q6_A_MARK_SPAN,
    Q6_A_MARK_FRAMES,
    Q6_A_MIN_LAP_MS,
    Q6_A_MIN_LAP_COUNTS
};

static const LineModeParams g_q7_mode_params = {
    0.0f,
    0.0f,
    0.0f,
    Q7_CRUISE_TARGET_COUNTS,
    Q7_CRUISE_BASE_PWM,
    Q7_LEFT_PWM_TRIM,
    Q7_RIGHT_PWM_TRIM,
    Q7_BRAKE_MS,
    Q7_LINE_KP,
    Q7_LINE_KD,
    Q7_LINE_CORR_LIMIT,
    Q7_LINE_SEARCH_PWM,
    Q7_A_MARK_TOTAL,
    Q7_A_MARK_SPAN,
    Q7_A_MARK_FRAMES,
    Q7_A_MIN_LAP_MS,
    Q7_A_MIN_LAP_COUNTS
};

static const StepperModeParams g_stepper_mode_params = {
    STEPPER_JOG_STEPS,
    STEPPER_DEFAULT_DPS,
    STEPPER_JOG_INTERVAL_MS
};

static void debug_putc(char ch)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)ch);
}

static void debug_puts(const char *text)
{
    while (*text != '\0') {
        debug_putc(*text++);
    }
}

static void debug_print_i32(int32_t value)
{
    char digits[11];
    uint8_t index = 0U;
    uint32_t mag;

    if (value < 0) {
        debug_putc('-');
        mag = (uint32_t)(-value);
    } else {
        mag = (uint32_t)value;
    }
    do {
        digits[index++] = (char)('0' + (mag % 10U));
        mag /= 10U;
    } while ((mag != 0U) && (index < (uint8_t)sizeof(digits)));

    while (index > 0U) {
        debug_putc(digits[--index]);
    }
}

static void debug_print_u8_hex(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    debug_putc(hex[(value >> 4) & 0x0F]);
    debug_putc(hex[value & 0x0F]);
}

static void debug_print_stepper_state(const char *tag)
{
    StepperArmState state = stepper_arm_get_state();

    debug_puts(tag);
    debug_puts(" CUR=");
    debug_print_i32(state.current_steps);
    debug_puts(" HIGH=");
    debug_print_i32(state.min_steps);
    debug_puts(" NEU=");
    debug_print_i32(state.neutral_steps);
    debug_puts(" LOW=");
    debug_print_i32(state.max_steps);
    debug_puts(" BUSY=");
    debug_print_i32(state.busy ? 1 : 0);
    debug_puts(" COMM=");
    debug_print_i32(state.communication_ok ? 1 : 0);
    debug_puts(" FW=");
    debug_print_i32((int32_t)state.firmware_version);
    debug_puts(" HW=0x");
    debug_print_u8_hex(state.hardware_type);
    debug_putc('/');
    debug_print_i32((int32_t)state.hardware_version);
    debug_puts(" ST=0x");
    debug_print_u8_hex(state.status_flags);
    debug_puts("\r\n");
}

static uint32_t run_time_snapshot_ms(void)
{
    uint32_t value;

    __disable_irq();
    value = g_run_time_ms;
    __enable_irq();
    return value;
}

static int16_t ramp_to(int16_t current, int16_t target, int16_t step)
{
    if (current < target) {
        current = (int16_t)(current + step);
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current = (int16_t)(current - step);
        if (current < target) {
            current = target;
        }
    }
    return current;
}

static int16_t float_to_i16_saturated(float value)
{
    if (value > 32767.0f) {
        return 32767;
    }
    if (value < -32768.0f) {
        return -32768;
    }
    return (int16_t)value;
}

static int16_t clamp_pwm(int32_t value)
{
    int32_t limit = (int32_t)(MOTOR_PWM_PERIOD - MOTOR_PWM_DEAD_TICKS);

    if (value > limit) {
        return (int16_t)limit;
    }
    if (value < -limit) {
        return (int16_t)(-limit);
    }
    return (int16_t)value;
}

static float clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/*
 * 仅供模式三使用：循迹误差刚进入弯道区间便快速建立纵向减速补偿，
 * 出弯时降低滤波权重，避免水管目标角突然跳回零位。
 */
static float q6_curve_feedforward_update(float current_deg,
                                         int16_t line_error,
                                         bool line_valid,
                                         bool q7_mode,
                                         uint32_t now_ms,
                                         bool *curve_active,
                                         uint32_t *curve_candidate_ms,
                                         uint32_t *curve_enter_ms,
                                         uint32_t *straight_since_ms)
{
    float target_deg = 0.0f;
    int32_t error_abs = (line_error < 0) ?
        -(int32_t)line_error : (int32_t)line_error;
    bool enabled = q7_mode ? (Q7_CURVE_FF_ENABLE != 0) :
        (Q6_CURVE_FF_ENABLE != 0);
    int32_t entry_err = q7_mode ? Q7_CURVE_FF_ENTRY_ERR :
        Q6_CURVE_FF_ENTRY_ERR;
    uint32_t entry_confirm_ms = q7_mode ? Q7_CURVE_FF_ENTRY_CONFIRM_MS :
        Q6_CURVE_FF_ENTRY_CONFIRM_MS;
    uint32_t min_hold_ms = q7_mode ? Q7_CURVE_FF_MIN_HOLD_MS :
        Q6_CURVE_FF_MIN_HOLD_MS;
    int32_t exit_err = q7_mode ? Q7_CURVE_FF_EXIT_ERR :
        Q6_CURVE_FF_EXIT_ERR;
    uint32_t exit_confirm_ms = q7_mode ? Q7_CURVE_FF_EXIT_CONFIRM_MS :
        Q6_CURVE_FF_EXIT_CONFIRM_MS;
    int32_t full_err = q7_mode ? Q7_CURVE_FF_FULL_ERR :
        Q6_CURVE_FF_FULL_ERR;
    float max_deg = q7_mode ? Q7_CURVE_FF_MAX_DEG : Q6_CURVE_FF_MAX_DEG;
    float hold_deg = q7_mode ? Q7_CURVE_FF_HOLD_DEG : Q6_CURVE_FF_HOLD_DEG;
    float sign = q7_mode ? Q7_CURVE_FF_SIGN : Q6_CURVE_FF_SIGN;
    float attack_new = q7_mode ? Q7_CURVE_FF_ATTACK_NEW :
        Q6_CURVE_FF_ATTACK_NEW;
    float release_new = q7_mode ? Q7_CURVE_FF_RELEASE_NEW :
        Q6_CURVE_FF_RELEASE_NEW;

    if (enabled && !*curve_active) {
        if (line_valid && (error_abs >= entry_err)) {
            if (*curve_candidate_ms == 0U) {
                *curve_candidate_ms = now_ms;
            } else if ((now_ms - *curve_candidate_ms) >=
                       entry_confirm_ms) {
                *curve_active = true;
                *curve_enter_ms = now_ms;
                *curve_candidate_ms = 0U;
                *straight_since_ms = 0U;
            }
        } else {
            *curve_candidate_ms = 0U;
        }
    } else if (enabled && line_valid &&
               (error_abs >= entry_err)) {
        *straight_since_ms = 0U;
    } else if (*curve_active && line_valid &&
               ((now_ms - *curve_enter_ms) >=
                min_hold_ms) &&
               (error_abs <= exit_err)) {
        if (*straight_since_ms == 0U) {
            *straight_since_ms = now_ms;
        } else if ((now_ms - *straight_since_ms) >=
                   exit_confirm_ms) {
            *curve_active = false;
            *straight_since_ms = 0U;
        }
    } else if (*curve_active && line_valid) {
        *straight_since_ms = 0U;
    }

    if (enabled && *curve_active) {
        float ratio = (float)(error_abs - entry_err) /
            (float)(full_err - entry_err);
        float demand_deg;

        ratio = clamp_f32(ratio, 0.0f, 1.0f);
        demand_deg = max_deg * ratio;
        if (demand_deg < hold_deg) {
            demand_deg = hold_deg;
        }
        target_deg = sign * demand_deg;
    }

    {
        float current_abs = (current_deg < 0.0f) ?
            -current_deg : current_deg;
        float target_abs = (target_deg < 0.0f) ?
            -target_deg : target_deg;
        float filter_new = (target_abs > current_abs) ?
            attack_new : release_new;

        current_deg += filter_new * (target_deg - current_deg);
    }
    if ((current_deg > -0.005f) && (current_deg < 0.005f)) {
        current_deg = 0.0f;
    }
    return current_deg;
}

static uint8_t count_bits_u8(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count = (uint8_t)(count + (value & 1U));
        value >>= 1U;
    }
    return count;
}

static bool b5_pressed(void)
{
    return (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_5) == 0U);
}

static bool b15_pressed(void)
{
    return (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_15) == 0U);
}

static bool b11_pressed(void)
{
    return (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_11) == 0U);
}

static bool b14_pressed(void)
{
    return (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_14) == 0U);
}

static const char *app_mode_name(AppMode mode)
{
    switch (mode) {
    case APP_MODE_LINE_TIMER:
        return "M1 LINE";
    case APP_MODE_Q4_AB:
        return "M2 Q4";
    case APP_MODE_Q6_ARBITRARY:
        return "M3 Q6 CENTER";
    case APP_MODE_Q7_ARBITRARY:
        return "M4 Q7 ANY";
    case APP_MODE_Q3_SEQUENCE:
        return "M5 Q3 SEQ";
    case APP_MODE_STEPPER_MANUAL:
        return "M5 STEPPER";
    default:
        return "UNKNOWN";
    }
}

static void apply_drive_mode_params(TuneParams *tune,
                                    const LineModeParams *params)
{
    tune->kp = params->kp;
    tune->ki = params->ki;
    tune->kd = params->kd;
    tune->target = params->target;
    tune->base_pwm = params->base_pwm;
    tune->left_pwm_trim = params->left_pwm_trim;
    tune->right_pwm_trim = params->right_pwm_trim;
    tune->brake_ms = params->brake_ms;
    tune->line_kp = params->line_kp;
    tune->line_kd = params->line_kd;
    tune->line_corr_limit = params->line_corr_limit;
    tune->line_search_pwm = params->line_search_pwm;
    tune->a_total_min = params->a_total_min;
    tune->a_span_min = params->a_span_min;
    tune->a_confirm_frames = params->a_confirm_frames;
    tune->a_min_lap_ms = params->a_min_lap_ms;
    tune->a_min_lap_counts = params->a_min_lap_counts;
    tune->a_debug_enable = false;
    tune->reset_pid = true;
}

static void apply_line_mode_params(TuneParams *tune)
{
    apply_drive_mode_params(tune, &g_line_mode_params);
}

static void apply_q4_mode_params(TuneParams *tune)
{
    apply_drive_mode_params(tune, &g_q4_mode_params);
}

static void apply_q6_mode_params(TuneParams *tune)
{
    apply_drive_mode_params(tune, &g_q6_mode_params);
}

static void apply_q7_mode_params(TuneParams *tune)
{
    apply_drive_mode_params(tune, &g_q7_mode_params);
}

static bool app_mode_is_q6_family(AppMode mode)
{
    return (mode == APP_MODE_Q6_ARBITRARY) ||
        (mode == APP_MODE_Q7_ARBITRARY);
}

static bool app_mode_is_ball_drive(AppMode mode)
{
    return (mode == APP_MODE_Q4_AB) ||
        app_mode_is_q6_family(mode);
}

static void stepper_jog_buttons_service(const StepperModeParams *params)
{
    static uint32_t last_jog_ms;

    if ((g_ms_ticks - last_jog_ms) < params->jog_interval_ms) {
        return;
    }
    if (b11_pressed() && !b14_pressed()) {
        last_jog_ms = g_ms_ticks;
        stepper_arm_jog_steps(params->jog_steps, params->jog_speed_dps);
    } else if (b14_pressed() && !b11_pressed()) {
        last_jog_ms = g_ms_ticks;
        stepper_arm_jog_steps(-params->jog_steps, params->jog_speed_dps);
    }
}

static bool button_latched_event(bool pressed, bool *wait_release,
                                 uint32_t *rearm_until_ms)
{
    if ((int32_t)(g_ms_ticks - *rearm_until_ms) < 0) {
        return false;
    }

    if (*wait_release) {
        if (!pressed) {
            *wait_release = false;
            *rearm_until_ms = g_ms_ticks + BUTTON_REARM_MS;
        }
        return false;
    }

    if (!pressed) {
        return false;
    }

    *wait_release = true;
    return true;
}

static bool b5_press_event(void)
{
    static bool wait_release;
    static uint32_t rearm_until_ms;

    return button_latched_event(b5_pressed(), &wait_release, &rearm_until_ms);
}

static bool b15_press_event(void)
{
    static bool wait_release;
    static uint32_t rearm_until_ms;

    return button_latched_event(b15_pressed(), &wait_release, &rearm_until_ms);
}

static bool b11_press_event(void)
{
    static bool wait_release;
    static uint32_t rearm_until_ms;

    return button_latched_event(b11_pressed(), &wait_release, &rearm_until_ms);
}

static bool b14_press_event(void)
{
    static bool wait_release;
    static uint32_t rearm_until_ms;

    return button_latched_event(b14_pressed(), &wait_release, &rearm_until_ms);
}

static bool buttons_released(void)
{
    return !b5_pressed() && !b15_pressed();
}

static void stop_line_outputs(TuneParams *tune)
{
    tune->run = false;
    tune->line_active = false;
    tune->brake_request = true;
    g_run_timer_enabled = false;
}

static void start_line_tracking(TuneParams *tune)
{
    tune->run = true;
    tune->line_active = true;
    tune->reset_pid = true;
    g_run_time_ms = 0U;
    g_run_timer_enabled = true;
    debug_puts("RUN TIME START\r\n");
}

static void resume_line_tracking(TuneParams *tune)
{
    tune->run = true;
    tune->line_active = true;
    tune->reset_pid = true;
    g_run_timer_enabled = true;
    debug_puts("RUN TIME RESUME\r\n");
}

static void pause_line_tracking(TuneParams *tune)
{
    tune->run = false;
    tune->line_active = false;
    tune->brake_request = true;
    g_run_timer_enabled = false;
    debug_puts("RUN TIME PAUSE ");
    debug_print_i32((int32_t)run_time_snapshot_ms());
    debug_puts("\r\n");
}

static void clear_line_timer(void)
{
    __disable_irq();
    g_run_time_ms = 0U;
    __enable_irq();
    debug_puts("RUN TIME CLEAR\r\n");
}

static void line_lap_reset(LineLapPhase *phase, uint8_t *confirm_frames,
                           uint8_t *release_frames, bool *released_start)
{
    *phase = LINE_LAP_IDLE;
    *confirm_frames = 0U;
    *release_frames = 0U;
    *released_start = false;
}

static void debug_print_keys(const char *tag, const TuneParams *tune)
{
    debug_puts(tag);
    debug_puts(" K1=");
    debug_print_i32(b5_pressed() ? 1 : 0);
    debug_puts(" K4=");
    debug_print_i32(b15_pressed() ? 1 : 0);
    debug_puts(" RUN=");
    debug_print_i32(tune->run ? 1 : 0);
    debug_puts("\r\n");
}

static void debug_print_mode(const char *tag, AppMode mode)
{
    debug_puts(tag);
    debug_puts(" MODE=");
    debug_puts(app_mode_name(mode));
    debug_puts("\r\n");
}

static AMarkStats line_a_mark_update(uint8_t gray_bits,
                                     const TuneParams *tune)
{
    AMarkStats stats;
    int8_t first = -1;
    int8_t last = -1;
    uint8_t i;

    stats.total_count = count_bits_u8(gray_bits);
    stats.center_count = 0U;
    for (i = 0U; i < 8U; i++) {
        if ((gray_bits & (uint8_t)(1U << i)) != 0U) {
            if (first < 0) {
                first = (int8_t)i;
            }
            last = (int8_t)i;
        }
    }
    stats.span = (first < 0) ? 0U : (uint8_t)(last - first + 1);
    stats.candidate = (stats.total_count >= tune->a_total_min) &&
        (stats.span >= tune->a_span_min);
    stats.finish_candidate =
        (stats.total_count >= A_MARK_FINISH_TOTAL) &&
        (stats.span >= A_MARK_FINISH_SPAN);
    return stats;
}

static void debug_print_a_status(LineLapPhase phase, uint32_t run_time_ms,
                                 uint8_t gray_bits,
                                 const AMarkStats *stats,
                                 uint8_t confirm_frames,
                                 const TuneParams *tune,
                                 uint32_t lap_counts,
                                 uint8_t baseline_total,
                                 uint8_t baseline_span)
{
    debug_puts("ADBG,T=");
    debug_print_i32((int32_t)run_time_ms);
    debug_puts(",PH=");
    debug_print_i32((int32_t)phase);
    debug_puts(",BITS=0x");
    debug_print_u8_hex(gray_bits);
    debug_puts(",TOT=");
    debug_print_i32((int32_t)stats->total_count);
    debug_puts(",SPAN=");
    debug_print_i32((int32_t)stats->span);
    debug_puts(",AF=");
    debug_print_i32((int32_t)confirm_frames);
    debug_puts(",ODOM=");
    debug_print_i32((int32_t)lap_counts);
    debug_puts(",BASE=");
    debug_print_i32((int32_t)baseline_total);
    debug_putc('/');
    debug_print_i32((int32_t)baseline_span);
    debug_puts(",AT=");
    debug_print_i32((int32_t)tune->a_total_min);
    debug_puts(",AC=");
    debug_print_i32((int32_t)tune->a_span_min);
    debug_puts(",ERR=");
    debug_print_i32((int32_t)tune->line_error);
    debug_puts(",COR=");
    debug_print_i32((int32_t)tune->line_correction);
    debug_puts("\r\n");
}

static void debug_print_a_gate(const char *tag, uint32_t run_time_ms,
                               uint8_t gray_bits,
                               const AMarkStats *stats,
                               uint32_t lap_counts,
                               uint8_t baseline_total,
                               uint8_t baseline_span,
                               bool enough_time,
                               bool enough_odom,
                               bool width_jump)
{
    debug_puts(tag);
    debug_puts(",T=");
    debug_print_i32((int32_t)run_time_ms);
    debug_puts(",BITS=0x");
    debug_print_u8_hex(gray_bits);
    debug_puts(",TOT=");
    debug_print_i32((int32_t)stats->total_count);
    debug_puts(",SPAN=");
    debug_print_i32((int32_t)stats->span);
    debug_puts(",ODOM=");
    debug_print_i32((int32_t)lap_counts);
    debug_puts(",BASE=");
    debug_print_i32((int32_t)baseline_total);
    debug_putc('/');
    debug_print_i32((int32_t)baseline_span);
    debug_puts(",TIME=");
    debug_print_i32(enough_time ? 1 : 0);
    debug_puts(",DIST=");
    debug_print_i32(enough_odom ? 1 : 0);
    debug_puts(",JUMP=");
    debug_print_i32(width_jump ? 1 : 0);
    debug_puts("\r\n");
}

void RUN_TIMER_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(RUN_TIMER_INST)) {
        case DL_TIMER_IIDX_ZERO:
            g_ms_ticks++;
            if (g_run_timer_enabled) {
                g_run_time_ms++;
            }
            break;
        default:
            break;
    }
}

void GROUP1_IRQHandler(void)
{
    encoder_handle_gpio_irq();
}

void UART_0_INST_IRQHandler(void)
{
    uart_tune_handle_irq();
}

void UART_2_INST_IRQHandler(void)
{
    k230_ball_handle_irq(g_ms_ticks);
}

void UART_X42S_INST_IRQHandler(void)
{
    stepper_arm_handle_uart_irq(g_ms_ticks);
}

int main(void)
{
    TuneParams tune;
    LineTracker tracker;
    PidController left_pid;
    PidController right_pid;
    uint32_t last_control_ms = 0;
#if OLED_RUNTIME_ENABLE
    uint32_t last_oled_ms = 0;
#endif
#if CONTROL_LAG_DEBUG_ENABLE
    uint32_t last_lag_report_ms = 0;
#endif
    uint32_t last_boot_key_report_ms = 0;
    uint32_t last_k230_status_ms = 0;
    uint32_t brake_until_ms = 0;
    AppMode app_mode = APP_MODE_LINE_TIMER;
    bool mode_started = false;
    bool line_paused = false;
    LineLapPhase line_lap_phase = LINE_LAP_IDLE;
    uint8_t a_confirm_frames = 0U;
    uint8_t a_release_frames = 0U;
    bool a_released_start = false;
    uint32_t line_lap_counts = 0U;
    uint8_t a_baseline_total = 1U;
    uint8_t a_baseline_span = 1U;
    uint32_t last_a_debug_ms = 0U;
    uint32_t last_a_gate_debug_ms = 0U;
    MotionState motion;
    K230BallSample ball_sample;
    bool ball_sample_ok = false;
    K230ControlState k230_control = {0};
    bool k230_control_ok = false;
    K230Q3TuneParams k230_q3_tune = {0};
    uint32_t q3_tune_applied_revision = 0U;
    K230StartAngleParams k230_start_angles = {0};
    uint32_t start_angles_applied_revision = 0U;
    Q4MotionSetpoint q4_setpoint;
    bool q4_b_time_reported = false;
    bool q4_preload_pending = false;
    bool q4_resume_after_preload = false;
    uint32_t q4_preload_start_ms = 0U;
    float q6_curve_feedforward_deg = 0.0f;
    bool q6_curve_active = false;
    uint32_t q6_curve_candidate_ms = 0U;
    uint32_t q6_curve_enter_ms = 0U;
    uint32_t q6_straight_since_ms = 0U;
    uint8_t q3_last_sequence_phase = 0xFFU;
    bool x42s_version_reported = false;
    int left_speed = 0;
    int right_speed = 0;
    int16_t left_pwm = 0;
    int16_t right_pwm = 0;
    uint8_t gray_raw = 0U;
    int oled_status;
    bool oled_ready = false;

    SYSCFG_DL_init();
    motor_init();
    stepper_arm_init();
    k230_ball_init();
    ball_balance_init();
    q4_motion_init();
    NVIC_EnableIRQ(RUN_TIMER_INST_INT_IRQN);
    __enable_irq();
    while (g_ms_ticks < 200U) {
        __NOP();
    }

    encoder_init();
    motion_state_init(&motion);
    line_tracker_init(&tracker);
    uart_tune_init(&tune);
    apply_line_mode_params(&tune);
    uart_tune_clear_rx();
#if OLED_RUNTIME_ENABLE
    oled_status = oled_init();
    if (oled_status >= 0) {
        oled_ready = true;
        oled_clear();
        oled_puts(0U, 0U, "OLED:OK");
    } else {
        oled_ready = false;
        debug_puts("OLED INIT FAIL\r\n");
    }
#else
    (void)oled_status;
    (void)oled_ready;
#endif
    tune.line_active = false;

    pid_init(&left_pid, tune.kp, tune.ki, tune.kd, tune.target,
        DEFAULT_MAX_INTEGRAL, DEFAULT_MAX_OUTPUT);
    pid_init(&right_pid, tune.kp, tune.ki, tune.kd, tune.target,
        DEFAULT_MAX_INTEGRAL, DEFAULT_MAX_OUTPUT);
    last_control_ms = g_ms_ticks;
#if OLED_RUNTIME_ENABLE
    last_oled_ms = g_ms_ticks;
#endif

    while (1) {
        if (g_ms_ticks < STARTUP_SAFE_MS) {
            tune.run = false;
            tune.line_active = false;
            tune.brake_request = false;
            left_pwm = 0;
            right_pwm = 0;
            motor_stop();
            uart_tune_clear_rx();
            continue;
        }

        if (!g_buttons_released_after_boot) {
            if (buttons_released()) {
                g_buttons_released_after_boot = true;
                debug_print_keys("BOOT READY", &tune);
            } else {
                tune.run = false;
                tune.line_active = false;
                motor_stop();
                if ((g_ms_ticks - last_boot_key_report_ms) >= 500U) {
                    last_boot_key_report_ms = g_ms_ticks;
                    debug_print_keys("BOOT WAIT KEYS", &tune);
                }
                continue;
            }
        }

        {
            bool was_run = tune.run;
            bool changed = uart_tune_poll(&tune);

            /*
             * 只有“本轮确实收到串口命令，并由停止变成运行”时才拦截非模式一
             * 的远程启动。此前这里只判断 tune.run，导致 B15 启动模式二后，
             * 下一轮主循环马上又把 run 清零并刹车，实车表现为猛动一下就停。
             */
            if ((app_mode != APP_MODE_LINE_TIMER) && changed &&
                !was_run && tune.run) {
                tune.run = false;
                tune.line_active = false;
                tune.brake_request = true;
                debug_puts("UART RUN IGNORED IN NON LINE MODE\r\n");
            } else if (changed && !was_run && tune.run) {
                mode_started = true;
                line_lap_phase = LINE_LAP_RUNNING;
                a_confirm_frames = 0U;
                a_release_frames = 0U;
                a_released_start = false;
                line_lap_counts = 0U;
                a_baseline_total = 1U;
                a_baseline_span = 1U;
                g_run_time_ms = 0U;
                g_run_timer_enabled = true;
                debug_puts("RUN TIME START\r\n");
            } else if (changed && was_run && !tune.run) {
                mode_started = false;
                line_paused = false;
                line_lap_reset(&line_lap_phase, &a_confirm_frames,
                    &a_release_frames, &a_released_start);
                line_lap_counts = 0U;
                a_baseline_total = 1U;
                a_baseline_span = 1U;
                g_run_timer_enabled = false;
                debug_puts("RUN TIME MS ");
                debug_print_i32((int32_t)run_time_snapshot_ms());
                debug_puts("\r\n");
            }

            if (changed) {
                if (tune.run && (app_mode == APP_MODE_LINE_TIMER)) {
                    tune.line_active = true;
                }
                if (app_mode == APP_MODE_LINE_TIMER) {
                    pid_set_gains(&left_pid, tune.kp, tune.ki, tune.kd);
                    pid_set_gains(&right_pid, tune.kp, tune.ki, tune.kd);
                }
            }
        }

        if (tune.ball_zero_request) {
            stepper_arm_mark_neutral();
            tune.ball_zero_request = false;
            debug_print_stepper_state("STEPPER NEUTRAL MARKED");
        }

        if (tune.stepper_mark_high_request) {
            stepper_arm_mark_high_limit();
            tune.stepper_mark_high_request = false;
            debug_print_stepper_state("STEPPER HIGH MARKED");
        }

        if (tune.stepper_mark_low_request) {
            stepper_arm_mark_low_limit();
            tune.stepper_mark_low_request = false;
            debug_print_stepper_state("STEPPER LOW MARKED");
        }

        if (tune.stepper_print_request) {
            tune.stepper_print_request = false;
            debug_print_stepper_state("STEPPER STATE");
        }

        if (tune.ball_stop_request) {
            ball_balance_stop();
            tune.ball_stop_request = false;
            debug_puts("BALL STOP\r\n");
        }

        if (tune.ball_start_request) {
            tune.run = false;
            tune.line_active = false;
            tune.brake_request = true;
            ball_balance_start_sequence();
            tune.ball_start_request = false;
            debug_puts("BALL Q3 START\r\n");
        }

        if (tune.ball_target_request) {
            tune.run = false;
            tune.line_active = false;
            tune.brake_request = true;
            ball_balance_hold_target(tune.ball_target_mm);
            tune.ball_target_request = false;
            debug_puts("BALL TARGET MM ");
            debug_print_i32((int32_t)tune.ball_target_mm);
            debug_puts("\r\n");
        }

        if (tune.reset_pid) {
            pid_reset(&left_pid);
            pid_reset(&right_pid);
            encoder_clear();
            line_tracker_reset(&tracker);
            tune.line_bits = 0U;
            tune.line_error = 0;
            tune.line_correction = 0;
            tune.line_valid = false;
            tune.reset_pid = false;
        }

        k230_control_ok = k230_ball_get_control_state(
            &k230_control, g_ms_ticks);

        /*
         * State 6 sends the six offline-tuning values independently of the
         * vision stream.  Accept them only while Tianmeng is in M5 and idle;
         * ball_balance_start_q3() snapshots them for the complete run.
         */
        if ((app_mode == APP_MODE_Q3_SEQUENCE) && !mode_started &&
            k230_ball_get_q3_tune(&k230_q3_tune) &&
            (k230_q3_tune.revision != q3_tune_applied_revision)) {
            Q3OpenLoopParams q3_params;
            uint8_t stage;

            for (stage = 0U; stage < 3U; stage++) {
                q3_params.angle_deg[stage] =
                    (float)k230_q3_tune.angle_cdeg[stage] * 0.01f;
                q3_params.time_ms[stage] = k230_q3_tune.time_ms[stage];
            }
            if (ball_balance_set_q3_open_loop_params(&q3_params)) {
                debug_puts("MODE Q3 LCD PARAM APPLIED\r\n");
            } else {
                debug_puts("MODE Q3 LCD PARAM REJECTED\r\n");
            }
            q3_tune_applied_revision = k230_q3_tune.revision;
        }

        /* State 5/7 initial compensation angles are private to modes 2-4. */
        if (app_mode_is_ball_drive(app_mode) && !mode_started &&
            !tune.run && !q4_preload_pending &&
            k230_ball_get_start_angles(&k230_start_angles) &&
            (k230_start_angles.revision != start_angles_applied_revision)) {
            bool angle_accepted = q4_motion_set_start_angles(
                (float)k230_start_angles.angle_cdeg[0] * 0.01f,
                (float)k230_start_angles.angle_cdeg[1] * 0.01f,
                (float)k230_start_angles.angle_cdeg[2] * 0.01f);
            bool bias_accepted = ball_balance_set_q67_target_biases(
                (float)k230_start_angles.target_bias_mm[0],
                (float)k230_start_angles.target_bias_mm[1],
                (float)k230_start_angles.target_bias_mm[2],
                (float)k230_start_angles.target_bias_mm[3]);
            bool accepted = angle_accepted && bias_accepted;

            debug_puts(accepted ? "MODE 2/3/4 LCD ANGLES APPLIED\r\n" :
                                  "MODE 2/3/4 LCD ANGLES REJECTED\r\n");
            start_angles_applied_revision = k230_start_angles.revision;
        }

        if (b5_press_event()) {
            debug_print_keys("KEY K1 NEXT", &tune);
            stop_line_outputs(&tune);
            q4_motion_stop();
            ball_balance_stop();
            stepper_arm_stop();
            q4_preload_pending = false;
            q6_curve_feedforward_deg = 0.0f;
            q6_curve_active = false;
            q6_curve_candidate_ms = 0U;
            q6_curve_enter_ms = 0U;
            q6_straight_since_ms = 0U;
            q3_last_sequence_phase = 0xFFU;
            mode_started = false;
            line_paused = false;
            line_lap_reset(&line_lap_phase, &a_confirm_frames,
                &a_release_frames, &a_released_start);
            line_lap_counts = 0U;
            a_baseline_total = 1U;
            a_baseline_span = 1U;
            app_mode = (AppMode)(((uint8_t)app_mode + 1U) %
                (uint8_t)APP_MODE_COUNT);
            if (app_mode == APP_MODE_LINE_TIMER) {
                apply_line_mode_params(&tune);
                pid_set_gains(&left_pid, tune.kp, tune.ki, tune.kd);
                pid_set_gains(&right_pid, tune.kp, tune.ki, tune.kd);
            } else if (app_mode == APP_MODE_Q4_AB) {
                apply_q4_mode_params(&tune);
            } else if (app_mode == APP_MODE_Q6_ARBITRARY) {
                apply_q6_mode_params(&tune);
            } else if (app_mode == APP_MODE_Q7_ARBITRARY) {
                apply_q7_mode_params(&tune);
            }
            debug_print_mode("MODE SWITCH", app_mode);
        }

        if (b15_press_event()) {
            debug_print_keys("KEY K4 START", &tune);
            if ((app_mode == APP_MODE_Q3_SEQUENCE) && !mode_started &&
                (!k230_control_ok || (k230_control.mode != 6U) ||
                 (k230_control.run_state != 1U))) {
                debug_puts("MODE Q3 BLOCK K230 STATE6 NOT RUN\r\n");
            } else if (app_mode_is_q6_family(app_mode) &&
                !q4_preload_pending && !tune.run &&
                (!k230_control_ok || (k230_control.mode != 7U) ||
                 (k230_control.run_state != 1U))) {
                debug_puts("MODE Q6/Q7 BLOCK K230 STATE7 NOT RUN\r\n");
            } else if (app_mode == APP_MODE_Q3_SEQUENCE) {
                tune.run = false;
                tune.line_active = false;
                motor_stop();
                if (mode_started) {
                    ball_balance_stop();
                    mode_started = false;
                    line_paused = true;
                    g_run_timer_enabled = false;
                    debug_puts("MODE Q3 PAUSE\r\n");
                } else {
                    if (ball_balance_start_q3(g_ms_ticks)) {
                        mode_started = true;
                        line_paused = false;
                        q3_last_sequence_phase =
                            k230_control.sequence_phase;
                        __disable_irq();
                        g_run_time_ms = 0U;
                        __enable_irq();
                        g_run_timer_enabled = true;
                        debug_puts("MODE Q3 OPEN LOOP START STATE6\r\n");
                    } else {
                        mode_started = false;
                        g_run_timer_enabled = false;
                        debug_puts("MODE Q3 BLOCK X42S NOT READY\r\n");
                    }
                }
            } else if (app_mode == APP_MODE_LINE_TIMER) {
                if (tune.run) {
                    line_paused = true;
                    debug_puts("MODE LINE PAUSE\r\n");
                    pause_line_tracking(&tune);
                } else {
                    mode_started = true;
                    if (line_paused) {
                        line_paused = false;
                        debug_puts("MODE LINE RESUME\r\n");
                        resume_line_tracking(&tune);
                    } else {
                        debug_puts("MODE LINE START\r\n");
                        line_lap_phase = LINE_LAP_RUNNING;
                        a_confirm_frames = 0U;
                        a_release_frames = 0U;
                        a_released_start = false;
                        line_lap_counts = 0U;
                        a_baseline_total = 1U;
                        a_baseline_span = 1U;
                        start_line_tracking(&tune);
                    }
                }
            } else if (app_mode_is_ball_drive(app_mode)) {
                if (q4_preload_pending) {
                    /* 预置等待期间再次按 B15，取消本次启动并回到零角。 */
                    q4_preload_pending = false;
                    q4_motion_stop();
                    ball_balance_stop();
                    mode_started = false;
                    line_paused = false;
                    debug_puts("MODE Q4 PRELOAD CANCEL\r\n");
                } else if (tune.run) {
                    line_paused = true;
                    q4_motion_stop();
                    ball_balance_set_feedforward(0.0f);
                    if (app_mode_is_q6_family(app_mode)) {
                        q6_curve_feedforward_deg = 0.0f;
                        q6_curve_active = false;
                        q6_curve_candidate_ms = 0U;
                        q6_curve_enter_ms = 0U;
                        q6_straight_since_ms = 0U;
                        ball_balance_set_curve_feedforward(0.0f);
                        ball_balance_set_feedback_scale(1.0f);
                    }
                    debug_puts("MODE Q4 PAUSE\r\n");
                    pause_line_tracking(&tune);
                } else {
                    q4_b_time_reported = false;
                    /*
                     * 每次模式二启动/继续都从零建立实际加速度估计，避免暂停
                     * 前的编码器速度残留被误认为新的启动冲击。
                     */
                    /* 预置期间底盘必须保持停止，运动计时到真正起步时再清零。 */
                    stop_line_outputs(&tune);
                    q4_motion_select_profile(
                        (app_mode == APP_MODE_Q7_ARBITRARY) ?
                        Q4_MOTION_PROFILE_Q7_ARBITRARY :
                        ((app_mode == APP_MODE_Q6_ARBITRARY) ?
                         Q4_MOTION_PROFILE_ARBITRARY :
                         Q4_MOTION_PROFILE_CENTER));
                    if (app_mode_is_q6_family(app_mode)) {
                        if (app_mode == APP_MODE_Q7_ARBITRARY) {
                            ball_balance_hold_q7(clamp_f32(
                                (float)k230_control.target_mm,
                                -150.0f, 150.0f));
                        } else {
                            /* Mode 3 always holds the calibrated center. */
                            ball_balance_hold_q6(0.0f);
                        }
                        q6_curve_feedforward_deg = 0.0f;
                        q6_curve_active = false;
                        q6_curve_candidate_ms = 0U;
                        q6_curve_enter_ms = 0U;
                        q6_straight_since_ms = 0U;
                        ball_balance_set_curve_feedforward(0.0f);
                    } else {
                        ball_balance_hold_q4(Q4_BALL_TARGET_MM);
                    }
                    /*
                     * 与 B15 启动事件同周期置入计划加速度前馈，不等待
                     * 编码器脉冲或下一帧 K230 数据。
                     */
                    ball_balance_set_initial_feedforward(
                        q4_motion_get_start_feedforward_deg());
                    if (!stepper_arm_send_pending_now(g_ms_ticks)) {
                        ball_balance_stop();
                        mode_started = false;
                        debug_puts("MODE Q4 BLOCK X42S NOT READY\r\n");
                    } else {
                        mode_started = true;
                        q4_preload_pending = true;
                        q4_resume_after_preload = line_paused;
                        q4_preload_start_ms = g_ms_ticks;
                        debug_puts("MODE Q4 PRELOAD LEAD WAIT\r\n");
                    }
                }
            } else {
                if (mode_started) {
                    stepper_arm_stop();
                    mode_started = false;
                    debug_puts("MODE STEPPER PAUSE\r\n");
                } else {
                    stop_line_outputs(&tune);
                    mode_started = true;
                    line_paused = false;
                    debug_puts("MODE STEPPER START\r\n");
                    debug_print_stepper_state("STEPPER READY");
                }
            }
        }

        /*
         * 补偿领先采用非阻塞计时：等待期间通信和X42S服务正常运行，
         * 但底盘保持停止；计时结束后才启动Q4运动时间轴和循迹。
         */
        if ((app_mode == APP_MODE_Q3_SEQUENCE) && mode_started &&
            k230_control_ok && (k230_control.mode == 6U) &&
            (k230_control.run_state == 1U)) {
            if (k230_control.sequence_phase != q3_last_sequence_phase) {
                q3_last_sequence_phase = k230_control.sequence_phase;
                debug_puts("MODE Q3 PHASE ");
                debug_print_i32((int32_t)q3_last_sequence_phase);
                debug_puts(" TARGET ");
                debug_print_i32((int32_t)k230_control.target_mm);
                debug_puts("\r\n");
            }
        }

        if (q4_preload_pending) {
            tune.run = false;
            tune.line_active = false;
            left_pwm = 0;
            right_pwm = 0;
            motor_stop();

            if ((g_ms_ticks - q4_preload_start_ms) >=
                q4_motion_get_preload_lead_ms()) {
                motion_state_init(&motion);
                q4_motion_start(g_ms_ticks);
                q4_preload_pending = false;
                line_paused = false;
                if (app_mode_is_q6_family(app_mode)) {
                    line_lap_phase = LINE_LAP_RUNNING;
                    a_confirm_frames = 0U;
                    a_release_frames = 0U;
                    a_released_start = false;
                    line_lap_counts = 0U;
                    a_baseline_total = 1U;
                    a_baseline_span = 1U;
                    __disable_irq();
                    g_run_time_ms = 0U;
                    __enable_irq();
                    g_run_timer_enabled = true;
                }
                if (q4_resume_after_preload) {
                    debug_puts("MODE Q4 RESUME AFTER PRELOAD\r\n");
                    resume_line_tracking(&tune);
                } else {
                    debug_puts("MODE Q4 START AFTER PRELOAD\r\n");
                    start_line_tracking(&tune);
                }
            }
        }

        if (app_mode == APP_MODE_LINE_TIMER) {
            if (b11_press_event()) {
                clear_line_timer();
                line_lap_phase = tune.run ? LINE_LAP_WAIT_START_A :
                    LINE_LAP_IDLE;
                a_confirm_frames = 0U;
                a_release_frames = 0U;
                a_released_start = false;
                line_lap_counts = 0U;
                a_baseline_total = 1U;
                a_baseline_span = 1U;
            }
            if (b14_press_event() && tune.run) {
                line_paused = true;
                pause_line_tracking(&tune);
            }
        }

        /*
         * 模式二尚未启动时，B11/B14 可直接点动 X42S，便于赛前检查
         * 水管角度。运行后禁止人工点动，避免与滚球平衡控制同时写目标角。
         * 模式一中的 B11 清零、B14 暂停语义保持不变。
         */
        if ((app_mode_is_ball_drive(app_mode) && !mode_started && !tune.run) ||
            ((app_mode == APP_MODE_STEPPER_MANUAL) && mode_started)) {
            stepper_jog_buttons_service(&g_stepper_mode_params);
        }

        if (tune.brake_request) {
            left_pwm = 0;
            right_pwm = 0;
            motor_brake();
            brake_until_ms = g_ms_ticks + tune.brake_ms;
            tune.brake_request = false;
        }

        if ((g_ms_ticks - last_control_ms) >= SPEED_CTRL_PERIOD_MS) {
            int32_t left_delta;
            int32_t right_delta;
            int32_t target_left_pwm = 0;
            int32_t target_right_pwm = 0;
            float drive_target = tune.target;
            int32_t drive_base_pwm = tune.base_pwm;
            float drive_ramp_ratio = 1.0f;
            uint32_t control_elapsed_ms = g_ms_ticks - last_control_ms;

            if (control_elapsed_ms > CONTROL_LAG_WARN_MS) {
                last_control_ms = g_ms_ticks;
#if CONTROL_LAG_DEBUG_ENABLE
                if ((g_ms_ticks - last_lag_report_ms) >= 1000U) {
                    last_lag_report_ms = g_ms_ticks;
                    debug_puts("CTRL LAG MS ");
                    debug_print_i32((int32_t)control_elapsed_ms);
                    debug_puts("\r\n");
                }
#endif
            } else {
                last_control_ms += SPEED_CTRL_PERIOD_MS;
            }
            encoder_get_delta(&left_delta, &right_delta);
            motion_state_update(&motion, left_delta, right_delta,
                SPEED_CTRL_PERIOD_MS);
            left_speed = (left_delta < 0) ? (int)(-left_delta) : (int)left_delta;
            right_speed = (right_delta < 0) ? (int)(-right_delta) : (int)right_delta;

            if (app_mode_is_ball_drive(app_mode) && tune.run &&
                q4_motion_is_active()) {
                q4_motion_get_setpoint(g_ms_ticks, motion.speed_cps,
                    motion.accel_cps2, &q4_setpoint);
                drive_target = q4_setpoint.speed_target_counts;
                drive_base_pwm = q4_setpoint.base_pwm;
                drive_ramp_ratio = q4_setpoint.ramp_ratio;
                ball_balance_set_feedforward(
                    q4_setpoint.pipe_feedforward_deg);
                if (app_mode_is_q6_family(app_mode)) {
                    float feedback_scale = 0.0f;
                    uint32_t initial_hold_ms =
                        (app_mode == APP_MODE_Q7_ARBITRARY) ?
                        Q7_INITIAL_ANGLE_HOLD_MS : Q6_INITIAL_ANGLE_HOLD_MS;
                    uint32_t feedback_blend_ms =
                        (app_mode == APP_MODE_Q7_ARBITRARY) ?
                        Q7_BALL_FEEDBACK_BLEND_MS : Q6_BALL_FEEDBACK_BLEND_MS;

                    if (q4_setpoint.elapsed_ms >
                        initial_hold_ms) {
                        feedback_scale = (float)(
                            q4_setpoint.elapsed_ms -
                            initial_hold_ms) /
                            (float)feedback_blend_ms;
                    }
                    ball_balance_set_feedback_scale(clamp_f32(
                        feedback_scale, 0.0f, 1.0f));
                }

                if (q4_setpoint.passed_b_time &&
                    !q4_b_time_reported) {
                    q4_b_time_reported = true;
                    debug_puts("Q4 B TARGET TIME ");
                    debug_print_i32((int32_t)q4_setpoint.elapsed_ms);
                    debug_puts("\r\n");
                }
            } else if (app_mode_is_ball_drive(app_mode) &&
                       !q4_preload_pending) {
                ball_balance_set_feedforward(0.0f);
            }

            if (((app_mode == APP_MODE_LINE_TIMER) ||
                 app_mode_is_q6_family(app_mode)) &&
                (line_lap_phase == LINE_LAP_RUNNING)) {
                uint32_t left_abs = (left_delta < 0) ?
                    (uint32_t)(-left_delta) : (uint32_t)left_delta;
                uint32_t right_abs = (right_delta < 0) ?
                    (uint32_t)(-right_delta) : (uint32_t)right_delta;

                line_lap_counts += (left_abs + right_abs) / 2U;
            }

            if (tune.line_active) {
                float left_target;
                float right_target;
                float base_target;
                int16_t speed_delta;
                int16_t curve_error_start =
                    (app_mode == APP_MODE_Q7_ARBITRARY) ?
                    Q7_LINE_CURVE_ERR_START :
                    ((app_mode == APP_MODE_Q6_ARBITRARY) ?
                     Q6_LINE_CURVE_ERR_START :
                    ((app_mode == APP_MODE_Q4_AB) ?
                     Q4_LINE_CURVE_ERR_START : LINE_CURVE_ERR_START));
                float curve_target_drop =
                    (app_mode == APP_MODE_Q7_ARBITRARY) ?
                    Q7_LINE_CURVE_TARGET_DROP :
                    ((app_mode == APP_MODE_Q6_ARBITRARY) ?
                     Q6_LINE_CURVE_TARGET_DROP :
                    ((app_mode == APP_MODE_Q4_AB) ?
                     Q4_LINE_CURVE_TARGET_DROP : LINE_CURVE_TARGET_DROP));

                gray_raw = gray_sensor_read_raw();
                speed_delta = line_tracker_update(&tracker, gray_raw,
                    tune.line_kp, tune.line_kd, tune.line_corr_limit);
                if (app_mode_is_ball_drive(app_mode)) {
                    speed_delta = (int16_t)(
                        (float)speed_delta * drive_ramp_ratio);
                }
                tune.line_bits = tracker.active_bits;
                tune.line_error = tracker.error;
                tune.line_correction = speed_delta;
                tune.line_valid = tracker.valid;

                if (app_mode_is_q6_family(app_mode)) {
                    uint32_t curve_arm_delay_ms =
                        (app_mode == APP_MODE_Q7_ARBITRARY) ?
                        Q7_CURVE_ARM_DELAY_MS : Q6_CURVE_ARM_DELAY_MS;

                    if (q4_setpoint.elapsed_ms >=
                        curve_arm_delay_ms) {
                        q6_curve_feedforward_deg =
                            q6_curve_feedforward_update(
                                q6_curve_feedforward_deg,
                                tracker.error, tracker.valid,
                                app_mode == APP_MODE_Q7_ARBITRARY,
                                g_ms_ticks,
                                &q6_curve_active,
                                &q6_curve_candidate_ms,
                                &q6_curve_enter_ms,
                                &q6_straight_since_ms);
                    } else {
                        q6_curve_feedforward_deg = 0.0f;
                        q6_curve_active = false;
                        q6_curve_candidate_ms = 0U;
                        q6_curve_enter_ms = 0U;
                        q6_straight_since_ms = 0U;
                    }
                    ball_balance_set_curve_feedforward(
                        q6_curve_feedforward_deg);
                }

                base_target = drive_target;
                if (tracker.valid &&
                    ((tracker.error >= curve_error_start) ||
                     (tracker.error <= -curve_error_start))) {
                    base_target -= curve_target_drop *
                        drive_ramp_ratio;
                    if (!app_mode_is_ball_drive(app_mode) &&
                        (base_target < LINE_MIN_WHEEL_TARGET)) {
                        base_target = LINE_MIN_WHEEL_TARGET;
                    } else if (base_target < 0.0f) {
                        base_target = 0.0f;
                    }
                }

                left_target = clamp_f32(base_target + (float)speed_delta,
                    app_mode_is_ball_drive(app_mode) ? 0.0f :
                        LINE_MIN_WHEEL_TARGET,
                    drive_target +
                        ((float)tune.line_corr_limit * drive_ramp_ratio));
                right_target = clamp_f32(base_target - (float)speed_delta,
                    app_mode_is_ball_drive(app_mode) ? 0.0f :
                        LINE_MIN_WHEEL_TARGET,
                    drive_target +
                        ((float)tune.line_corr_limit * drive_ramp_ratio));
                if (!app_mode_is_ball_drive(app_mode)) {
                    pid_set_target(&left_pid, left_target);
                    pid_set_target(&right_pid, right_target);
                }

                if (((app_mode == APP_MODE_LINE_TIMER) ||
                     app_mode_is_q6_family(app_mode)) &&
                    (line_lap_phase != LINE_LAP_IDLE) &&
                    (line_lap_phase != LINE_LAP_FINISHED)) {
                    AMarkStats a_stats = line_a_mark_update(gray_raw, &tune);

                    bool enough_odom =
                        line_lap_counts >= tune.a_min_lap_counts;
                    bool enough_time =
                        g_run_time_ms >= tune.a_min_lap_ms;
                    bool width_jump =
                        (a_stats.total_count >=
                         (uint8_t)(a_baseline_total + A_MARK_JUMP_TOTAL)) ||
                        (a_stats.span >=
                         (uint8_t)(a_baseline_span + A_MARK_JUMP_SPAN));
                    bool finish_hit = a_stats.finish_candidate &&
                        enough_time && enough_odom && width_jump;

                    if (tune.a_debug_enable && a_released_start &&
                        a_stats.finish_candidate &&
                        ((g_ms_ticks - last_a_gate_debug_ms) >= 500U)) {
                        last_a_gate_debug_ms = g_ms_ticks;
                        debug_print_a_gate(finish_hit ? "AHIT" : "AREJ",
                            run_time_snapshot_ms(), gray_raw, &a_stats,
                            line_lap_counts, a_baseline_total,
                            a_baseline_span, enough_time, enough_odom,
                            width_jump);
                    }

                    if ((line_lap_phase == LINE_LAP_RUNNING) &&
                        a_released_start && !finish_hit &&
                        (!enough_time || !enough_odom ||
                         !a_stats.finish_candidate)) {
                        a_baseline_total = (uint8_t)
                            (((uint16_t)a_baseline_total *
                              (A_MARK_BASELINE_ALPHA - 1U) +
                              a_stats.total_count) /
                             A_MARK_BASELINE_ALPHA);
                        a_baseline_span = (uint8_t)
                            (((uint16_t)a_baseline_span *
                              (A_MARK_BASELINE_ALPHA - 1U) +
                              a_stats.span) /
                             A_MARK_BASELINE_ALPHA);
                    }

                    if (line_lap_phase == LINE_LAP_WAIT_START_A &&
                        a_stats.candidate) {
                        if (a_confirm_frames < 255U) {
                            a_confirm_frames++;
                        }
                    } else if (line_lap_phase == LINE_LAP_RUNNING &&
                        a_released_start && finish_hit) {
                        if (a_confirm_frames < 255U) {
                            a_confirm_frames++;
                        }
                    } else if (line_lap_phase == LINE_LAP_RUNNING) {
                        a_confirm_frames = 0U;
                    } else {
                        a_confirm_frames = 0U;
                    }

                    if (line_lap_phase == LINE_LAP_RUNNING) {
                        bool a_like = a_stats.candidate ||
                            a_stats.finish_candidate;

                        if (a_like) {
                            a_release_frames = 0U;
                        } else if (a_release_frames < 255U) {
                            a_release_frames++;
                        }
                    }

                    if (line_lap_phase == LINE_LAP_WAIT_START_A) {
                        if (a_confirm_frames >= tune.a_confirm_frames) {
                            __disable_irq();
                            g_run_time_ms = 0U;
                            __enable_irq();
                            g_run_timer_enabled = true;
                            line_lap_phase = LINE_LAP_RUNNING;
                            a_confirm_frames = 0U;
                            a_release_frames = 0U;
                            a_released_start = false;
                            line_lap_counts = 0U;
                            a_baseline_total = 1U;
                            a_baseline_span = 1U;
                            debug_puts("A START\r\n");
                        }
                    } else if (line_lap_phase == LINE_LAP_RUNNING) {
                        if (!a_released_start &&
                            (a_release_frames >= A_MARK_RELEASE_FRAMES)) {
                            a_released_start = true;
                            debug_puts("A LEAVE\r\n");
                        }
                        if (a_released_start && finish_hit &&
                            (a_confirm_frames >= tune.a_confirm_frames)) {
                            g_run_timer_enabled = false;
                            line_lap_phase = LINE_LAP_FINISHED;
                            mode_started = true;
                            if (!app_mode_is_q6_family(app_mode)) {
                                tune.run = false;
                                tune.line_active = false;
                                tune.brake_request = true;
                            }
                            debug_puts("A FINISH TIME ");
                            debug_print_i32((int32_t)run_time_snapshot_ms());
                            debug_puts("\r\n");
                        }
                    }

                    if (tune.a_debug_enable &&
                        ((g_ms_ticks - last_a_debug_ms) >=
                        A_MARK_DEBUG_PERIOD_MS)) {
                        last_a_debug_ms = g_ms_ticks;
                        debug_print_a_status(line_lap_phase,
                            run_time_snapshot_ms(), gray_raw, &a_stats,
                            a_confirm_frames, &tune, line_lap_counts,
                            a_baseline_total, a_baseline_span);
                    }
                }
            } else if (!app_mode_is_ball_drive(app_mode)) {
                pid_set_target(&left_pid, drive_target);
                pid_set_target(&right_pid, drive_target);
            }

            if (brake_until_ms != 0U) {
                if ((int32_t)(g_ms_ticks - brake_until_ms) < 0) {
                    left_pwm = 0;
                    right_pwm = 0;
                    motor_brake();
                } else {
                    brake_until_ms = 0U;
                    motor_stop();
                }
            } else if (tune.run) {
                if (app_mode_is_ball_drive(app_mode)) {
                    int32_t line_pwm = tune.line_active ?
                        (int32_t)tune.line_correction : 0;
                    int32_t q4_base_pwm = drive_base_pwm;
                    int32_t line_error_abs = (tune.line_error < 0) ?
                        -(int32_t)tune.line_error :
                        (int32_t)tune.line_error;
                    int16_t line_pwm_limit =
                        (app_mode == APP_MODE_Q7_ARBITRARY) ?
                        Q7_LINE_PWM_LIMIT :
                        ((app_mode == APP_MODE_Q6_ARBITRARY) ?
                         Q6_LINE_PWM_LIMIT : Q4_LINE_PWM_LIMIT);
                    int16_t curve_error_start =
                        (app_mode == APP_MODE_Q7_ARBITRARY) ?
                        Q7_LINE_CURVE_ERR_START :
                        ((app_mode == APP_MODE_Q6_ARBITRARY) ?
                         Q6_LINE_CURVE_ERR_START : Q4_LINE_CURVE_ERR_START);
                    int16_t curve_pwm_drop =
                        (app_mode == APP_MODE_Q7_ARBITRARY) ?
                        Q7_LINE_CURVE_PWM_DROP :
                        ((app_mode == APP_MODE_Q6_ARBITRARY) ?
                         Q6_LINE_CURVE_PWM_DROP : Q4_LINE_CURVE_PWM_DROP);
                    int16_t start_pwm =
                        (app_mode == APP_MODE_Q7_ARBITRARY) ? Q7_START_PWM :
                        ((app_mode == APP_MODE_Q6_ARBITRARY) ?
                         Q6_START_PWM : Q4_START_PWM);

                    /*
                     * 模式二彻底绕过速度 PID。唯一主命令是 q4_motion 生成的
                     * 线性 PWM；循迹只允许在左右轮之间施加小幅差值，不能
                     * 整体抬高驱动强度，因此不会在起步阶段额外加力。
                     */
                    if (line_pwm > line_pwm_limit) {
                        line_pwm = line_pwm_limit;
                    } else if (line_pwm < -line_pwm_limit) {
                        line_pwm = -line_pwm_limit;
                    }
                    if (tune.line_active &&
                        (line_error_abs >= curve_error_start)) {
                        q4_base_pwm -= (int32_t)(
                            (float)curve_pwm_drop *
                            drive_ramp_ratio);
                    }
                    if (q4_base_pwm < start_pwm) {
                        q4_base_pwm = start_pwm;
                    }
                    target_left_pwm = q4_base_pwm +
                        (int32_t)((float)tune.left_pwm_trim *
                            drive_ramp_ratio) + line_pwm;
                    target_right_pwm = q4_base_pwm +
                        (int32_t)((float)tune.right_pwm_trim *
                            drive_ramp_ratio) - line_pwm;
                    if (target_left_pwm < 0) {
                        target_left_pwm = 0;
                    }
                    if (target_right_pwm < 0) {
                        target_right_pwm = 0;
                    }
                } else {
                    target_left_pwm = drive_base_pwm +
                                      (int32_t)tune.left_pwm_trim +
                                      (int32_t)pid_update(
                                          &left_pid, (float)left_speed);
                    target_right_pwm = drive_base_pwm +
                                       (int32_t)tune.right_pwm_trim +
                                       (int32_t)pid_update(
                                           &right_pid, (float)right_speed);
                }
            } else {
                pid_reset(&left_pid);
                pid_reset(&right_pid);
            }

            /*
             * 模式二使用独立的极小 PWM 变化率。即使速度闭环或悬空编码器
             * 瞬间给出大误差，输出也只能逐步增加，不可能突然冲到高转速。
             */
            {
                int16_t pwm_step = (app_mode == APP_MODE_Q7_ARBITRARY) ?
                    Q7_PWM_STEP_LIMIT :
                    ((app_mode == APP_MODE_Q6_ARBITRARY) ?
                    Q6_PWM_STEP_LIMIT :
                    ((app_mode == APP_MODE_Q4_AB) ?
                     Q4_PWM_STEP_LIMIT : PWM_STEP_LIMIT));

                left_pwm = ramp_to(left_pwm, clamp_pwm(target_left_pwm),
                    pwm_step);
                right_pwm = ramp_to(right_pwm, clamp_pwm(target_right_pwm),
                    pwm_step);
            }

            if (tune.run) {
                motor_set_left(left_pwm);
                motor_set_right(right_pwm);
            } else if (brake_until_ms == 0U) {
                left_pwm = 0;
                right_pwm = 0;
                motor_stop();
            }
        }

        ball_sample_ok = k230_ball_get_sample(&ball_sample, g_ms_ticks);
        ball_balance_update(g_ms_ticks, ball_sample_ok, &ball_sample);
        stepper_arm_service(g_ms_ticks);
        if (!x42s_version_reported) {
            StepperArmState x42s_state = stepper_arm_get_state();

            if (x42s_state.firmware_version != 0U) {
                x42s_version_reported = true;
                debug_print_stepper_state("X42S VERSION");
            }
        }

        if ((g_ms_ticks - last_k230_status_ms) >= K230_STATUS_PERIOD_MS) {
            BallBalanceState ball_state = ball_balance_get_state();
            StepperArmState x42s_state = stepper_arm_get_state();
            uint8_t reported_phase = (uint8_t)ball_state.mode;
            uint8_t status_flags = 0U;
            bool ball_control_running =
                (ball_state.mode != BALL_MODE_IDLE) &&
                (ball_state.mode != BALL_MODE_DONE) &&
                (ball_state.mode != BALL_MODE_ERROR);

            /*
             * K230 仅消费 0x81 状态帧，不反向接管 B5/B15 的模式逻辑。
             * 这样模式一、模式二仍完全由天猛星按键决定，视觉端只负责显示。
             */
            if (tune.run || ball_control_running) {
                status_flags |= K230_MCU_FLAG_RUNNING;
            }
            if (ball_sample_ok) {
                status_flags |= K230_MCU_FLAG_SAMPLE_VALID;
            }
            if (ball_state.mode == BALL_MODE_DONE) {
                status_flags |= K230_MCU_FLAG_TASK_DONE;
            }
            if ((ball_state.mode == BALL_MODE_ERROR) ||
                !x42s_state.communication_ok || x42s_state.stall_fault) {
                status_flags |= K230_MCU_FLAG_FAULT;
            }
            if (x42s_state.communication_ok) {
                status_flags |= K230_MCU_FLAG_X42S_ONLINE;
            }
            if (app_mode == APP_MODE_Q4_AB) {
                status_flags |= K230_MCU_FLAG_MODE_Q4;
            } else if (app_mode_is_q6_family(app_mode)) {
                status_flags |= K230_MCU_FLAG_MODE_Q6;
            }

            if (app_mode == APP_MODE_Q3_SEQUENCE) {
                reported_phase = ball_balance_get_q3_report_phase();
            }
            k230_ball_send_mcu_status(reported_phase,
                float_to_i16_saturated(ball_state.target_mm),
                float_to_i16_saturated(ball_state.theta_deg * 100.0f),
                float_to_i16_saturated(ball_state.velocity_mm_s),
                status_flags);
            last_k230_status_ms = g_ms_ticks;
        }

#if OLED_RUNTIME_ENABLE
        if (oled_ready && (!tune.run || !tune.line_active ||
            (line_lap_phase == LINE_LAP_FINISHED)) &&
            ((g_ms_ticks - last_oled_ms) >= 500U)) {
            last_oled_ms = g_ms_ticks;
            if ((app_mode == APP_MODE_LINE_TIMER) ||
                app_mode_is_ball_drive(app_mode) ||
                (app_mode == APP_MODE_Q3_SEQUENCE)) {
                BallBalanceState ball_state = ball_balance_get_state();
                StepperArmState x42s_state = stepper_arm_get_state();
                bool display_running =
                    (app_mode == APP_MODE_Q3_SEQUENCE) ?
                    mode_started : tune.run;

                oled_show_line_mode(g_run_time_ms, mode_started,
                    display_running,
                    line_paused, (uint8_t)app_mode + 1U,
                    (int32_t)ball_state.position_mm,
                    x42s_state.communication_ok);
            } else {
                StepperArmState stepper_state = stepper_arm_get_state();

                oled_show_stepper_mode(&stepper_state, mode_started);
            }
        }
#endif

    }
}
