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
    APP_MODE_STEPPER_MANUAL,
    APP_MODE_COUNT
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
        return "LINE";
    case APP_MODE_STEPPER_MANUAL:
        return "STEPPER";
    default:
        return "UNKNOWN";
    }
}

static void apply_line_mode_params(TuneParams *tune)
{
    tune->kp = g_line_mode_params.kp;
    tune->ki = g_line_mode_params.ki;
    tune->kd = g_line_mode_params.kd;
    tune->target = g_line_mode_params.target;
    tune->base_pwm = g_line_mode_params.base_pwm;
    tune->left_pwm_trim = g_line_mode_params.left_pwm_trim;
    tune->right_pwm_trim = g_line_mode_params.right_pwm_trim;
    tune->brake_ms = g_line_mode_params.brake_ms;
    tune->line_kp = g_line_mode_params.line_kp;
    tune->line_kd = g_line_mode_params.line_kd;
    tune->line_corr_limit = g_line_mode_params.line_corr_limit;
    tune->line_search_pwm = g_line_mode_params.line_search_pwm;
    tune->a_total_min = g_line_mode_params.a_total_min;
    tune->a_span_min = g_line_mode_params.a_span_min;
    tune->a_confirm_frames = g_line_mode_params.a_confirm_frames;
    tune->a_min_lap_ms = g_line_mode_params.a_min_lap_ms;
    tune->a_min_lap_counts = g_line_mode_params.a_min_lap_counts;
    tune->a_debug_enable = false;
    tune->reset_pid = true;
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

void STEPPER_PWM_INST_IRQHandler(void)
{
    stepper_arm_handle_irq();
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

            if ((app_mode != APP_MODE_LINE_TIMER) && tune.run) {
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
                pid_set_gains(&left_pid, tune.kp, tune.ki, tune.kd);
                pid_set_gains(&right_pid, tune.kp, tune.ki, tune.kd);
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

        if (b5_press_event()) {
            debug_print_keys("KEY K1 NEXT", &tune);
            stop_line_outputs(&tune);
            ball_balance_stop();
            stepper_arm_stop();
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
            }
            debug_print_mode("MODE SWITCH", app_mode);
        }

        if (b15_press_event()) {
            debug_print_keys("KEY K4 START", &tune);
            if (app_mode == APP_MODE_LINE_TIMER) {
                if (!tune.run) {
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
                } else {
                    debug_puts("MODE LINE ALREADY RUN\r\n");
                }
            } else {
                stop_line_outputs(&tune);
                mode_started = true;
                line_paused = false;
                debug_puts("MODE STEPPER START\r\n");
                debug_print_stepper_state("STEPPER READY");
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

        if ((app_mode == APP_MODE_STEPPER_MANUAL) && mode_started) {
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
            if ((app_mode == APP_MODE_LINE_TIMER) &&
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

                gray_raw = gray_sensor_read_raw();
                speed_delta = line_tracker_update(&tracker, gray_raw,
                    tune.line_kp, tune.line_kd, tune.line_corr_limit);
                tune.line_bits = tracker.active_bits;
                tune.line_error = tracker.error;
                tune.line_correction = speed_delta;
                tune.line_valid = tracker.valid;

                base_target = tune.target;
                if (tracker.valid &&
                    ((tracker.error >= LINE_CURVE_ERR_START) ||
                     (tracker.error <= -LINE_CURVE_ERR_START))) {
                    base_target -= LINE_CURVE_TARGET_DROP;
                    if (base_target < LINE_MIN_WHEEL_TARGET) {
                        base_target = LINE_MIN_WHEEL_TARGET;
                    }
                }

                left_target = clamp_f32(base_target + (float)speed_delta,
                    LINE_MIN_WHEEL_TARGET, tune.target + LINE_CORR_LIMIT);
                right_target = clamp_f32(base_target - (float)speed_delta,
                    LINE_MIN_WHEEL_TARGET, tune.target + LINE_CORR_LIMIT);
                pid_set_target(&left_pid, left_target);
                pid_set_target(&right_pid, right_target);

                if ((app_mode == APP_MODE_LINE_TIMER) &&
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
                            tune.run = false;
                            tune.line_active = false;
                            tune.brake_request = true;
                            g_run_timer_enabled = false;
                            line_lap_phase = LINE_LAP_FINISHED;
                            mode_started = true;
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
            } else {
                pid_set_target(&left_pid, tune.target);
                pid_set_target(&right_pid, tune.target);
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
                target_left_pwm = tune.base_pwm + tune.left_pwm_trim +
                                  (int32_t)pid_update(
                                      &left_pid, (float)left_speed);
                target_right_pwm = tune.base_pwm + tune.right_pwm_trim +
                                   (int32_t)pid_update(
                                       &right_pid, (float)right_speed);
            } else {
                pid_reset(&left_pid);
                pid_reset(&right_pid);
            }

            left_pwm = ramp_to(left_pwm, clamp_pwm(target_left_pwm),
                PWM_STEP_LIMIT);
            right_pwm = ramp_to(right_pwm, clamp_pwm(target_right_pwm),
                PWM_STEP_LIMIT);

            if (tune.run) {
                motor_set_left(left_pwm);
                motor_set_right(right_pwm);
            } else if (brake_until_ms == 0U) {
                left_pwm = 0;
                right_pwm = 0;
                motor_stop();
            }
        }

        {
            bool ball_sample_ok = k230_ball_get_sample(&ball_sample,
                g_ms_ticks);

            ball_balance_update(g_ms_ticks, ball_sample_ok, &ball_sample);
        }

#if OLED_RUNTIME_ENABLE
        if (oled_ready && (!tune.run || !tune.line_active ||
            (line_lap_phase == LINE_LAP_FINISHED)) &&
            ((g_ms_ticks - last_oled_ms) >= 500U)) {
            last_oled_ms = g_ms_ticks;
            if (app_mode == APP_MODE_LINE_TIMER) {
                oled_show_line_mode(g_run_time_ms, mode_started, tune.run,
                    line_paused);
            } else {
                StepperArmState stepper_state = stepper_arm_get_state();

                oled_show_stepper_mode(&stepper_state, mode_started);
            }
        }
#endif

    }
}
