#include "app_config.h"
#include "encoder.h"
#include "gray_sensor.h"
#include "line_tracker.h"
#include "motion_state.h"
#include "motor.h"
#include "oled.h"
#include "pid.h"
#include "ti_msp_dl_config.h"
#include "uart_tune.h"
#include <stdbool.h>
#include <stdint.h>

static volatile uint32_t g_ms_ticks;
static volatile uint32_t g_run_time_ms;
static volatile bool g_run_timer_enabled;
static bool g_buttons_released_after_boot;

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

static bool b5_pressed(void)
{
    return (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_5) == 0U);
}

static bool b15_pressed(void)
{
    return (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_15) == 0U);
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

static bool buttons_released(void)
{
    return !b5_pressed() && !b15_pressed();
}

static void stop_car(TuneParams *tune)
{
    tune->run = false;
    tune->line_active = false;
    tune->brake_request = true;
    g_run_timer_enabled = false;
    debug_puts("RUN TIME MS ");
    debug_print_i32((int32_t)run_time_snapshot_ms());
    debug_puts("\r\n");
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

static void debug_print_keys(const char *tag, const TuneParams *tune)
{
    debug_puts(tag);
    debug_puts(" B5=");
    debug_print_i32(b5_pressed() ? 1 : 0);
    debug_puts(" B15=");
    debug_print_i32(b15_pressed() ? 1 : 0);
    debug_puts(" RUN=");
    debug_print_i32(tune->run ? 1 : 0);
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
    uint32_t last_lag_report_ms = 0;
    uint32_t last_boot_key_report_ms = 0;
    uint32_t brake_until_ms = 0;
    MotionState motion;
    int left_speed = 0;
    int right_speed = 0;
    int16_t left_pwm = 0;
    int16_t right_pwm = 0;
    uint8_t gray_raw = 0U;
    int oled_status;

    SYSCFG_DL_init();
    motor_init();
    NVIC_EnableIRQ(RUN_TIMER_INST_INT_IRQN);
    __enable_irq();
    while (g_ms_ticks < 200U) {
        __NOP();
    }

    encoder_init();
    motion_state_init(&motion);
    line_tracker_init(&tracker);
    uart_tune_init(&tune);
    uart_tune_clear_rx();
#if OLED_RUNTIME_ENABLE
    oled_status = oled_init();
    if (oled_status >= 0) {
        oled_clear();
        oled_puts(0U, 0U, "OLED:OK");
    } else {
        (void)oled_status;
    }
#else
    (void)oled_status;
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

            if (changed && !was_run && tune.run) {
                g_run_time_ms = 0U;
                g_run_timer_enabled = true;
                debug_puts("RUN TIME START\r\n");
            } else if (changed && was_run && !tune.run) {
                g_run_timer_enabled = false;
                debug_puts("RUN TIME MS ");
                debug_print_i32((int32_t)run_time_snapshot_ms());
                debug_puts("\r\n");
            }

            if (changed) {
            if (tune.run) {
                tune.line_active = true;
            }
            pid_set_gains(&left_pid, tune.kp, tune.ki, tune.kd);
            pid_set_gains(&right_pid, tune.kp, tune.ki, tune.kd);
            }
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
            debug_print_keys("KEY B5 STOP", &tune);
            stop_car(&tune);
        }

        if (b15_press_event()) {
            debug_print_keys("KEY B15 EVENT", &tune);
            if (!tune.run) {
                debug_puts("KEY B15 START\r\n");
                start_line_tracking(&tune);
            } else {
                debug_puts("KEY B15 IGNORED RUNNING\r\n");
            }
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
                if ((g_ms_ticks - last_lag_report_ms) >= 1000U) {
                    last_lag_report_ms = g_ms_ticks;
                    debug_puts("CTRL LAG MS ");
                    debug_print_i32((int32_t)control_elapsed_ms);
                    debug_puts("\r\n");
                }
            } else {
                last_control_ms += SPEED_CTRL_PERIOD_MS;
            }
            encoder_get_delta(&left_delta, &right_delta);
            motion_state_update(&motion, left_delta, right_delta,
                SPEED_CTRL_PERIOD_MS);
            left_speed = (left_delta < 0) ? (int)(-left_delta) : (int)left_delta;
            right_speed = (right_delta < 0) ? (int)(-right_delta) : (int)right_delta;

            if (tune.line_active) {
                float left_target;
                float right_target;
                int16_t speed_delta;

                gray_raw = gray_sensor_read_raw();
                speed_delta = line_tracker_update(&tracker, gray_raw,
                    tune.line_kp, tune.line_kd, tune.line_corr_limit);
                tune.line_bits = tracker.active_bits;
                tune.line_error = tracker.error;
                tune.line_correction = speed_delta;
                tune.line_valid = tracker.valid;

                left_target = clamp_f32(tune.target + (float)speed_delta,
                    LINE_MIN_WHEEL_TARGET, tune.target + LINE_CORR_LIMIT);
                right_target = clamp_f32(tune.target - (float)speed_delta,
                    LINE_MIN_WHEEL_TARGET, tune.target + LINE_CORR_LIMIT);
                pid_set_target(&left_pid, left_target);
                pid_set_target(&right_pid, right_target);
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

#if OLED_RUNTIME_ENABLE
        if ((g_ms_ticks - last_oled_ms) >= 200U) {
            last_oled_ms += 200U;
            oled_show_motion(&motion, g_run_time_ms, tune.run);
        }
#endif

    }
}
