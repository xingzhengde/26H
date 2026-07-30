#include "stepper_arm.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

static volatile int32_t g_current_steps;
static volatile int32_t g_target_steps;
static volatile int32_t g_min_steps;
static volatile int32_t g_max_steps;
static volatile int32_t g_neutral_steps;
static volatile uint32_t g_steps_remaining;
static volatile int8_t g_step_dir;
static volatile bool g_busy;
static volatile bool g_homed;

static int32_t clamp_steps(int32_t steps)
{
    if (steps < STEPPER_MIN_STEPS) {
        steps = STEPPER_MIN_STEPS;
    }
    if (steps > STEPPER_MAX_STEPS) {
        steps = STEPPER_MAX_STEPS;
    }
    if (steps < g_min_steps) {
        return g_min_steps;
    }
    if (steps > g_max_steps) {
        return g_max_steps;
    }
    return steps;
}

static float clamp_f32_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint32_t stepper_period_from_speed(float motor_dps)
{
    float pulse_hz;
    uint32_t period;

    motor_dps = clamp_f32_local(motor_dps, STEPPER_STEP_DEG,
        STEPPER_MAX_DPS);
    pulse_hz = motor_dps / STEPPER_STEP_DEG;
    period = (uint32_t)((float)STEPPER_PWM_INST_CLK_FREQ / pulse_hz);
    if (period > 65535U) {
        period = 65535U;
    }
    if (period < STEPPER_MIN_PERIOD) {
        period = STEPPER_MIN_PERIOD;
    }
    return period;
}

static void stepper_apply_speed(float motor_dps)
{
    uint32_t period = stepper_period_from_speed(motor_dps);

    DL_Timer_setLoadValue(STEPPER_PWM_INST, period);
    DL_TimerG_setCaptureCompareValue(STEPPER_PWM_INST, period / 2U,
        GPIO_STEPPER_PWM_C1_IDX);
}

static void stepper_set_dir(int8_t dir)
{
    if (dir >= 0) {
        DL_GPIO_setPins(GPIO_STEPPER_CTRL_PORT,
            GPIO_STEPPER_CTRL_STEPPER1_DIR_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_STEPPER_CTRL_PORT,
            GPIO_STEPPER_CTRL_STEPPER1_DIR_PIN);
    }
    g_step_dir = dir;
}

void stepper_arm_init(void)
{
    DL_GPIO_setPins(GPIO_STEPPER_CTRL_PORT,
        GPIO_STEPPER_CTRL_STEPPER1_RST_PIN |
            GPIO_STEPPER_CTRL_STEPPER1_SLP_PIN |
            GPIO_STEPPER_CTRL_STEPPER1_DCY_PIN);
    DL_Timer_stopCounter(STEPPER_PWM_INST);
    DL_TimerG_setCaptureCompareValue(STEPPER_PWM_INST, 0U,
        GPIO_STEPPER_PWM_C1_IDX);
    stepper_apply_speed(STEPPER_DEFAULT_DPS);
    NVIC_EnableIRQ(STEPPER_PWM_INST_INT_IRQN);
    g_current_steps = STEPPER_NEUTRAL_STEPS;
    g_target_steps = STEPPER_NEUTRAL_STEPS;
    g_min_steps = STEPPER_MIN_STEPS;
    g_max_steps = STEPPER_MAX_STEPS;
    g_neutral_steps = STEPPER_NEUTRAL_STEPS;
    g_steps_remaining = 0U;
    g_step_dir = 1;
    g_busy = false;
    g_homed = false;
}

void stepper_arm_mark_neutral(void)
{
    __disable_irq();
    g_neutral_steps = g_current_steps;
    g_target_steps = g_current_steps;
    g_steps_remaining = 0U;
    g_busy = false;
    g_homed = true;
    __enable_irq();
    DL_Timer_stopCounter(STEPPER_PWM_INST);
}

void stepper_arm_mark_high_limit(void)
{
    __disable_irq();
    g_min_steps = g_current_steps;
    if (g_neutral_steps < g_min_steps) {
        g_neutral_steps = g_min_steps;
    }
    if (g_max_steps < g_min_steps) {
        g_max_steps = g_min_steps;
    }
    g_target_steps = g_current_steps;
    g_steps_remaining = 0U;
    g_busy = false;
    __enable_irq();
    DL_Timer_stopCounter(STEPPER_PWM_INST);
}

void stepper_arm_mark_low_limit(void)
{
    __disable_irq();
    g_max_steps = g_current_steps;
    if (g_neutral_steps > g_max_steps) {
        g_neutral_steps = g_max_steps;
    }
    if (g_min_steps > g_max_steps) {
        g_min_steps = g_max_steps;
    }
    g_target_steps = g_current_steps;
    g_steps_remaining = 0U;
    g_busy = false;
    __enable_irq();
    DL_Timer_stopCounter(STEPPER_PWM_INST);
}

void stepper_arm_stop(void)
{
    DL_Timer_stopCounter(STEPPER_PWM_INST);
    __disable_irq();
    g_steps_remaining = 0U;
    g_target_steps = g_current_steps;
    g_busy = false;
    __enable_irq();
}

static void stepper_move_to(int32_t target_steps, float speed_dps)
{
    int32_t delta;

    target_steps = clamp_steps(target_steps);
    __disable_irq();
    delta = target_steps - g_current_steps;
    if (delta == 0) {
        g_target_steps = target_steps;
        g_steps_remaining = 0U;
        g_busy = false;
        __enable_irq();
        DL_Timer_stopCounter(STEPPER_PWM_INST);
        return;
    }
    g_target_steps = target_steps;
    g_steps_remaining = (delta > 0) ? (uint32_t)delta : (uint32_t)(-delta);
    g_busy = true;
    __enable_irq();

    stepper_set_dir((delta > 0) ? 1 : -1);
    stepper_apply_speed(speed_dps);
    DL_Timer_startCounter(STEPPER_PWM_INST);
}

void stepper_arm_jog_steps(int32_t delta_steps, float speed_dps)
{
    int32_t base_steps;

    __disable_irq();
    base_steps = g_target_steps;
    __enable_irq();
    stepper_move_to(base_steps + delta_steps, speed_dps);
}

void stepper_arm_set_pipe_angle(float pipe_angle_deg, float speed_dps)
{
    int32_t target_steps = STEPPER_NEUTRAL_STEPS +
        (int32_t)(pipe_angle_deg * STEPPER_STEPS_PER_PIPE_DEG);

    if ((pipe_angle_deg > -0.05f) && (pipe_angle_deg < 0.05f)) {
        StepperArmState state = stepper_arm_get_state();

        if ((state.current_steps >= STEPPER_NEUTRAL_LOW_STEPS) &&
            (state.current_steps <= STEPPER_NEUTRAL_HIGH_STEPS)) {
            return;
        }
    }
    target_steps += (g_neutral_steps - STEPPER_NEUTRAL_STEPS);
    stepper_move_to(target_steps, speed_dps);
}

void stepper_arm_handle_irq(void)
{
    switch (DL_Timer_getPendingInterrupt(STEPPER_PWM_INST)) {
    case DL_TIMER_IIDX_LOAD:
        if (g_steps_remaining == 0U) {
            DL_Timer_stopCounter(STEPPER_PWM_INST);
            g_busy = false;
            break;
        }
        g_current_steps += g_step_dir;
        g_steps_remaining--;
        if (g_steps_remaining == 0U) {
            DL_Timer_stopCounter(STEPPER_PWM_INST);
            g_busy = false;
        }
        break;
    default:
        break;
    }
}

StepperArmState stepper_arm_get_state(void)
{
    StepperArmState state;

    __disable_irq();
    state.current_steps = g_current_steps;
    state.target_steps = g_target_steps;
    state.min_steps = g_min_steps;
    state.max_steps = g_max_steps;
    state.neutral_steps = g_neutral_steps;
    state.busy = g_busy;
    state.homed = g_homed;
    __enable_irq();
    return state;
}
