#include "motor.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

static uint16_t limit_abs_pwm(int16_t pwm)
{
    int16_t max_pwm = (int16_t)(MOTOR_PWM_PERIOD - MOTOR_PWM_DEAD_TICKS);

    if (pwm < 0) {
        pwm = (int16_t)(-pwm);
    }
    if (pwm > max_pwm) {
        pwm = max_pwm;
    }
    return (uint16_t)pwm;
}

static uint16_t pwm_to_compare(uint16_t duty)
{
    if (duty >= MOTOR_PWM_PERIOD) {
        return 0U;
    }
    return (uint16_t)(MOTOR_PWM_PERIOD - duty);
}

static void set_pin(GPIO_Regs *port, uint32_t pin, bool level)
{
    if (level) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static void set_left_direction(int16_t pwm)
{
    bool forward = (pwm >= 0);

#if LEFT_MOTOR_INVERT
    forward = !forward;
#endif

    set_pin(GPIOA, DL_GPIO_PIN_16, forward);
    set_pin(GPIOB, DL_GPIO_PIN_24, !forward);
}

static void set_right_direction(int16_t pwm)
{
    bool forward = (pwm >= 0);

#if RIGHT_MOTOR_INVERT
    forward = !forward;
#endif

    set_pin(GPIOB, DL_GPIO_PIN_17, forward);
    set_pin(GPIOB, DL_GPIO_PIN_19, !forward);
}

void motor_init(void)
{
    motor_stop();
    DL_TimerG_startCounter(MOTOR_PWM_INST);
}

void motor_set_left(int16_t pwm)
{
    uint16_t duty = limit_abs_pwm(pwm);

    set_left_direction(pwm);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        pwm_to_compare(duty), DL_TIMER_CC_1_INDEX);
}

void motor_set_right(int16_t pwm)
{
    uint16_t duty = limit_abs_pwm(pwm);

    set_right_direction(pwm);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        pwm_to_compare(duty), DL_TIMER_CC_0_INDEX);
}

void motor_brake(void)
{
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_17 | DL_GPIO_PIN_19 | DL_GPIO_PIN_24);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_16);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 0, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 0, DL_TIMER_CC_1_INDEX);
}

void motor_stop(void)
{
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_17 | DL_GPIO_PIN_19 | DL_GPIO_PIN_24);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        MOTOR_PWM_PERIOD, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        MOTOR_PWM_PERIOD, DL_TIMER_CC_1_INDEX);
}
