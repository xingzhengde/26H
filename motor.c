#include "motor.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

#define MOTOR_PWM_RIGHT_IOMUX      (IOMUX_PINCM34)
#define MOTOR_PWM_RIGHT_FUNC       (IOMUX_PINCM34_PF_TIMG0_CCP0)
#define MOTOR_PWM_LEFT_IOMUX       (IOMUX_PINCM35)
#define MOTOR_PWM_LEFT_FUNC        (IOMUX_PINCM35_PF_TIMG0_CCP1)
#define MOTOR_PWM_PINS             (DL_GPIO_PIN_12 | DL_GPIO_PIN_13)
#define MOTOR_PWM_INST             (TIMG0)

static bool g_pwm_outputs_enabled;
static bool g_pwm_timer_ready;

static void motor_pwm_timer_init(void)
{
    static const DL_TimerG_ClockConfig clock_config = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U
    };
    static const DL_TimerG_PWMConfig pwm_config = {
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period = MOTOR_PWM_PERIOD,
        .isTimerWithFourCC = false,
        .startTimer = DL_TIMER_STOP
    };

    if (g_pwm_timer_ready) {
        return;
    }

    DL_TimerG_reset(MOTOR_PWM_INST);
    DL_TimerG_enablePower(MOTOR_PWM_INST);
    DL_TimerG_setClockConfig(MOTOR_PWM_INST,
        (DL_TimerG_ClockConfig *)&clock_config);
    DL_TimerG_initPWMMode(MOTOR_PWM_INST,
        (DL_TimerG_PWMConfig *)&pwm_config);
    DL_TimerG_setCounterControl(MOTOR_PWM_INST, DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND, DL_TIMER_CLC_CCCTL0_LCOND);
    DL_TimerG_setCaptureCompareOutCtl(MOTOR_PWM_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(MOTOR_PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, MOTOR_PWM_PERIOD,
        DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareOutCtl(MOTOR_PWM_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(MOTOR_PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, MOTOR_PWM_PERIOD,
        DL_TIMER_CC_1_INDEX);
    DL_TimerG_enableClock(MOTOR_PWM_INST);
    DL_TimerG_setCCPDirection(MOTOR_PWM_INST,
        DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    g_pwm_timer_ready = true;
}

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

static void pwm_outputs_gpio_low(void)
{
    DL_GPIO_clearPins(GPIOA, MOTOR_PWM_PINS);
    DL_GPIO_initDigitalOutput(MOTOR_PWM_RIGHT_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_PWM_LEFT_IOMUX);
    DL_GPIO_enableOutput(GPIOA, MOTOR_PWM_PINS);
    DL_GPIO_clearPins(GPIOA, MOTOR_PWM_PINS);
    g_pwm_outputs_enabled = false;
}

static void pwm_outputs_gpio_high(void)
{
    DL_GPIO_setPins(GPIOA, MOTOR_PWM_PINS);
    DL_GPIO_initDigitalOutput(MOTOR_PWM_RIGHT_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_PWM_LEFT_IOMUX);
    DL_GPIO_enableOutput(GPIOA, MOTOR_PWM_PINS);
    DL_GPIO_setPins(GPIOA, MOTOR_PWM_PINS);
    g_pwm_outputs_enabled = false;
}

static void pwm_outputs_peripheral(void)
{
    if (g_pwm_outputs_enabled) {
        return;
    }

    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        MOTOR_PWM_PERIOD, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        MOTOR_PWM_PERIOD, DL_TIMER_CC_1_INDEX);
    DL_GPIO_initPeripheralOutputFunction(MOTOR_PWM_RIGHT_IOMUX,
        MOTOR_PWM_RIGHT_FUNC);
    DL_GPIO_initPeripheralOutputFunction(MOTOR_PWM_LEFT_IOMUX,
        MOTOR_PWM_LEFT_FUNC);
    DL_GPIO_enableOutput(GPIOA, MOTOR_PWM_PINS);
    g_pwm_outputs_enabled = true;
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
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_17 | DL_GPIO_PIN_19 | DL_GPIO_PIN_24);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16 | MOTOR_PWM_PINS);
    motor_pwm_timer_init();
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        MOTOR_PWM_PERIOD, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        MOTOR_PWM_PERIOD, DL_TIMER_CC_1_INDEX);
    DL_TimerG_startCounter(MOTOR_PWM_INST);
    pwm_outputs_gpio_low();
}

void motor_set_left(int16_t pwm)
{
    uint16_t duty = limit_abs_pwm(pwm);

    pwm_outputs_peripheral();
    set_left_direction(pwm);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        pwm_to_compare(duty), DL_TIMER_CC_1_INDEX);
}

void motor_set_right(int16_t pwm)
{
    uint16_t duty = limit_abs_pwm(pwm);

    pwm_outputs_peripheral();
    set_right_direction(pwm);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        pwm_to_compare(duty), DL_TIMER_CC_0_INDEX);
}

void motor_brake(void)
{
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_17 | DL_GPIO_PIN_19 | DL_GPIO_PIN_24);
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_16);
    pwm_outputs_gpio_high();
}

void motor_stop(void)
{
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_17 | DL_GPIO_PIN_19 | DL_GPIO_PIN_24);
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        MOTOR_PWM_PERIOD, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST,
        MOTOR_PWM_PERIOD, DL_TIMER_CC_1_INDEX);
    pwm_outputs_gpio_low();
}
