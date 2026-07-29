#include "encoder.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

static volatile int32_t g_left_count;
static volatile int32_t g_right_count;
static uint8_t g_left_state;
static uint8_t g_right_state;

static int read_pin(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U) ? 1 : 0;
}

static int8_t quadrature_step(uint8_t old_state, uint8_t new_state)
{
    static const int8_t table[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0
    };

    return table[((old_state & 0x03U) << 2U) | (new_state & 0x03U)];
}

static uint8_t read_left_state(void)
{
    int a = read_pin(GPIOA, DL_GPIO_PIN_25);
    int b = read_pin(GPIOA, DL_GPIO_PIN_14);

    return (uint8_t)((a << 1) | b);
}

static uint8_t read_right_state(void)
{
    int a = read_pin(GPIOA, DL_GPIO_PIN_26);
    int b = read_pin(GPIOA, DL_GPIO_PIN_27);

    return (uint8_t)((a << 1) | b);
}

static void update_left_from_ab(void)
{
    uint8_t new_state = read_left_state();
    int step = quadrature_step(g_left_state, new_state);

#if LEFT_ENCODER_INVERT
    step = -step;
#endif
    g_left_count += step;
    g_left_state = new_state;
}

static void update_right_from_ab(void)
{
    uint8_t new_state = read_right_state();
    int step = quadrature_step(g_right_state, new_state);

#if RIGHT_ENCODER_INVERT
    step = -step;
#endif
    g_right_count += step;
    g_right_state = new_state;
}

void encoder_init(void)
{
    g_left_count = 0;
    g_right_count = 0;
    g_left_state = read_left_state();
    g_right_state = read_right_state();
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

void encoder_handle_gpio_irq(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        DL_GPIO_PIN_14 | DL_GPIO_PIN_25 | DL_GPIO_PIN_26 | DL_GPIO_PIN_27);

    if ((status & (DL_GPIO_PIN_25 | DL_GPIO_PIN_14)) != 0U) {
        update_left_from_ab();
    }
    if ((status & (DL_GPIO_PIN_26 | DL_GPIO_PIN_27)) != 0U) {
        update_right_from_ab();
    }

    DL_GPIO_clearInterruptStatus(GPIOA, status);
}

void encoder_get_delta(int32_t *left_delta, int32_t *right_delta)
{
    __disable_irq();
    *left_delta = g_left_count;
    *right_delta = g_right_count;
    g_left_count = 0;
    g_right_count = 0;
    __enable_irq();
}

void encoder_clear(void)
{
    __disable_irq();
    g_left_count = 0;
    g_right_count = 0;
    g_left_state = read_left_state();
    g_right_state = read_right_state();
    __enable_irq();
}
