#include "gray_sensor.h"

#include "ti_msp_dl_config.h"

uint8_t gray_sensor_read_raw(void)
{
    uint8_t bits = 0U;

    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_27) != 0U) {
        bits |= (1U << 0);
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_26) != 0U) {
        bits |= (1U << 1);
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_23) != 0U) {
        bits |= (1U << 2);
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13) != 0U) {
        bits |= (1U << 3);
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12) != 0U) {
        bits |= (1U << 4);
    }
    if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_11) != 0U) {
        bits |= (1U << 5);
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_16) != 0U) {
        bits |= (1U << 6);
    }
    if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_10) != 0U) {
        bits |= (1U << 7);
    }
    return bits;
}
