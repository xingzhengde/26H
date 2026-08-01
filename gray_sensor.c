#include "gray_sensor.h"

#include "ti_msp_dl_config.h"

uint8_t gray_sensor_read_raw(void)
{
    uint8_t bits = 0U;

    /*
     * 新8路模块校准后为低电平检测到黑线、高电平位于白底。
     * 在驱动层统一转换成“1=黑线”的有效位，保持巡线权重、A点统计
     * 和上层状态机的原有语义不变。bit0=车头最左OUT1，bit7=最右OUT8。
     */
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_27) == 0U) {
        bits |= (1U << 0);
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_26) == 0U) {
        bits |= (1U << 1);
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_23) == 0U) {
        bits |= (1U << 2);
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13) == 0U) {
        bits |= (1U << 3);
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12) == 0U) {
        bits |= (1U << 4);
    }
    if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_11) == 0U) {
        bits |= (1U << 5);
    }
    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_16) == 0U) {
        bits |= (1U << 6);
    }
    if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_10) == 0U) {
        bits |= (1U << 7);
    }
    return bits;
}
