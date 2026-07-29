#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoder_init(void);
void encoder_handle_gpio_irq(void);
void encoder_get_delta(int32_t *left_delta, int32_t *right_delta);
void encoder_clear(void);

#endif
