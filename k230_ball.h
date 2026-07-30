#ifndef K230_BALL_H
#define K230_BALL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float x_mm;
    uint8_t confidence;
    uint32_t timestamp_ms;
    bool valid;
} K230BallSample;

void k230_ball_init(void);
void k230_ball_handle_irq(uint32_t now_ms);
bool k230_ball_get_sample(K230BallSample *sample, uint32_t now_ms);

#endif
