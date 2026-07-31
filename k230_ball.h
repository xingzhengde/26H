#ifndef K230_BALL_H
#define K230_BALL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float x_mm;
    uint8_t confidence;
    uint32_t timestamp_ms;
    uint16_t frame_id;
    uint16_t frame_time_ms;
    uint8_t vision_latency_ms;
    bool frame_meta_valid;
    bool vision_latency_valid;
    bool valid;
} K230BallSample;

/*
 * 0x81 MCU status 帧的 flags 定义。低四位与 main_new.py 的显示和
 * 完成判定保持一致；高位额外提供模式和 X42S 在线信息，旧接收端会忽略。
 */
#define K230_MCU_FLAG_RUNNING       (0x01U)
#define K230_MCU_FLAG_SAMPLE_VALID  (0x02U)
#define K230_MCU_FLAG_TASK_DONE     (0x04U)
#define K230_MCU_FLAG_FAULT         (0x08U)
#define K230_MCU_FLAG_X42S_ONLINE   (0x10U)
#define K230_MCU_FLAG_MODE_Q4       (0x20U)

void k230_ball_init(void);
void k230_ball_handle_irq(uint32_t now_ms);
bool k230_ball_get_sample(K230BallSample *sample, uint32_t now_ms);
void k230_ball_send_mcu_status(uint8_t phase, int16_t target_mm,
                              int16_t angle_cdeg, int16_t velocity_mm_s,
                              uint8_t flags);

#endif
