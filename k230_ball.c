#include "k230_ball.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

#define K230_RX_BUF_SIZE 96U
#define K230_FRAME_HEAD_0 0xAAU
#define K230_FRAME_HEAD_1 0x55U
#define K230_MSG_BALL_POSITION 0x01U
#define K230_MSG_CONTROL_STATE 0x02U
#define K230_MSG_Q3_TUNE 0x04U
#define K230_MSG_START_ANGLES 0x05U
#define K230_MSG_MCU_STATUS 0x81U
#define K230_MAX_PAYLOAD_SIZE 12U

static volatile uint8_t g_rx_buf[K230_RX_BUF_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint32_t g_irq_time_ms;
static K230BallSample g_sample;
static K230ControlState g_control_state;
static K230Q3TuneParams g_q3_tune;
static K230StartAngleParams g_start_angles;

typedef enum {
    K230_PARSE_HEAD_0 = 0,
    K230_PARSE_HEAD_1,
    K230_PARSE_TYPE,
    K230_PARSE_LENGTH,
    K230_PARSE_PAYLOAD,
    K230_PARSE_CHECKSUM
} K230ParseState;

static K230ParseState g_parse_state;
static uint8_t g_frame_type;
static uint8_t g_frame_length;
static uint8_t g_frame_index;
static uint8_t g_frame_checksum;
static uint8_t g_frame_payload[K230_MAX_PAYLOAD_SIZE];

/**
 * @brief 从中断环形缓冲区取出一个字节。
 *
 * 中断只负责快速搬运字节，协议解析留在主循环执行，避免视觉串口
 * 数据影响电机与步进电机的实时中断。
 */
static bool rx_pop(uint8_t *byte)
{
    if (g_rx_tail == g_rx_head) {
        return false;
    }
    *byte = g_rx_buf[g_rx_tail];
    g_rx_tail = (uint16_t)((g_rx_tail + 1U) % K230_RX_BUF_SIZE);
    return true;
}

/**
 * @brief 处理一帧通过异或校验的数据。
 *
 * K230 的球位置载荷为 little-endian int16 毫米值和置信度百分比。
 * 只有完整且长度正确的帧才更新样本，避免串口错位让摆杆突跳。
 */
static void handle_valid_frame(void)
{
    if ((g_frame_type == K230_MSG_BALL_POSITION) &&
        ((g_frame_length == 3U) || (g_frame_length == 7U) ||
         (g_frame_length == 8U))) {
        int16_t x_mm = (int16_t)((uint16_t)g_frame_payload[0] |
            ((uint16_t)g_frame_payload[1] << 8U));
        uint8_t confidence = g_frame_payload[2];

        if (confidence > 100U) {
            confidence = 100U;
        }
        g_sample.x_mm = (float)x_mm;
        g_sample.confidence = confidence;
        g_sample.timestamp_ms = g_irq_time_ms;
        if (g_frame_length >= 7U) {
            g_sample.frame_id = (uint16_t)g_frame_payload[3] |
                ((uint16_t)g_frame_payload[4] << 8U);
            g_sample.frame_time_ms = (uint16_t)g_frame_payload[5] |
                ((uint16_t)g_frame_payload[6] << 8U);
            g_sample.frame_meta_valid = true;
            if (g_frame_length == 8U) {
                g_sample.vision_latency_ms = g_frame_payload[7];
                g_sample.vision_latency_valid = true;
            } else {
                g_sample.vision_latency_ms = 0U;
                g_sample.vision_latency_valid = false;
            }
        } else {
            /* 兼容旧版 K230 的三字节位置帧。 */
            g_sample.frame_id = 0U;
            g_sample.frame_time_ms = 0U;
            g_sample.frame_meta_valid = false;
            g_sample.vision_latency_ms = 0U;
            g_sample.vision_latency_valid = false;
        }
        g_sample.valid = true;
    } else if ((g_frame_type == K230_MSG_CONTROL_STATE) &&
               (g_frame_length == 5U)) {
        g_control_state.mode = g_frame_payload[0];
        g_control_state.run_state = g_frame_payload[1];
        g_control_state.target_mm = (int16_t)(
            (uint16_t)g_frame_payload[2] |
            ((uint16_t)g_frame_payload[3] << 8U));
        g_control_state.sequence_phase = g_frame_payload[4];
        g_control_state.timestamp_ms = g_irq_time_ms;
        g_control_state.valid = true;
    } else if ((g_frame_type == K230_MSG_Q3_TUNE) &&
               (g_frame_length == 12U)) {
        uint8_t stage;
        bool changed = !g_q3_tune.valid;

        for (stage = 0U; stage < 3U; stage++) {
            uint8_t offset = (uint8_t)(stage * 4U);
            int16_t angle_cdeg = (int16_t)(
                (uint16_t)g_frame_payload[offset] |
                ((uint16_t)g_frame_payload[offset + 1U] << 8U));
            uint16_t time_ms = (uint16_t)(
                (uint16_t)g_frame_payload[offset + 2U] |
                ((uint16_t)g_frame_payload[offset + 3U] << 8U));

            if ((g_q3_tune.angle_cdeg[stage] != angle_cdeg) ||
                (g_q3_tune.time_ms[stage] != time_ms)) {
                changed = true;
            }
            g_q3_tune.angle_cdeg[stage] = angle_cdeg;
            g_q3_tune.time_ms[stage] = time_ms;
        }
        if (changed) {
            g_q3_tune.revision++;
        }
        g_q3_tune.valid = true;
    } else if ((g_frame_type == K230_MSG_START_ANGLES) &&
               ((g_frame_length == 6U) || (g_frame_length == 10U))) {
        uint8_t mode_index;
        bool changed = !g_start_angles.valid;

        for (mode_index = 0U; mode_index < 3U; mode_index++) {
            uint8_t offset = (uint8_t)(mode_index * 2U);
            int16_t angle_cdeg = (int16_t)(
                (uint16_t)g_frame_payload[offset] |
                ((uint16_t)g_frame_payload[offset + 1U] << 8U));

            if (g_start_angles.angle_cdeg[mode_index] != angle_cdeg) {
                changed = true;
            }
            g_start_angles.angle_cdeg[mode_index] = angle_cdeg;
        }
        if (g_frame_length == 10U) {
            for (mode_index = 0U; mode_index < 4U; mode_index++) {
                int8_t target_bias_mm =
                    (int8_t)g_frame_payload[6U + mode_index];

                if (g_start_angles.target_bias_mm[mode_index] !=
                    target_bias_mm) {
                    changed = true;
                }
                g_start_angles.target_bias_mm[mode_index] =
                    target_bias_mm;
            }
        }
        if (changed) {
            g_start_angles.revision++;
        }
        g_start_angles.valid = true;
    }
}

/**
 * @brief 逐字节解析 AA 55 TYPE LEN PAYLOAD XOR 帧。
 *
 * 校验规则与 K230 main.py 的 make_frame() 完全一致：
 * checksum = TYPE XOR LEN XOR 所有 PAYLOAD 字节。
 */
static void parse_byte(uint8_t byte)
{
    switch (g_parse_state) {
    case K230_PARSE_HEAD_0:
        if (byte == K230_FRAME_HEAD_0) {
            g_parse_state = K230_PARSE_HEAD_1;
        }
        break;
    case K230_PARSE_HEAD_1:
        if (byte == K230_FRAME_HEAD_1) {
            g_parse_state = K230_PARSE_TYPE;
        } else {
            g_parse_state = (byte == K230_FRAME_HEAD_0) ?
                K230_PARSE_HEAD_1 : K230_PARSE_HEAD_0;
        }
        break;
    case K230_PARSE_TYPE:
        g_frame_type = byte;
        g_frame_checksum = byte;
        g_parse_state = K230_PARSE_LENGTH;
        break;
    case K230_PARSE_LENGTH:
        if (byte > K230_MAX_PAYLOAD_SIZE) {
            g_parse_state = K230_PARSE_HEAD_0;
            break;
        }
        g_frame_length = byte;
        g_frame_index = 0U;
        g_frame_checksum ^= byte;
        g_parse_state = (byte == 0U) ?
            K230_PARSE_CHECKSUM : K230_PARSE_PAYLOAD;
        break;
    case K230_PARSE_PAYLOAD:
        g_frame_payload[g_frame_index++] = byte;
        g_frame_checksum ^= byte;
        if (g_frame_index >= g_frame_length) {
            g_parse_state = K230_PARSE_CHECKSUM;
        }
        break;
    case K230_PARSE_CHECKSUM:
        if (byte == g_frame_checksum) {
            handle_valid_frame();
        }
        g_parse_state = K230_PARSE_HEAD_0;
        break;
    default:
        g_parse_state = K230_PARSE_HEAD_0;
        break;
    }
}

void k230_ball_init(void)
{
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_parse_state = K230_PARSE_HEAD_0;
    g_frame_type = 0U;
    g_frame_length = 0U;
    g_frame_index = 0U;
    g_frame_checksum = 0U;
    g_sample.frame_meta_valid = false;
    g_sample.vision_latency_valid = false;
    g_sample.valid = false;
    g_control_state.valid = false;
    g_q3_tune.revision = 0U;
    g_q3_tune.valid = false;
    g_start_angles.revision = 0U;
    g_start_angles.target_bias_mm[0] =
        (int8_t)Q6_STRAIGHT_TARGET_BIAS_MM;
    g_start_angles.target_bias_mm[1] =
        (int8_t)Q6_CURVE_TARGET_BIAS_MM;
    g_start_angles.target_bias_mm[2] =
        (int8_t)Q7_STRAIGHT_TARGET_BIAS_MM;
    g_start_angles.target_bias_mm[3] =
        (int8_t)Q7_CURVE_TARGET_BIAS_MM;
    g_start_angles.valid = false;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

void k230_ball_handle_irq(uint32_t now_ms)
{
    g_irq_time_ms = now_ms;
    switch (DL_UART_Main_getPendingInterrupt(UART_2_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_2_INST)) {
            uint16_t next = (uint16_t)((g_rx_head + 1U) % K230_RX_BUF_SIZE);
            uint8_t byte = DL_UART_Main_receiveData(UART_2_INST);

            if (next != g_rx_tail) {
                g_rx_buf[g_rx_head] = byte;
                g_rx_head = next;
            }
        }
        break;
    default:
        break;
    }
}

bool k230_ball_get_sample(K230BallSample *sample, uint32_t now_ms)
{
    uint8_t byte;

    while (rx_pop(&byte)) {
        parse_byte(byte);
    }

    if (!g_sample.valid) {
        return false;
    }
    if ((now_ms - g_sample.timestamp_ms) > K230_BALL_TIMEOUT_MS) {
        return false;
    }
    if (g_sample.confidence < K230_BALL_MIN_CONF) {
        return false;
    }
    *sample = g_sample;
    return true;
}

bool k230_ball_get_control_state(K230ControlState *state, uint32_t now_ms)
{
    uint8_t byte;

    while (rx_pop(&byte)) {
        parse_byte(byte);
    }
    if (!g_control_state.valid ||
        ((now_ms - g_control_state.timestamp_ms) > K230_BALL_TIMEOUT_MS)) {
        return false;
    }
    *state = g_control_state;
    return true;
}

bool k230_ball_get_q3_tune(K230Q3TuneParams *params)
{
    uint8_t byte;

    while (rx_pop(&byte)) {
        parse_byte(byte);
    }
    if (!g_q3_tune.valid || (params == 0)) {
        return false;
    }
    *params = g_q3_tune;
    return true;
}

bool k230_ball_get_start_angles(K230StartAngleParams *params)
{
    uint8_t byte;

    while (rx_pop(&byte)) {
        parse_byte(byte);
    }
    if (!g_start_angles.valid || (params == 0)) {
        return false;
    }
    *params = g_start_angles;
    return true;
}

static void tx_byte(uint8_t byte)
{
    DL_UART_Main_transmitDataBlocking(UART_2_INST, byte);
}

static void put_i16_le(uint8_t *payload, uint8_t offset, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    payload[offset] = (uint8_t)(raw & 0xFFU);
    payload[(uint8_t)(offset + 1U)] = (uint8_t)(raw >> 8U);
}

void k230_ball_send_mcu_status(uint8_t phase, int16_t target_mm,
                              int16_t angle_cdeg, int16_t velocity_mm_s,
                              uint8_t flags)
{
    uint8_t payload[8];
    uint8_t checksum = K230_MSG_MCU_STATUS ^ (uint8_t)sizeof(payload);
    uint8_t i;

    /*
     * main_new.py 固定按以下 8 字节解析：
     * phase, target_mm(i16), angle_cdeg(i16), velocity_mm_s(i16), flags。
     * 状态帧只有 13 字节，100 ms 发送一次，在 115200 baud 下占线很低。
     */
    payload[0] = phase;
    put_i16_le(payload, 1U, target_mm);
    put_i16_le(payload, 3U, angle_cdeg);
    put_i16_le(payload, 5U, velocity_mm_s);
    payload[7] = flags;

    tx_byte(K230_FRAME_HEAD_0);
    tx_byte(K230_FRAME_HEAD_1);
    tx_byte(K230_MSG_MCU_STATUS);
    tx_byte((uint8_t)sizeof(payload));
    for (i = 0U; i < (uint8_t)sizeof(payload); i++) {
        checksum ^= payload[i];
        tx_byte(payload[i]);
    }
    tx_byte(checksum);
}
