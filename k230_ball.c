#include "k230_ball.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"
#include <ctype.h>
#include <string.h>

#define K230_RX_BUF_SIZE 96U
#define K230_LINE_SIZE   48U

static volatile uint8_t g_rx_buf[K230_RX_BUF_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint32_t g_irq_time_ms;
static K230BallSample g_sample;

static float parse_decimal_local(const char *text)
{
    int sign = 1;
    int32_t whole = 0;
    int32_t frac = 0;
    int32_t scale = 1;

    while ((*text == ' ') || (*text == '\t')) {
        text++;
    }
    if (*text == '-') {
        sign = -1;
        text++;
    } else if (*text == '+') {
        text++;
    }
    while ((*text >= '0') && (*text <= '9')) {
        whole = whole * 10 + (*text - '0');
        text++;
    }
    if (*text == '.') {
        text++;
        while ((*text >= '0') && (*text <= '9') && (scale < 1000)) {
            frac = frac * 10 + (*text - '0');
            scale *= 10;
            text++;
        }
    }
    return (float)sign * ((float)whole + ((float)frac / (float)scale));
}

static bool rx_pop(char *ch)
{
    if (g_rx_tail == g_rx_head) {
        return false;
    }
    *ch = (char)g_rx_buf[g_rx_tail];
    g_rx_tail = (uint16_t)((g_rx_tail + 1U) % K230_RX_BUF_SIZE);
    return true;
}

static void parse_line(char *line)
{
    char *token = strtok(line, ", ");
    float x_mm;
    int conf = 100;

    if (token == 0) {
        return;
    }
    if ((toupper((unsigned char)token[0]) == 'B') &&
        (toupper((unsigned char)token[1]) == 'A')) {
        token = strtok(0, ", ");
        if (token == 0) {
            return;
        }
    }
    x_mm = parse_decimal_local(token);
    token = strtok(0, ", ");
    if (token != 0) {
        conf = (int)parse_decimal_local(token);
    }
    if (conf < 0) {
        conf = 0;
    }
    if (conf > 100) {
        conf = 100;
    }
    g_sample.x_mm = x_mm;
    g_sample.confidence = (uint8_t)conf;
    g_sample.timestamp_ms = g_irq_time_ms;
    g_sample.valid = true;
}

void k230_ball_init(void)
{
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_sample.valid = false;
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
    static char line[K230_LINE_SIZE];
    static uint8_t index;
    char ch;

    while (rx_pop(&ch)) {
        if ((ch == '\r') || (ch == '\n')) {
            if (index > 0U) {
                line[index] = '\0';
                parse_line(line);
                index = 0U;
            }
        } else if (index < (K230_LINE_SIZE - 1U)) {
            line[index++] = ch;
        }
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
