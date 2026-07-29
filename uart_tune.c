#include "uart_tune.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"
#include <ctype.h>
#include <string.h>

#define RX_BUF_SIZE 128U
#define LINE_SIZE   96U

static volatile uint8_t g_rx_buf[RX_BUF_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;

static void uart_putc(char ch)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)ch);
}

static void uart_puts(const char *text)
{
    while (*text != '\0') {
        uart_putc(*text++);
    }
}

static char *skip_space(char *text)
{
    while ((*text == ' ') || (*text == '\t')) {
        text++;
    }
    return text;
}

static float parse_decimal(const char *text)
{
    int sign = 1;
    int32_t whole = 0;
    int32_t frac = 0;
    int32_t scale = 1;

    text = skip_space((char *)text);
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

static float parse_token_value(char *token, uint8_t prefix_len)
{
    char *colon = strchr(token, ':');

    if (colon != NULL) {
        return parse_decimal(colon + 1);
    }
    return parse_decimal(token + prefix_len);
}

static void uart_print_u32(uint32_t value)
{
    char digits[10];
    uint8_t index = 0;

    if (value == 0U) {
        uart_putc('0');
        return;
    }
    while (value > 0U) {
        digits[index++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (index > 0U) {
        uart_putc(digits[--index]);
    }
}

static void uart_print_i32(int32_t value)
{
    if (value < 0) {
        uart_putc('-');
        value = -value;
    }
    uart_print_u32((uint32_t)value);
}

static void uart_print_3_digits(uint32_t value)
{
    uart_putc((char)('0' + ((value / 100U) % 10U)));
    uart_putc((char)('0' + ((value / 10U) % 10U)));
    uart_putc((char)('0' + (value % 10U)));
}

static void uart_print_float_3(float value)
{
    int32_t scaled = (int32_t)((value * 1000.0f) +
        ((value >= 0.0f) ? 0.5f : -0.5f));

    if (scaled < 0) {
        uart_putc('-');
        scaled = -scaled;
    }
    uart_print_i32(scaled / 1000);
    uart_putc('.');
    uart_print_3_digits((uint32_t)(scaled % 1000));
}

void uart_tune_init(TuneParams *params)
{
    params->kp = DEFAULT_KP;
    params->ki = DEFAULT_KI;
    params->kd = DEFAULT_KD;
    params->target = LINE_DEFAULT_TARGET;
    params->base_pwm = LINE_DEFAULT_BASE_PWM;
    params->left_pwm_trim = DEFAULT_LEFT_PWM_TRIM;
    params->right_pwm_trim = DEFAULT_RIGHT_PWM_TRIM;
    params->brake_ms = DEFAULT_BRAKE_MS;
    params->line_kp = LINE_DEFAULT_KP;
    params->line_kd = LINE_DEFAULT_KD;
    params->line_corr_limit = LINE_CORR_LIMIT;
    params->line_search_pwm = LINE_SEARCH_PWM;
    params->line_error = 0;
    params->line_correction = 0;
    params->line_bits = 0U;
    params->line_active = false;
    params->line_valid = false;
    params->brake_request = false;
    params->run = false;
    params->reset_pid = true;

    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    uart_tune_send_help();
}

void uart_tune_handle_irq(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
            uint16_t next = (uint16_t)((g_rx_head + 1U) % RX_BUF_SIZE);
            uint8_t byte = DL_UART_Main_receiveData(UART_0_INST);
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

void uart_tune_clear_rx(void)
{
    g_rx_head = 0U;
    g_rx_tail = 0U;
    while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
        (void)DL_UART_Main_receiveData(UART_0_INST);
    }
}

static bool rx_pop(char *ch)
{
    if (g_rx_tail == g_rx_head) {
        return false;
    }
    *ch = (char)g_rx_buf[g_rx_tail];
    g_rx_tail = (uint16_t)((g_rx_tail + 1U) % RX_BUF_SIZE);
    return true;
}

static void apply_token(TuneParams *params, char *token)
{
    char cmd;
    float value;

    token = skip_space(token);
    if (*token == '\0') {
        return;
    }
    if ((toupper((unsigned char)token[0]) == 'B') &&
        (toupper((unsigned char)token[1]) == 'L')) {
        params->left_pwm_trim = (int)parse_token_value(token, 2U);
        return;
    }
    if ((toupper((unsigned char)token[0]) == 'B') &&
        (toupper((unsigned char)token[1]) == 'R')) {
        params->right_pwm_trim = (int)parse_token_value(token, 2U);
        return;
    }
    if ((toupper((unsigned char)token[0]) == 'B') &&
        (toupper((unsigned char)token[1]) == 'K')) {
        int brake_ms = (int)parse_token_value(token, 2U);
        if (brake_ms < 0) {
            brake_ms = 0;
        }
        if (brake_ms > 1000) {
            brake_ms = 1000;
        }
        params->brake_ms = (uint16_t)brake_ms;
        return;
    }
    if ((toupper((unsigned char)token[0]) == 'L') &&
        (toupper((unsigned char)token[1]) == 'I')) {
        params->line_active = true;
        params->run = true;
        params->reset_pid = true;
        return;
    }
    if ((toupper((unsigned char)token[0]) == 'L') &&
        (toupper((unsigned char)token[1]) == 'P')) {
        params->line_kp = parse_token_value(token, 2U);
        return;
    }
    if ((toupper((unsigned char)token[0]) == 'L') &&
        (toupper((unsigned char)token[1]) == 'D')) {
        params->line_kd = parse_token_value(token, 2U);
        return;
    }
    if ((toupper((unsigned char)token[0]) == 'L') &&
        (toupper((unsigned char)token[1]) == 'C')) {
        int limit = (int)parse_token_value(token, 2U);
        if (limit < 0) {
            limit = -limit;
        }
        if (limit > (int)(MOTOR_PWM_PERIOD - MOTOR_PWM_DEAD_TICKS)) {
            limit = (int)(MOTOR_PWM_PERIOD - MOTOR_PWM_DEAD_TICKS);
        }
        params->line_corr_limit = (int16_t)limit;
        return;
    }
    if ((toupper((unsigned char)token[0]) == 'L') &&
        (toupper((unsigned char)token[1]) == 'S')) {
        int search_pwm = (int)parse_token_value(token, 2U);
        if (search_pwm < 0) {
            search_pwm = -search_pwm;
        }
        if (search_pwm > (int)(MOTOR_PWM_PERIOD - MOTOR_PWM_DEAD_TICKS)) {
            search_pwm = (int)(MOTOR_PWM_PERIOD - MOTOR_PWM_DEAD_TICKS);
        }
        params->line_search_pwm = (int16_t)search_pwm;
        return;
    }

    cmd = (char)toupper((unsigned char)token[0]);
    value = parse_decimal(token + 1);
    switch (cmd) {
    case 'P':
        params->kp = value;
        params->reset_pid = true;
        break;
    case 'I':
        params->ki = value;
        params->reset_pid = true;
        break;
    case 'D':
        params->kd = value;
        params->reset_pid = true;
        break;
    case 'T':
        params->target = value;
        params->reset_pid = true;
        break;
    case 'B':
        params->base_pwm = (int)value;
        break;
    case 'R':
        params->line_active = true;
        params->run = true;
        params->reset_pid = true;
        break;
    case 'S':
        params->run = false;
        params->line_active = false;
        params->brake_request = true;
        params->reset_pid = true;
        break;
    case 'H':
    case '?':
        uart_tune_send_help();
        break;
    default:
        break;
    }
}

static void parse_line(TuneParams *params, char *line)
{
    char *token = strtok(line, " ,;");

    while (token != NULL) {
        apply_token(params, token);
        token = strtok(NULL, " ,;");
    }

    uart_puts("OK P");
    uart_print_float_3(params->kp);
    uart_puts(" I");
    uart_print_float_3(params->ki);
    uart_puts(" D");
    uart_print_float_3(params->kd);
    uart_puts(" T");
    uart_print_float_3(params->target);
    uart_puts(" B");
    uart_print_i32(params->base_pwm);
    uart_puts(" BL");
    uart_print_i32(params->left_pwm_trim);
    uart_puts(" BR");
    uart_print_i32(params->right_pwm_trim);
    uart_puts(" BK");
    uart_print_i32(params->brake_ms);
    uart_puts(" LINE LP");
    uart_print_float_3(params->line_kp);
    uart_puts(" LD");
    uart_print_float_3(params->line_kd);
    uart_puts(" LC");
    uart_print_i32(params->line_corr_limit);
    uart_puts(" LS");
    uart_print_i32(params->line_search_pwm);
    uart_putc(' ');
    uart_puts(params->run ? "RUN\r\n" : "STOP\r\n");
}

bool uart_tune_poll(TuneParams *params)
{
    static char line[LINE_SIZE];
    static uint8_t index;
    bool changed = false;
    char ch;

    while (rx_pop(&ch)) {
        if ((ch == '\r') || (ch == '\n')) {
            if (index > 0U) {
                line[index] = '\0';
                parse_line(params, line);
                index = 0;
                changed = true;
            }
        } else if (index < (LINE_SIZE - 1U)) {
            line[index++] = ch;
        }
    }
    return changed;
}

void uart_tune_send_status(const TuneParams *params, int left_speed,
                           int right_speed, int left_pwm, int right_pwm,
                           uint8_t gray_raw, uint8_t key_raw)
{
    static uint32_t timestamp_ms;
    int input = (left_speed + right_speed) / 2;
    int pwm = (left_pwm + right_pwm) / 2;
    float error = params->target - (float)input;

    timestamp_ms += UART_REPORT_PERIOD_MS;
    uart_print_u32(timestamp_ms);
    uart_puts(",SPEED,");
    uart_print_float_3(params->target);
    uart_putc(',');
    uart_print_i32(input);
    uart_putc(',');
    uart_print_i32(pwm);
    uart_putc(',');
    uart_print_float_3(error);
    uart_putc(',');
    uart_print_float_3(params->kp);
    uart_putc(',');
    uart_print_float_3(params->ki);
    uart_putc(',');
    uart_print_float_3(params->kd);
    uart_putc(',');
    uart_print_i32(left_speed);
    uart_putc(',');
    uart_print_i32(right_speed);
    uart_putc(',');
    uart_print_i32(left_pwm);
    uart_putc(',');
    uart_print_i32(right_pwm);
    uart_putc(',');
    uart_print_u32(gray_raw);
    uart_putc(',');
    uart_print_u32(key_raw);
    uart_putc(',');
    uart_puts(params->line_active ? "LINE" : (params->run ? "RUN" : "STOP"));
    uart_putc(',');
    uart_print_u32(params->line_bits);
    uart_putc(',');
    uart_print_i32(params->line_error);
    uart_putc(',');
    uart_print_i32(params->line_correction);
    uart_putc(',');
    uart_print_u32(params->line_valid ? 1U : 0U);
    uart_putc(',');
    uart_print_float_3(params->line_kp);
    uart_putc(',');
    uart_print_float_3(params->line_kd);
    uart_putc(',');
    uart_print_i32(params->line_corr_limit);
    uart_putc(',');
    uart_print_i32(params->line_search_pwm);
    uart_puts("\r\n");
}

void uart_tune_send_help(void)
{
    uart_puts("\r\nCascade line PID ready\r\n");
    uart_puts("UART1 PB6=TX PB7=RX 9600 8N1\r\n");
    uart_puts("CSV: timestamp,loop,setpoint,input,pwm,error,p,i,d,left_speed,right_speed,left_pwm,right_pwm,gray_raw,key_raw,run_state,line_bits,line_error,line_corr,line_valid,line_p,line_d,line_limit,line_search_pwm\r\n");
    uart_puts("Cmd: B15/LINE=start B5/S=stop | T120 B240 P0.25 I0.002 D0 LP1.2 LD3 LC110 LS260 BR-14\r\n");
}
