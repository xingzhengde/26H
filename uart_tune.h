#ifndef UART_TUNE_H
#define UART_TUNE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    int base_pwm;
    int left_pwm_trim;
    int right_pwm_trim;
    uint16_t brake_ms;
    float line_kp;
    float line_kd;
    int16_t line_corr_limit;
    int16_t line_search_pwm;
    int16_t line_error;
    int16_t line_correction;
    uint8_t line_bits;
    bool line_active;
    bool line_valid;
    bool brake_request;
    bool run;
    bool reset_pid;
} TuneParams;

void uart_tune_init(TuneParams *params);
void uart_tune_handle_irq(void);
void uart_tune_clear_rx(void);
bool uart_tune_poll(TuneParams *params);
void uart_tune_send_status(const TuneParams *params, int left_speed,
                           int right_speed, int left_pwm, int right_pwm,
                           uint8_t gray_raw, uint8_t key_raw,
                           uint32_t run_time_ms);
void uart_tune_send_help(void);

#endif
