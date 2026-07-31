#ifndef OLED_H
#define OLED_H

#include "app_config.h"
#include "motion_state.h"
#include "stepper_arm.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#define OLED_ADDR_3C    (0x3CU)
#define OLED_ADDR_3D    (0x3DU)
#define OLED_I2C_WAIT   (100000U)

static uint8_t g_oled_addr = OLED_ADDR_3C;

static inline uint8_t oled_get_addr(void)
{
    return g_oled_addr;
}

static inline void oled_set_addr(uint8_t addr)
{
    g_oled_addr = addr;
}

static inline int oled_i2c_wait_idle(void)
{
    uint32_t timeout = OLED_I2C_WAIT;

    while ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--timeout == 0U) {
            return -1;
        }
    }
    return 0;
}

static inline int oled_i2c_wait_done(void)
{
    uint32_t timeout = OLED_I2C_WAIT;

    while ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
             DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            return -2;
        }
        if (--timeout == 0U) {
            return -1;
        }
    }
    return 0;
}

static inline int oled_i2c_write(const uint8_t *data, uint8_t len)
{
    if ((data == 0) || (len == 0U)) {
        return -3;
    }
    if (oled_i2c_wait_idle() != 0) {
        return -1;
    }
    DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
    DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, data, len);
    DL_I2C_startControllerTransfer(I2C_OLED_INST, g_oled_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, len);
    delay_cycles(16);
    return oled_i2c_wait_done();
}

static inline int oled_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00U, cmd};

    return oled_i2c_write(buf, 2U);
}

static inline int oled_data(const uint8_t *data, uint8_t len)
{
    uint8_t buf[8];
    uint8_t offset = 0U;

    while (offset < len) {
        uint8_t chunk = (uint8_t)(len - offset);
        uint8_t i;

        if (chunk > 7U) {
            chunk = 7U;
        }
        buf[0] = 0x40U;
        for (i = 0U; i < chunk; i++) {
            buf[i + 1U] = data[offset + i];
        }
        if (oled_i2c_write(buf, (uint8_t)(chunk + 1U)) != 0) {
            return -1;
        }
        offset = (uint8_t)(offset + chunk);
    }
    return 0;
}

static inline const uint8_t *oled_font5x7(char c)
{
    static const uint8_t digits[10][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E},
        {0x00, 0x42, 0x7F, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46},
        {0x21, 0x41, 0x45, 0x4B, 0x31},
        {0x18, 0x14, 0x12, 0x7F, 0x10},
        {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30},
        {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36},
        {0x06, 0x49, 0x49, 0x29, 0x1E}
    };
    static const uint8_t letters[26][5] = {
        {0x7E, 0x11, 0x11, 0x11, 0x7E},
        {0x7F, 0x49, 0x49, 0x49, 0x36},
        {0x3E, 0x41, 0x41, 0x41, 0x22},
        {0x7F, 0x41, 0x41, 0x22, 0x1C},
        {0x7F, 0x49, 0x49, 0x49, 0x41},
        {0x7F, 0x09, 0x09, 0x09, 0x01},
        {0x3E, 0x41, 0x49, 0x49, 0x7A},
        {0x7F, 0x08, 0x08, 0x08, 0x7F},
        {0x00, 0x41, 0x7F, 0x41, 0x00},
        {0x20, 0x40, 0x41, 0x3F, 0x01},
        {0x7F, 0x08, 0x14, 0x22, 0x41},
        {0x7F, 0x40, 0x40, 0x40, 0x40},
        {0x7F, 0x02, 0x0C, 0x02, 0x7F},
        {0x7F, 0x04, 0x08, 0x10, 0x7F},
        {0x3E, 0x41, 0x41, 0x41, 0x3E},
        {0x7F, 0x09, 0x09, 0x09, 0x06},
        {0x3E, 0x41, 0x51, 0x21, 0x5E},
        {0x7F, 0x09, 0x19, 0x29, 0x46},
        {0x46, 0x49, 0x49, 0x49, 0x31},
        {0x01, 0x01, 0x7F, 0x01, 0x01},
        {0x3F, 0x40, 0x40, 0x40, 0x3F},
        {0x1F, 0x20, 0x40, 0x20, 0x1F},
        {0x7F, 0x20, 0x18, 0x20, 0x7F},
        {0x63, 0x14, 0x08, 0x14, 0x63},
        {0x07, 0x08, 0x70, 0x08, 0x07},
        {0x61, 0x51, 0x49, 0x45, 0x43}
    };
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const uint8_t minus[5] = {0x08, 0x08, 0x08, 0x08, 0x08};

    if ((c >= '0') && (c <= '9')) {
        return digits[c - '0'];
    }
    if ((c >= 'A') && (c <= 'Z')) {
        return letters[c - 'A'];
    }
    if ((c >= 'a') && (c <= 'z')) {
        return letters[c - 'a'];
    }
    if (c == ':') {
        return colon;
    }
    if (c == '.') {
        return dot;
    }
    if (c == '-') {
        return minus;
    }
    return blank;
}

static inline void oled_set_pos(uint8_t page, uint8_t col)
{
    (void)oled_cmd((uint8_t)(0xB0U | (page & 0x07U)));
    (void)oled_cmd((uint8_t)(0x00U | (col & 0x0FU)));
    (void)oled_cmd((uint8_t)(0x10U | (col >> 4U)));
}

static inline void oled_puts(uint8_t page, uint8_t col, const char *text)
{
    oled_set_pos(page, col);
    while (*text != '\0') {
        uint8_t out[6];
        const uint8_t *glyph = oled_font5x7(*text++);
        uint8_t i;

        for (i = 0U; i < 5U; i++) {
            out[i] = glyph[i];
        }
        out[5] = 0x00U;
        (void)oled_data(out, 6U);
    }
}

static inline void oled_puts_fixed(uint8_t page, uint8_t col,
                                   const char *text, uint8_t width)
{
    uint8_t count = 0U;

    oled_set_pos(page, col);
    while ((*text != '\0') && (count < width)) {
        uint8_t out[6];
        const uint8_t *glyph = oled_font5x7(*text++);
        uint8_t i;

        for (i = 0U; i < 5U; i++) {
            out[i] = glyph[i];
        }
        out[5] = 0x00U;
        (void)oled_data(out, 6U);
        count++;
    }
    while (count < width) {
        uint8_t out[6] = {0};

        (void)oled_data(out, 6U);
        count++;
    }
}

static inline void oled_clear(void)
{
    uint8_t zeros[8] = {0};
    uint8_t page;
    uint8_t col;

    for (page = 0U; page < 8U; page++) {
        oled_set_pos(page, 0U);
        for (col = 0U; col < 16U; col++) {
            (void)oled_data(zeros, 8U);
        }
    }
}

static inline int oled_init_at(uint8_t addr)
{
    static const uint8_t cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x2E, 0xAF
    };
    uint8_t i;

    oled_set_addr(addr);
    for (i = 0U; i < (uint8_t)sizeof(cmds); i++) {
        if (oled_cmd(cmds[i]) != 0) {
            return -(int)(i + 1U);
        }
    }
    oled_clear();
    return 0;
}

static inline void oled_force_all_on(bool enabled)
{
    (void)oled_cmd(enabled ? 0xA5U : 0xA4U);
}

static inline int oled_init(void)
{
    int status = oled_init_at(OLED_ADDR_3C);

    if (status == 0) {
        return 0;
    }
    status = oled_init_at(OLED_ADDR_3D);
    if (status == 0) {
        return 1;
    }
    oled_set_addr(OLED_ADDR_3C);
    return status;
}

static inline void oled_i32_to_text(char *dst, int32_t value)
{
    char temp[11];
    uint8_t pos = 0U;
    uint8_t out = 0U;
    uint32_t mag;

    if (value < 0) {
        dst[out++] = '-';
        mag = (uint32_t)(-value);
    } else {
        mag = (uint32_t)value;
    }

    do {
        temp[pos++] = (char)('0' + (mag % 10U));
        mag /= 10U;
    } while ((mag != 0U) && (pos < (uint8_t)sizeof(temp)));

    while (pos > 0U) {
        dst[out++] = temp[--pos];
    }
    dst[out] = '\0';
}

static inline void oled_make_line(char *dst, const char *label, int32_t value)
{
    uint8_t i = 0U;

    while (*label != '\0') {
        dst[i++] = *label++;
    }
    oled_i32_to_text(&dst[i], value);
}

static inline void oled_make_time_text(char *dst, uint32_t run_time_ms)
{
    uint32_t total_tenths = run_time_ms / 100U;
    uint32_t minutes = total_tenths / 600U;
    uint32_t seconds = (total_tenths / 10U) % 60U;
    uint32_t tenths = total_tenths % 10U;

    dst[0] = 'T';
    dst[1] = 'I';
    dst[2] = 'M';
    dst[3] = 'E';
    dst[4] = ':';
    dst[5] = (char)('0' + ((minutes / 10U) % 10U));
    dst[6] = (char)('0' + (minutes % 10U));
    dst[7] = ':';
    dst[8] = (char)('0' + ((seconds / 10U) % 10U));
    dst[9] = (char)('0' + (seconds % 10U));
    dst[10] = '.';
    dst[11] = (char)('0' + tenths);
    dst[12] = 'S';
    dst[13] = '\0';
}

static inline void oled_make_step_angle_text(char *dst, int32_t steps)
{
    uint32_t angle_tenths;
    uint32_t whole;
    uint32_t tenth;

    if (steps < 0) {
        steps = -steps;
    }
    /*
     * TTL 位置控制下 StepperArmState 的兼容字段以 0.01° 为单位，
     * OLED 只显示到 0.1°，因此直接做整数换算，避免继续依赖脉冲细分。
     */
    angle_tenths = ((uint32_t)steps + 5U) / 10U;
    whole = angle_tenths / 10U;
    tenth = angle_tenths % 10U;

    dst[0] = 'A';
    dst[1] = 'N';
    dst[2] = 'G';
    dst[3] = ':';
    oled_i32_to_text(&dst[4], (int32_t)whole);
    while (*dst != '\0') {
        dst++;
    }
    *dst++ = '.';
    *dst++ = (char)('0' + tenth);
    *dst++ = 'D';
    *dst = '\0';
}

static inline void oled_show_line_mode(uint32_t run_time_ms, bool started,
                                       bool running, bool paused,
                                       uint8_t competition_mode,
                                       int32_t ball_position_mm,
                                       bool x42s_ok)
{
    char line[20];

    oled_puts_fixed(0U, 0U, "26H CAR", 21U);
    if (competition_mode == 3U) {
        if (paused) {
            oled_puts_fixed(2U, 0U, "M3 Q6 ANY PAUSE", 21U);
        } else {
            oled_puts_fixed(2U, 0U, started ?
                (running ? "M3 Q6 ANY RUN" : "M3 Q6 ANY DONE") :
                "M3 Q6 ANY READY", 21U);
        }
    } else if (competition_mode == 2U) {
        if (paused) {
            oled_puts_fixed(2U, 0U, "M2 Q4 PAUSE", 21U);
        } else {
            oled_puts_fixed(2U, 0U, started ?
                (running ? "M2 Q4 RUN" : "M2 Q4 DONE") :
                "M2 Q4 READY", 21U);
        }
    } else {
        if (paused) {
            oled_puts_fixed(2U, 0U, "M1 LINE PAUSE", 21U);
        } else {
            oled_puts_fixed(2U, 0U, started ?
                (running ? "M1 LINE RUN" : "M1 LINE DONE") :
                "M1 LINE READY", 21U);
        }
    }
    oled_make_time_text(line, run_time_ms);
    oled_puts_fixed(4U, 0U, line, 21U);
    if (competition_mode >= 2U) {
        oled_make_line(line, x42s_ok ? "BALL:" : "X42S! BALL:",
            ball_position_mm);
        oled_puts_fixed(6U, 0U, line, 21U);
    } else {
        oled_puts_fixed(6U, 0U, "B11 CLR B14 PAUSE", 21U);
    }
}

static inline void oled_show_stepper_mode(const StepperArmState *state,
                                          bool started)
{
    char line[20];

    oled_puts_fixed(0U, 0U, "26H CAR", 21U);
    oled_puts_fixed(2U, 0U, started ? "MODE:STEP ADJ" :
        "MODE:STEP RDY", 21U);
    oled_make_step_angle_text(line, state->current_steps);
    oled_puts_fixed(4U, 0U, line, 21U);
    oled_make_line(line, "STEP:", state->current_steps);
    oled_puts_fixed(6U, 0U, line, 21U);
}

#endif
