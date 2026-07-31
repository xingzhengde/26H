#include "stepper_arm.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

#define X42S_FRAME_END              0x6BU
#define X42S_CMD_ZERO_POSITION      0x0AU
#define X42S_CMD_ENABLE             0xF3U
#define X42S_CMD_DIRECT_POSITION_X  0xFBU
#define X42S_CMD_STOP               0xFEU
#define X42S_CMD_READ_POSITION      0x36U
#define X42S_CMD_READ_STATUS        0x3AU
#define X42S_CMD_READ_VERSION       0x1FU
#define X42S_RX_BUFFER_SIZE         64U
#define X42S_STARTUP_COMMAND_GAP_MS 50U

typedef enum {
    X42S_STARTUP_WAIT = 0,
    X42S_STARTUP_ENABLE_SENT,
    X42S_STARTUP_READY
} X42SStartupState;

static volatile uint8_t g_rx_buffer[X42S_RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint32_t g_rx_irq_ms;

static uint8_t g_parse_frame[8];
static uint8_t g_parse_index;
static uint8_t g_parse_expected;
static X42SStartupState g_startup_state;
static uint32_t g_startup_ms;
static uint32_t g_startup_command_ms;
static uint32_t g_last_command_ms;
static uint32_t g_last_position_poll_ms;
static uint32_t g_last_status_poll_ms;
static uint32_t g_last_valid_rx_ms;
static float g_actual_motor_deg;
static float g_target_motor_deg;
static float g_pending_motor_deg;
static float g_pending_speed_dps;
static float g_min_motor_deg;
static float g_max_motor_deg;
static uint8_t g_status_flags;
static uint8_t g_last_error_code;
static uint16_t g_firmware_version;
static uint8_t g_hardware_type;
static uint8_t g_hardware_version;
static uint32_t g_service_ms;
static uint32_t g_last_version_request_ms;
static bool g_version_requested;
static bool g_target_pending;
static bool g_homed;

static float clamp_f32_local(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float abs_f32_local(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 发送一帧 X42S TTL 命令。
 *
 * UART0 仅服务一台 X42S，最长运动帧 12 字节，在 115200 baud 下约 1.1ms。
 * 发送频率被限制为 20ms 一次，不会占用 K230 的 UART2，也不会在中断中阻塞。
 */
static void x42s_send_bytes(const uint8_t *data, uint8_t length)
{
    uint8_t index;

    for (index = 0U; index < length; index++) {
        DL_UART_Main_transmitDataBlocking(UART_X42S_INST, data[index]);
    }
}

static void x42s_send_enable(void)
{
    const uint8_t command[6] = {
        X42S_ADDRESS, X42S_CMD_ENABLE, 0xABU, 0x01U, 0x00U,
        X42S_FRAME_END
    };

    x42s_send_bytes(command, (uint8_t)sizeof(command));
}

static void x42s_send_zero_position(void)
{
    const uint8_t command[4] = {
        X42S_ADDRESS, X42S_CMD_ZERO_POSITION, 0x6DU, X42S_FRAME_END
    };

    x42s_send_bytes(command, (uint8_t)sizeof(command));
}

static void x42s_send_stop(void)
{
    const uint8_t command[5] = {
        X42S_ADDRESS, X42S_CMD_STOP, 0x98U, 0x00U, X42S_FRAME_END
    };

    x42s_send_bytes(command, (uint8_t)sizeof(command));
}

static void x42s_send_read(uint8_t function_code)
{
    const uint8_t command[3] = {
        X42S_ADDRESS, function_code, X42S_FRAME_END
    };

    x42s_send_bytes(command, (uint8_t)sizeof(command));
}

/**
 * @brief 下发 X 固件直通限速绝对位置命令。
 *
 * S_PosTDP=Enable 时位置单位为 0.01°。使用绝对坐标避免丢包或复位后累计
 * 相对位移误差；X42S 支持新位置命令实时打断旧命令并平滑过渡。
 */
static void x42s_send_absolute_position(float motor_deg, float speed_dps)
{
    uint8_t command[12];
    uint32_t position_units;
    uint16_t speed_tenth_rpm;
    float magnitude_deg = abs_f32_local(motor_deg);
    float speed_value = abs_f32_local(speed_dps) * (10.0f / 6.0f);

    speed_value = clamp_f32_local(speed_value, 1.0f, 30000.0f);
    speed_tenth_rpm = (uint16_t)(speed_value + 0.5f);
    position_units = (uint32_t)(
        magnitude_deg * X42S_POSITION_UNITS_PER_DEG + 0.5f);

    command[0] = X42S_ADDRESS;
    command[1] = X42S_CMD_DIRECT_POSITION_X;
    command[2] = (motor_deg < 0.0f) ? 0x01U : 0x00U;
    command[3] = (uint8_t)(speed_tenth_rpm >> 8U);
    command[4] = (uint8_t)speed_tenth_rpm;
    command[5] = (uint8_t)(position_units >> 24U);
    command[6] = (uint8_t)(position_units >> 16U);
    command[7] = (uint8_t)(position_units >> 8U);
    command[8] = (uint8_t)position_units;
    command[9] = 0x01U;  /* 相对清零坐标的绝对位置。 */
    command[10] = 0x00U; /* 立即执行，不等待多机同步。 */
    command[11] = X42S_FRAME_END;
    x42s_send_bytes(command, (uint8_t)sizeof(command));
}

static bool x42s_rx_pop(uint8_t *byte)
{
    if (g_rx_tail == g_rx_head) {
        return false;
    }
    *byte = g_rx_buffer[g_rx_tail];
    g_rx_tail = (uint16_t)((g_rx_tail + 1U) % X42S_RX_BUFFER_SIZE);
    return true;
}

static uint8_t x42s_expected_length(uint8_t function_code)
{
    if (function_code == X42S_CMD_READ_POSITION) {
        return 8U;
    }
    if (function_code == X42S_CMD_READ_STATUS) {
        return 4U;
    }
    if (function_code == X42S_CMD_READ_VERSION) {
        return 7U;
    }
    if ((function_code == X42S_CMD_ZERO_POSITION) ||
        (function_code == X42S_CMD_ENABLE) ||
        (function_code == X42S_CMD_DIRECT_POSITION_X) ||
        (function_code == X42S_CMD_STOP)) {
        return 4U;
    }
    return 0U;
}

static void x42s_handle_frame(uint32_t now_ms)
{
    uint8_t function_code = g_parse_frame[1];

    g_last_valid_rx_ms = now_ms;
    if (function_code == X42S_CMD_READ_POSITION) {
        uint32_t raw_position =
            ((uint32_t)g_parse_frame[3] << 24U) |
            ((uint32_t)g_parse_frame[4] << 16U) |
            ((uint32_t)g_parse_frame[5] << 8U) |
            (uint32_t)g_parse_frame[6];
        float position_deg = (float)raw_position / 10.0f;

        g_actual_motor_deg =
            (g_parse_frame[2] == 0x01U) ? -position_deg : position_deg;
    } else if (function_code == X42S_CMD_READ_STATUS) {
        g_status_flags = g_parse_frame[2];
    } else if (function_code == X42S_CMD_READ_VERSION) {
        g_firmware_version =
            ((uint16_t)g_parse_frame[2] << 8U) |
            (uint16_t)g_parse_frame[3];
        g_hardware_type = g_parse_frame[4];
        g_hardware_version = g_parse_frame[5];
    } else {
        uint8_t response = g_parse_frame[2];

        if (response == 0x02U) {
            g_last_error_code = 0U;
        } else {
            g_last_error_code = response;
        }
    }
}

/**
 * @brief 在主循环解析定长 X42S 回包。
 *
 * 回包格式由功能码确定，ISR 只搬运字节，避免串口接收影响底盘和滚球控制。
 */
static void x42s_parse_rx(uint32_t now_ms)
{
    uint8_t byte;

    while (x42s_rx_pop(&byte)) {
        if (g_parse_index == 0U) {
            if (byte == X42S_ADDRESS) {
                g_parse_frame[0] = byte;
                g_parse_index = 1U;
            }
            continue;
        }
        if (g_parse_index == 1U) {
            g_parse_frame[1] = byte;
            g_parse_expected = x42s_expected_length(byte);
            if (g_parse_expected == 0U) {
                g_parse_index = 0U;
            } else {
                g_parse_index = 2U;
            }
            continue;
        }

        g_parse_frame[g_parse_index++] = byte;
        if (g_parse_index >= g_parse_expected) {
            if (g_parse_frame[g_parse_expected - 1U] == X42S_FRAME_END) {
                x42s_handle_frame(now_ms);
            }
            g_parse_index = 0U;
            g_parse_expected = 0U;
        }
    }
}

void stepper_arm_init(void)
{
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_parse_index = 0U;
    g_parse_expected = 0U;
    g_startup_state = X42S_STARTUP_WAIT;
    g_startup_ms = 0U;
    g_startup_command_ms = 0U;
    g_last_command_ms = 0U;
    g_last_position_poll_ms = 0U;
    g_last_status_poll_ms = 0U;
    g_last_valid_rx_ms = 0U;
    g_actual_motor_deg = 0.0f;
    g_target_motor_deg = 0.0f;
    g_pending_motor_deg = 0.0f;
    g_pending_speed_dps = STEPPER_DEFAULT_DPS;
    g_min_motor_deg = -X42S_MAX_MOTOR_ANGLE_DEG;
    g_max_motor_deg = X42S_MAX_MOTOR_ANGLE_DEG;
    g_status_flags = 0U;
    g_last_error_code = 0U;
    g_firmware_version = 0U;
    g_hardware_type = 0U;
    g_hardware_version = 0U;
    g_service_ms = 0U;
    g_last_version_request_ms = 0U;
    g_version_requested = false;
    g_target_pending = false;
    g_homed = false;
    NVIC_EnableIRQ(UART_X42S_INST_INT_IRQN);
}

void stepper_arm_handle_uart_irq(uint32_t now_ms)
{
    g_rx_irq_ms = now_ms;
    switch (DL_UART_Main_getPendingInterrupt(UART_X42S_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_X42S_INST)) {
            uint16_t next =
                (uint16_t)((g_rx_head + 1U) % X42S_RX_BUFFER_SIZE);
            uint8_t byte = DL_UART_Main_receiveData(UART_X42S_INST);

            if (next != g_rx_tail) {
                g_rx_buffer[g_rx_head] = byte;
                g_rx_head = next;
            }
        }
        break;
    default:
        break;
    }
}

void stepper_arm_service(uint32_t now_ms)
{
    g_service_ms = now_ms;
    x42s_parse_rx(now_ms);

    if (g_startup_ms == 0U) {
        g_startup_ms = now_ms;
    }
    if (g_startup_state == X42S_STARTUP_WAIT) {
        if ((now_ms - g_startup_ms) >= X42S_STARTUP_DELAY_MS) {
            x42s_send_enable();
            g_startup_command_ms = now_ms;
            g_startup_state = X42S_STARTUP_ENABLE_SENT;
        }
        return;
    }
    if (g_startup_state == X42S_STARTUP_ENABLE_SENT) {
        if ((now_ms - g_startup_command_ms) >=
            X42S_STARTUP_COMMAND_GAP_MS) {
            /*
             * 上电时水管必须人工放平；此命令把该机械位置定义为绝对零点。
             */
            x42s_send_zero_position();
            g_actual_motor_deg = 0.0f;
            g_target_motor_deg = 0.0f;
            g_homed = true;
            g_startup_state = X42S_STARTUP_READY;
            g_last_command_ms = now_ms;
        }
        return;
    }

    if (!g_version_requested ||
        ((g_firmware_version == 0U) &&
         ((now_ms - g_last_version_request_ms) >= 1000U))) {
        x42s_send_read(X42S_CMD_READ_VERSION);
        g_version_requested = true;
        g_last_version_request_ms = now_ms;
        return;
    }
    if (g_target_pending &&
        ((now_ms - g_last_command_ms) >= X42S_COMMAND_PERIOD_MS)) {
        x42s_send_absolute_position(g_pending_motor_deg,
            g_pending_speed_dps);
        g_target_motor_deg = g_pending_motor_deg;
        g_target_pending = false;
        g_last_command_ms = now_ms;
        return;
    }
    if ((now_ms - g_last_position_poll_ms) >= X42S_POSITION_POLL_MS) {
        x42s_send_read(X42S_CMD_READ_POSITION);
        g_last_position_poll_ms = now_ms;
        return;
    }
    if ((now_ms - g_last_status_poll_ms) >= X42S_STATUS_POLL_MS) {
        x42s_send_read(X42S_CMD_READ_STATUS);
        g_last_status_poll_ms = now_ms;
    }
}

void stepper_arm_mark_neutral(void)
{
    x42s_send_zero_position();
    g_actual_motor_deg = 0.0f;
    g_target_motor_deg = 0.0f;
    g_pending_motor_deg = 0.0f;
    g_target_pending = false;
    g_homed = true;
}

void stepper_arm_mark_high_limit(void)
{
    g_min_motor_deg = g_actual_motor_deg;
}

void stepper_arm_mark_low_limit(void)
{
    g_max_motor_deg = g_actual_motor_deg;
}

void stepper_arm_stop(void)
{
    if (g_startup_state == X42S_STARTUP_READY) {
        x42s_send_stop();
    }
    g_pending_motor_deg = g_target_motor_deg;
    g_target_pending = false;
}

void stepper_arm_jog_steps(int32_t delta_steps, float speed_dps)
{
    float target = g_target_motor_deg + ((float)delta_steps * 0.1f);

    target = clamp_f32_local(target, g_min_motor_deg, g_max_motor_deg);
    g_pending_motor_deg = target;
    g_pending_speed_dps = speed_dps;
    g_target_pending = true;
}

void stepper_arm_set_pipe_angle(float pipe_angle_deg, float speed_dps)
{
    float motor_deg = pipe_angle_deg * X42S_MOTOR_DEG_PER_PIPE_DEG;

    motor_deg = clamp_f32_local(motor_deg,
        g_min_motor_deg, g_max_motor_deg);
    if (abs_f32_local(motor_deg - g_pending_motor_deg) >=
        (1.0f / X42S_POSITION_UNITS_PER_DEG)) {
        g_pending_motor_deg = motor_deg;
        g_pending_speed_dps = clamp_f32_local(speed_dps,
            1.0f, STEPPER_MAX_DPS);
        g_target_pending = true;
    }
}

bool stepper_arm_send_pending_now(uint32_t now_ms)
{
    if (g_startup_state != X42S_STARTUP_READY) {
        return false;
    }
    if (g_target_pending) {
        /*
         * 模式二起步专用：绕过常规 20ms 调度，确保倾角命令完整离开
         * UART 后才允许小车启动。12 字节帧在 115200 baud 下约 1ms。
         */
        x42s_send_absolute_position(g_pending_motor_deg,
            g_pending_speed_dps);
        while (DL_UART_Main_isBusy(UART_X42S_INST)) {
        }
        g_target_motor_deg = g_pending_motor_deg;
        g_target_pending = false;
        g_last_command_ms = now_ms;
    }
    return true;
}

StepperArmState stepper_arm_get_state(void)
{
    StepperArmState state;

    state.current_steps = (int32_t)(g_actual_motor_deg * 100.0f);
    state.target_steps = (int32_t)(g_target_motor_deg * 100.0f);
    state.min_steps = (int32_t)(g_min_motor_deg * 100.0f);
    state.max_steps = (int32_t)(g_max_motor_deg * 100.0f);
    state.neutral_steps = 0;
    state.busy = abs_f32_local(
        g_target_motor_deg - g_actual_motor_deg) >
        X42S_POSITION_TOLERANCE_DEG;
    state.homed = g_homed;
    state.actual_motor_deg = g_actual_motor_deg;
    state.target_motor_deg = g_target_motor_deg;
    state.status_flags = g_status_flags;
    state.last_error_code = g_last_error_code;
    state.firmware_version = g_firmware_version;
    state.hardware_type = g_hardware_type;
    state.hardware_version = g_hardware_version;
    state.communication_ok = (g_last_valid_rx_ms != 0U) &&
        ((g_service_ms - g_last_valid_rx_ms) <= X42S_RX_TIMEOUT_MS);
    state.stall_fault = (g_status_flags & 0x0CU) != 0U;
    return state;
}
