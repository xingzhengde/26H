#ifndef STEPPER_ARM_H
#define STEPPER_ARM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t current_steps;
    int32_t target_steps;
    int32_t min_steps;
    int32_t max_steps;
    int32_t neutral_steps;
    bool busy;
    bool homed;
    float actual_motor_deg;
    float target_motor_deg;
    uint8_t status_flags;
    uint8_t last_error_code;
    uint16_t firmware_version;
    uint8_t hardware_type;
    uint8_t hardware_version;
    bool communication_ok;
    bool stall_fault;
} StepperArmState;

void stepper_arm_init(void);
void stepper_arm_handle_uart_irq(uint32_t now_ms);
void stepper_arm_service(uint32_t now_ms);
void stepper_arm_mark_neutral(void);
void stepper_arm_mark_high_limit(void);
void stepper_arm_mark_low_limit(void);
void stepper_arm_stop(void);
void stepper_arm_jog_steps(int32_t delta_steps, float speed_dps);
void stepper_arm_set_pipe_angle(float pipe_angle_deg, float speed_dps);
bool stepper_arm_send_pending_now(uint32_t now_ms);
StepperArmState stepper_arm_get_state(void);

#endif
