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
} StepperArmState;

void stepper_arm_init(void);
void stepper_arm_handle_irq(void);
void stepper_arm_mark_neutral(void);
void stepper_arm_mark_high_limit(void);
void stepper_arm_mark_low_limit(void);
void stepper_arm_stop(void);
void stepper_arm_jog_steps(int32_t delta_steps, float speed_dps);
void stepper_arm_set_pipe_angle(float pipe_angle_deg, float speed_dps);
StepperArmState stepper_arm_get_state(void);

#endif
