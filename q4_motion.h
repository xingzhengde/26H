#ifndef Q4_MOTION_H
#define Q4_MOTION_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 第四问在当前控制周期使用的底盘与摆杆前馈设定值。
 *
 * ramp_ratio 是从 0 到 1 的线性时间比例。底盘直接使用 base_pwm，
 * speed_target_counts 仅用于状态显示和加速度量纲换算，不参与速度 PID。
 */
typedef struct {
    float speed_target_counts;
    int16_t base_pwm;
    float ramp_ratio;
    float forward_accel_mps2;
    float pipe_feedforward_deg;
    uint32_t elapsed_ms;
    bool passed_b_time;
} Q4MotionSetpoint;

void q4_motion_init(void);
void q4_motion_start(uint32_t now_ms);
void q4_motion_stop(void);
bool q4_motion_is_active(void);
float q4_motion_get_start_feedforward_deg(void);
void q4_motion_get_setpoint(uint32_t now_ms, int32_t measured_speed_cps,
                            int32_t measured_accel_cps2,
                            Q4MotionSetpoint *setpoint);

#endif
