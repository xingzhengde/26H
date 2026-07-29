#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(void);
void motor_set_left(int16_t pwm);
void motor_set_right(int16_t pwm);
void motor_brake(void);
void motor_stop(void);

#endif
