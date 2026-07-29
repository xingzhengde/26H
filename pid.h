#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    float integral;
    float last_error;
    float max_integral;
    float max_output;
} PidController;

void pid_init(PidController *pid, float kp, float ki, float kd,
              float target, float max_integral, float max_output);
void pid_reset(PidController *pid);
float pid_update(PidController *pid, float measurement);
void pid_set_gains(PidController *pid, float kp, float ki, float kd);
void pid_set_target(PidController *pid, float target);

#endif
