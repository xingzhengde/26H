#include "pid.h"

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void pid_init(PidController *pid, float kp, float ki, float kd,
              float target, float max_integral, float max_output)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = target;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->max_integral = max_integral;
    pid->max_output = max_output;
}

void pid_reset(PidController *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

float pid_update(PidController *pid, float measurement)
{
    float error = pid->target - measurement;
    float derivative = error - pid->last_error;

    pid->integral += error;
    pid->integral = clamp_float(pid->integral,
        -pid->max_integral, pid->max_integral);
    pid->last_error = error;

    return clamp_float(pid->kp * error + pid->ki * pid->integral +
                           pid->kd * derivative,
        -pid->max_output, pid->max_output);
}

void pid_set_gains(PidController *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_target(PidController *pid, float target)
{
    pid->target = target;
}
