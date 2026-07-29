#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#define MOTOR_PWM_PERIOD        (1600U)
#define MOTOR_PWM_DEAD_TICKS    (20U)
#define SPEED_CTRL_PERIOD_MS    (20U)
#define UART_REPORT_PERIOD_MS   (100U)
#define PWM_STEP_LIMIT          (30)
#define STARTUP_SAFE_MS         (800U)

#define DEFAULT_TARGET_COUNTS   (120.0f)
#define DEFAULT_KP              (1.20f)
#define DEFAULT_KI              (0.003f)
#define DEFAULT_KD              (0.0f)
#define DEFAULT_MAX_OUTPUT      (390.0f)
#define DEFAULT_MAX_INTEGRAL    (270.0f)

#define DEFAULT_LEFT_PWM_TRIM   (0)
#define DEFAULT_RIGHT_PWM_TRIM  (-14)
#define DEFAULT_BRAKE_MS        (120U)

#define LINE_DEFAULT_KP         (2.45f)
#define LINE_DEFAULT_KD         (1.00f)
#define LINE_DEFAULT_TARGET     (120.0f)
#define LINE_DEFAULT_BASE_PWM   (255)
#define LINE_CORR_LIMIT         (200)
#define LINE_MIN_WHEEL_TARGET   (25.0f)
#define LINE_SEARCH_PWM         (0)
#define LINE_CORRECTION_SIGN    (1)

#define LEFT_MOTOR_INVERT       (0)
#define RIGHT_MOTOR_INVERT      (0)
#define LEFT_ENCODER_INVERT     (0)
#define RIGHT_ENCODER_INVERT    (0)

#endif
