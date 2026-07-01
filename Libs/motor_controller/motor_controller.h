#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "main.h"
#include <stdint.h>

typedef enum
{
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_FOLLOWING_ERROR = 1,
    MOTOR_FAULT_ENCODER_JUMP = 2,
    MOTOR_FAULT_COMMAND_LIMIT = 3
} MotorFault_t;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float dt;

    float integral;
    float previous_error;

    float max_integral;
    float max_velocity_steps_s;
    float max_accel_steps_s2;

    float steps_per_encoder_count;

    float following_error_limit_counts;

    int32_t target_position_counts;
    float target_velocity_counts_s;

    int32_t actual_position_counts;
    uint16_t last_abs_raw;
    uint8_t encoder_initialized;

    float previous_velocity_cmd_steps_s;

    uint8_t enabled;
    MotorFault_t fault;
} MotorController_t;

void MotorController_Init(MotorController_t *motor);
void MotorController_Enable(MotorController_t *motor);
void MotorController_Disable(MotorController_t *motor);

void MotorController_SetTarget(MotorController_t *motor,
                               int32_t target_position_counts,
                               float target_velocity_counts_s);

void MotorController_SetRelativeTarget(MotorController_t *motor,
                                       int32_t delta_counts,
                                       float target_velocity_counts_s);

void MotorController_HoldCurrentPosition(MotorController_t *motor);

void MotorController_Update_1kHz(MotorController_t *motor);

int32_t MotorController_GetActualPosition(MotorController_t *motor);
float MotorController_GetPositionError(MotorController_t *motor);

MotorFault_t MotorController_GetFault(MotorController_t *motor);
void MotorController_ClearFault(MotorController_t *motor);

#endif