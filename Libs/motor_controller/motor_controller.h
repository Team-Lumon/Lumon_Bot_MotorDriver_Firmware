#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "main.h"
#include <stdint.h>

typedef enum
{
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_FOLLOWING_ERROR = 1,
    MOTOR_FAULT_ENCODER_JUMP = 2,
    MOTOR_FAULT_COMMAND_LIMIT = 3,
    MOTOR_FAULT_ENCODER_NOT_READY = 4,
    MOTOR_FAULT_STALL_GUARD = 5
} MotorFault_t;

typedef enum
{
    MOTOR_CONTROL_POSITION = 0,
    MOTOR_CONTROL_SPEED = 1
} MotorControlMode_t;

typedef enum
{
    MOTOR_PROFILE_WAIT_ENCODER = 0,
    MOTOR_PROFILE_MOVING = 1,
    MOTOR_PROFILE_HOLDING = 2
} MotorProfileState_t;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float speed_Kp;
    float speed_Ki;
    float speed_Kd;
    float max_integral;
    float max_speed_integral;
    float max_velocity_steps_s;
    float max_accel_steps_s2;
    float steps_per_encoder_count;
    float following_error_limit_counts;
    float feedforward_gain;
    float feedback_gain;
    float target_delta_counts;
    float profile_max_velocity_counts_s;
    float profile_accel_counts_s2;
    uint32_t monitor_interval_ms;
} MotorControllerConfig_t;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float speed_Kp;
    float speed_Ki;
    float speed_Kd;

    float dt;

    float integral;
    float previous_error;
    float speed_integral;
    float previous_speed_error;

    float max_integral;
    float max_speed_integral;
    float max_velocity_steps_s;
    float max_accel_steps_s2;

    float steps_per_encoder_count;
    float feedforward_gain;
    float feedback_gain;

    float following_error_limit_counts;

    int32_t target_position_counts;
    float target_velocity_counts_s;
    float target_speed_counts_s;

    int32_t actual_position_counts;
    float actual_speed_counts_s;
    int32_t previous_position_counts;
    uint16_t last_abs_raw;
    uint8_t encoder_initialized;
    uint8_t speed_initialized;

    float previous_velocity_cmd_steps_s;
    float last_p_term_counts_s;
    float last_i_term_counts_s;
    float last_d_term_counts_s;
    float last_correction_counts_s;
    float last_velocity_cmd_counts_s;

    MotorControlMode_t mode;
    uint8_t enabled;
    MotorFault_t fault;

    float profile_target_counts;
    float profile_final_counts;
    float profile_velocity_counts_s;
    float profile_target_delta_counts;
    float profile_max_velocity_counts_s;
    float profile_accel_counts_s2;
    MotorProfileState_t profile_state;

    uint32_t monitor_interval_ms;
    uint32_t monitor_timer_ms;
    float monitor_previous_speed_counts_s;
    float monitor_speed_counts_s;
    float monitor_accel_counts_s2;
    int32_t monitor_actual_counts;
    int32_t monitor_target_counts;
    float monitor_target_velocity_counts_s;
    float monitor_error_counts;
    float monitor_p_counts_s;
    float monitor_i_counts_s;
    float monitor_d_counts_s;
    float monitor_correction_counts_s;
    float monitor_command_counts_s;
    MotorProfileState_t monitor_state;
    volatile uint8_t monitor_pending;
} MotorController_t;

void MotorController_Init(MotorController_t *motor);
void MotorController_ApplyConfig(MotorController_t *motor,
                                 const MotorControllerConfig_t *config);
void MotorController_Enable(MotorController_t *motor);
void MotorController_Disable(MotorController_t *motor);
void MotorController_SetFault(MotorController_t *motor, MotorFault_t fault);

void MotorController_SetTarget(MotorController_t *motor,
                               int32_t target_position_counts,
                               float target_velocity_counts_s);

void MotorController_SetRelativeTarget(MotorController_t *motor,
                                       int32_t delta_counts,
                                       float target_velocity_counts_s);

void MotorController_SetTargetSpeed(MotorController_t *motor,
                                    float target_speed_counts_s);

void MotorController_HoldCurrentPosition(MotorController_t *motor);

void MotorController_Update_1kHz(MotorController_t *motor);
void MotorController_Service_1kHz(MotorController_t *motor);
void MotorController_StartRelativeProfile(MotorController_t *motor,
                                          float delta_counts);
uint8_t MotorController_MonitorPending(MotorController_t *motor);
void MotorController_PrintMonitor(MotorController_t *motor);

int32_t MotorController_GetActualPosition(MotorController_t *motor);
float MotorController_GetPositionError(MotorController_t *motor);
float MotorController_GetActualSpeed(MotorController_t *motor);
float MotorController_GetTargetSpeed(MotorController_t *motor);

MotorFault_t MotorController_GetFault(MotorController_t *motor);
void MotorController_ClearFault(MotorController_t *motor);
const char *MotorController_FaultName(MotorFault_t fault);

#endif
