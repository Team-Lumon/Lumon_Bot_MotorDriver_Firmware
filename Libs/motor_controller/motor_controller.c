#include "motor_controller.h"
#include "stepper_oc.h"
#include <math.h>
#include <stdio.h>

#include "as5600.h"

extern AS5600_t encoder;

/*
 * Optional fault callback.
 * You can implement this function somewhere else later to send CAN error.
 *
 * Example in another file:
 *
 * void MotorController_OnFault(MotorFault_t fault)
 * {
 *     CAN_SendMotorError(fault);
 * }
 */
__weak void MotorController_OnFault(MotorFault_t fault)
{
    (void)fault;
}

/*
 * Small utility
 */
static float clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t round_float_to_int(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                             (int32_t)(value - 0.5f);
}

static void MotorController_RequestEncoderSample(void)
{
    if (!AS5600_Busy(&encoder))
    {
        (void)AS5600_Read(&encoder);
    }
}

static uint8_t MotorController_ReadEncoder(MotorController_t *motor,
                                           int32_t *position_counts)
{
    if (!AS5600_Ready(&encoder))
    {
        MotorController_RequestEncoderSample();
        return 0U;
    }

    motor->actual_position_counts = AS5600_Abs(&encoder);
    motor->last_abs_raw = AS5600_Raw(&encoder);
    motor->encoder_initialized = 1U;
    *position_counts = motor->actual_position_counts;
    return 1U;
}

void MotorController_Init(MotorController_t *motor)
{
    /*
     * TODO:
     * Tune these later. Position mode starts with Kp only.
     */
    motor->Kp = 10.0f;
    motor->Ki = 0.0f;
    motor->Kd = 0.0f;

    /*
     * Speed loop output is a correction in encoder counts/s.
     */
    motor->speed_Kp = 10.0f;
    motor->speed_Ki = 0.0f;
    motor->speed_Kd = 0.0f;

    /*
     * 1 kHz controller update.
     */
    motor->dt = 0.001f;

    motor->integral = 0.0f;
    motor->previous_error = 0.0f;
    motor->speed_integral = 0.0f;
    motor->previous_speed_error = 0.0f;

    /*
     * TODO:
     * Tune these based on your motor, driver, pulley, and cable load.
     */
    motor->max_integral = 10000.0f;
    motor->max_speed_integral = 10000.0f;
    motor->max_velocity_steps_s = 10000.0f;
    motor->max_accel_steps_s2 = 100000.0f;

    motor->steps_per_encoder_count = 3200.0/4096.0f; // 3200 steps per rev, 4096 counts per rev
    motor->feedforward_gain = 0.05f;
    motor->feedback_gain = 0.95f;

    /*
     * TODO:
     * Edit this based on acceptable cable position error.
     */
    motor->following_error_limit_counts = 1000.0f;

    motor->target_position_counts = 0;
    motor->target_velocity_counts_s = 0.0f;
    motor->target_speed_counts_s = 0.0f;

    motor->actual_position_counts = 0;
    motor->actual_speed_counts_s = 0.0f;
    motor->previous_position_counts = 0;
    motor->last_abs_raw = 0;
    motor->encoder_initialized = 0;
    motor->speed_initialized = 0;

    motor->previous_velocity_cmd_steps_s = 0.0f;
    motor->last_p_term_counts_s = 0.0f;
    motor->last_i_term_counts_s = 0.0f;
    motor->last_d_term_counts_s = 0.0f;
    motor->last_correction_counts_s = 0.0f;
    motor->last_velocity_cmd_counts_s = 0.0f;

    motor->mode = MOTOR_CONTROL_SPEED;
    motor->enabled = 0;
    motor->fault = MOTOR_FAULT_NONE;

    motor->profile_target_counts = 0.0f;
    motor->profile_final_counts = 0.0f;
    motor->profile_velocity_counts_s = 0.0f;
    motor->profile_target_delta_counts = 2048.0f;
    motor->profile_max_velocity_counts_s = 7500.0f;
    motor->profile_accel_counts_s2 = 10000.0f;
    /* Stay idle until a target/profile is explicitly requested. */
    motor->profile_state = MOTOR_PROFILE_HOLDING;

    motor->monitor_interval_ms = 50U;
    motor->monitor_timer_ms = 0U;
    motor->monitor_previous_speed_counts_s = 0.0f;
    motor->monitor_speed_counts_s = 0.0f;
    motor->monitor_accel_counts_s2 = 0.0f;
    motor->monitor_actual_counts = 0;
    motor->monitor_target_counts = 0;
    motor->monitor_target_velocity_counts_s = 0.0f;
    motor->monitor_error_counts = 0.0f;
    motor->monitor_p_counts_s = 0.0f;
    motor->monitor_i_counts_s = 0.0f;
    motor->monitor_d_counts_s = 0.0f;
    motor->monitor_correction_counts_s = 0.0f;
    motor->monitor_command_counts_s = 0.0f;
    motor->monitor_state = MOTOR_PROFILE_WAIT_ENCODER;
    motor->monitor_pending = 0U;

    int32_t current_pos = 0;
    if (MotorController_ReadEncoder(motor, &current_pos))
    {
        motor->target_position_counts = current_pos;
        motor->previous_position_counts = current_pos;
    }
}

void MotorController_ApplyConfig(MotorController_t *motor,
                                 const MotorControllerConfig_t *config)
{
    if ((motor == NULL) || (config == NULL))
    {
        return;
    }

    motor->Kp = config->Kp;
    motor->Ki = config->Ki;
    motor->Kd = config->Kd;
    motor->speed_Kp = config->speed_Kp;
    motor->speed_Ki = config->speed_Ki;
    motor->speed_Kd = config->speed_Kd;
    motor->max_integral = config->max_integral;
    motor->max_speed_integral = config->max_speed_integral;
    motor->max_velocity_steps_s = config->max_velocity_steps_s;
    motor->max_accel_steps_s2 = config->max_accel_steps_s2;
    motor->steps_per_encoder_count = config->steps_per_encoder_count;
    motor->following_error_limit_counts =
        config->following_error_limit_counts;
    motor->feedforward_gain = config->feedforward_gain;
    motor->feedback_gain = config->feedback_gain;
    motor->profile_target_delta_counts = config->target_delta_counts;
    motor->profile_max_velocity_counts_s =
        config->profile_max_velocity_counts_s;
    motor->profile_accel_counts_s2 = config->profile_accel_counts_s2;
    motor->monitor_interval_ms = config->monitor_interval_ms;
}

void MotorController_Enable(MotorController_t *motor)
{
    motor->integral = 0.0f;
    motor->previous_error = 0.0f;
    motor->speed_integral = 0.0f;
    motor->previous_speed_error = 0.0f;
    motor->previous_velocity_cmd_steps_s = 0.0f;
    motor->last_p_term_counts_s = 0.0f;
    motor->last_i_term_counts_s = 0.0f;
    motor->last_d_term_counts_s = 0.0f;
    motor->last_correction_counts_s = 0.0f;
    motor->last_velocity_cmd_counts_s = 0.0f;
    motor->actual_speed_counts_s = 0.0f;
    motor->speed_initialized = 0U;

    motor->fault = MOTOR_FAULT_NONE;

    /*
     * Start from current position to avoid sudden jump.
     */
    int32_t current_pos = 0;
    if (MotorController_ReadEncoder(motor, &current_pos))
    {
        motor->target_position_counts = current_pos;
        motor->previous_position_counts = current_pos;
    }
    motor->target_velocity_counts_s = 0.0f;
    motor->target_speed_counts_s = 0.0f;

    Stepper_Enable();

    motor->enabled = 1;
}

void MotorController_Disable(MotorController_t *motor)
{
    motor->enabled = 0;
    Stepper_Stop();
}

void MotorController_SetFault(MotorController_t *motor, MotorFault_t fault)
{
    if ((motor == NULL) || (fault == MOTOR_FAULT_NONE))
    {
        return;
    }

    motor->fault = fault;
    motor->enabled = 0;
    Stepper_Stop();
    MotorController_OnFault(fault);
}

void MotorController_SetTarget(MotorController_t *motor,
                               int32_t target_position_counts,
                               float target_velocity_counts_s)
{
    motor->mode = MOTOR_CONTROL_POSITION;
    motor->target_position_counts = target_position_counts;
    motor->target_velocity_counts_s = target_velocity_counts_s;
}

void MotorController_SetRelativeTarget(MotorController_t *motor,
                                       int32_t delta_counts,
                                       float target_velocity_counts_s)
{
    int32_t current_position_counts = 0;

    if (!MotorController_ReadEncoder(motor, &current_position_counts))
    {
        Stepper_Stop();
        return;
    }

    motor->mode = MOTOR_CONTROL_POSITION;
    motor->target_position_counts = current_position_counts + delta_counts;
    motor->target_velocity_counts_s = target_velocity_counts_s;
}

void MotorController_SetTargetSpeed(MotorController_t *motor,
                                    float target_speed_counts_s)
{
    target_speed_counts_s = clamp_float(target_speed_counts_s,
                                        -motor->max_velocity_steps_s / motor->steps_per_encoder_count,
                                         motor->max_velocity_steps_s / motor->steps_per_encoder_count);

    motor->mode = MOTOR_CONTROL_SPEED;
    motor->target_speed_counts_s = target_speed_counts_s;
}

void MotorController_HoldCurrentPosition(MotorController_t *motor)
{
    int32_t current_pos = 0;
    if (!MotorController_ReadEncoder(motor, &current_pos))
    {
        Stepper_Stop();
        return;
    }

    motor->mode = MOTOR_CONTROL_POSITION;
    motor->target_position_counts = current_pos;
    motor->target_velocity_counts_s = 0.0f;
    motor->target_speed_counts_s = 0.0f;
    motor->integral = 0.0f;
    motor->previous_error = 0.0f;
    motor->speed_integral = 0.0f;
    motor->previous_speed_error = 0.0f;
    motor->previous_velocity_cmd_steps_s = 0.0f;
    motor->last_p_term_counts_s = 0.0f;
    motor->last_i_term_counts_s = 0.0f;
    motor->last_d_term_counts_s = 0.0f;
    motor->last_correction_counts_s = 0.0f;
    motor->last_velocity_cmd_counts_s = 0.0f;

    Stepper_Stop();
}

static void MotorController_ApplyVelocityCounts(MotorController_t *motor,
                                                float velocity_cmd_counts_s)
{
    float velocity_cmd_steps_s =
        velocity_cmd_counts_s * motor->steps_per_encoder_count;

    velocity_cmd_steps_s =
        clamp_float(velocity_cmd_steps_s,
                    -motor->max_velocity_steps_s,
                     motor->max_velocity_steps_s);

    float max_delta_v =
        motor->max_accel_steps_s2 * motor->dt;

    float delta_v =
        velocity_cmd_steps_s - motor->previous_velocity_cmd_steps_s;

    delta_v = clamp_float(delta_v, -max_delta_v, max_delta_v);

    velocity_cmd_steps_s =
        motor->previous_velocity_cmd_steps_s + delta_v;

    motor->previous_velocity_cmd_steps_s = velocity_cmd_steps_s;

    Stepper_SetVelocityStepsPerSec(velocity_cmd_steps_s);
}

/*
 * Call this exactly every 1 ms.
 */
void MotorController_Update_1kHz(MotorController_t *motor)
{
    if (!motor->enabled || motor->fault != MOTOR_FAULT_NONE)
    {
        Stepper_Stop();
        return;
    }

    int32_t actual_pos = 0;
    if (!MotorController_ReadEncoder(motor, &actual_pos))
    {
        Stepper_Stop();
        return;
    }

    if (!motor->speed_initialized)
    {
        motor->previous_position_counts = actual_pos;
        motor->actual_speed_counts_s = 0.0f;
        motor->speed_initialized = 1U;
        MotorController_RequestEncoderSample();
        return;
    }

    int32_t delta_counts = actual_pos - motor->previous_position_counts;
    motor->previous_position_counts = actual_pos;
    motor->actual_speed_counts_s = (float)delta_counts / motor->dt;

    if (motor->mode == MOTOR_CONTROL_SPEED)
    {
        float speed_error_counts_s =
            motor->target_speed_counts_s - motor->actual_speed_counts_s;

        motor->speed_integral += speed_error_counts_s * motor->dt;
        motor->speed_integral = clamp_float(motor->speed_integral,
                                            -motor->max_speed_integral,
                                             motor->max_speed_integral);

        float speed_derivative =
            (speed_error_counts_s - motor->previous_speed_error) / motor->dt;

        motor->previous_speed_error = speed_error_counts_s;

        motor->last_p_term_counts_s =
            motor->speed_Kp * speed_error_counts_s;
        motor->last_i_term_counts_s =
            motor->speed_Ki * motor->speed_integral;
        motor->last_d_term_counts_s =
            motor->speed_Kd * speed_derivative;
        motor->last_correction_counts_s =
            motor->last_p_term_counts_s +
            motor->last_i_term_counts_s +
            motor->last_d_term_counts_s;
        motor->last_velocity_cmd_counts_s =
            motor->target_speed_counts_s + motor->last_correction_counts_s;

        MotorController_ApplyVelocityCounts(motor,
                                            motor->last_velocity_cmd_counts_s);
        MotorController_RequestEncoderSample();
        return;
    }

    float error_counts =
        (float)(motor->target_position_counts - actual_pos);

    /*
     * Safety fault:
     * if encoder position is too far from reference, stop.
     */
    if (fabsf(error_counts) > motor->following_error_limit_counts)
    {
        motor->fault = MOTOR_FAULT_FOLLOWING_ERROR;
        Stepper_Stop();
        MotorController_OnFault(motor->fault);
        return;
    }

    /*
     * PID terms
     */
    motor->integral += error_counts * motor->dt;
    motor->integral = clamp_float(motor->integral,
                                  -motor->max_integral,
                                   motor->max_integral);

    float derivative =
        (error_counts - motor->previous_error) / motor->dt;

    motor->previous_error = error_counts;

    /*
     * PID output = correction velocity in encoder counts/s.
     */
    motor->last_p_term_counts_s = motor->Kp * error_counts;
    motor->last_i_term_counts_s = motor->Ki * motor->integral;
    motor->last_d_term_counts_s = motor->Kd * derivative;
    motor->last_correction_counts_s =
        motor->last_p_term_counts_s +
        motor->last_i_term_counts_s +
        motor->last_d_term_counts_s;

    /*
     * Feed-forward velocity from trajectory generator/CAN command.
     */
    float velocity_cmd_counts_s =
        (motor->target_velocity_counts_s * motor->feedforward_gain) +
        (motor->last_correction_counts_s * motor->feedback_gain);

    motor->last_velocity_cmd_counts_s = velocity_cmd_counts_s;

    MotorController_ApplyVelocityCounts(motor, velocity_cmd_counts_s);
    MotorController_RequestEncoderSample();
}

void MotorController_StartRelativeProfile(MotorController_t *motor,
                                          float delta_counts)
{
    motor->profile_target_delta_counts = delta_counts;
    motor->profile_target_counts = 0.0f;
    motor->profile_final_counts = 0.0f;
    motor->profile_velocity_counts_s = 0.0f;
    motor->profile_state = MOTOR_PROFILE_WAIT_ENCODER;
}

static void MotorController_UpdateProfileTarget(MotorController_t *motor)
{
    if (!AS5600_Ready(&encoder))
    {
        MotorController_RequestEncoderSample();
        return;
    }

    switch (motor->profile_state)
    {
        case MOTOR_PROFILE_WAIT_ENCODER:
            motor->profile_target_counts = (float)AS5600_Abs(&encoder);
            motor->profile_final_counts =
                motor->profile_target_counts + motor->profile_target_delta_counts;
            motor->profile_velocity_counts_s = 0.0f;
            MotorController_SetTarget(motor,
                                      round_float_to_int(motor->profile_target_counts),
                                      0.0f);
            motor->profile_state = MOTOR_PROFILE_MOVING;
            break;

        case MOTOR_PROFILE_MOVING:
        {
            const float dt_s = 0.001f;
            float remaining_counts =
                motor->profile_final_counts - motor->profile_target_counts;
            float direction = (remaining_counts >= 0.0f) ? 1.0f : -1.0f;
            float velocity_abs = abs_float(motor->profile_velocity_counts_s);
            float braking_distance_counts =
                (velocity_abs * velocity_abs) /
                (2.0f * motor->profile_accel_counts_s2);
            float acceleration_counts_s2 = motor->profile_accel_counts_s2;

            if ((abs_float(remaining_counts) <= 0.5f) &&
                (velocity_abs < 1.0f))
            {
                motor->profile_target_counts = motor->profile_final_counts;
                motor->profile_velocity_counts_s = 0.0f;
                MotorController_SetTarget(motor,
                                          round_float_to_int(motor->profile_final_counts),
                                          0.0f);
                motor->profile_state = MOTOR_PROFILE_HOLDING;
                break;
            }

            if (abs_float(remaining_counts) <= braking_distance_counts)
            {
                acceleration_counts_s2 = -motor->profile_accel_counts_s2;
            }

            velocity_abs += acceleration_counts_s2 * dt_s;
            if (velocity_abs < 0.0f)
            {
                velocity_abs = 0.0f;
            }
            if (velocity_abs > motor->profile_max_velocity_counts_s)
            {
                velocity_abs = motor->profile_max_velocity_counts_s;
            }

            motor->profile_velocity_counts_s = direction * velocity_abs;
            motor->profile_target_counts +=
                motor->profile_velocity_counts_s * dt_s;

            if (((direction > 0.0f) &&
                 (motor->profile_target_counts > motor->profile_final_counts)) ||
                ((direction < 0.0f) &&
                 (motor->profile_target_counts < motor->profile_final_counts)))
            {
                motor->profile_target_counts = motor->profile_final_counts;
                motor->profile_velocity_counts_s = 0.0f;
            }

            MotorController_SetTarget(motor,
                                      round_float_to_int(motor->profile_target_counts),
                                      motor->profile_velocity_counts_s);
            break;
        }

        case MOTOR_PROFILE_HOLDING:
        default:
            break;
    }
}

static void MotorController_UpdateMonitor(MotorController_t *motor)
{
    float speed_counts_s = MotorController_GetActualSpeed(motor);

    motor->monitor_speed_counts_s = speed_counts_s;
    motor->monitor_accel_counts_s2 =
        (speed_counts_s - motor->monitor_previous_speed_counts_s) * 1000.0f;
    motor->monitor_previous_speed_counts_s = speed_counts_s;

    motor->monitor_timer_ms++;
    if (motor->monitor_timer_ms >= motor->monitor_interval_ms)
    {
        motor->monitor_timer_ms = 0U;
        motor->monitor_actual_counts = MotorController_GetActualPosition(motor);
        motor->monitor_target_counts = motor->target_position_counts;
        motor->monitor_target_velocity_counts_s =
            motor->target_velocity_counts_s;
        motor->monitor_error_counts = MotorController_GetPositionError(motor);
        motor->monitor_p_counts_s = motor->last_p_term_counts_s;
        motor->monitor_i_counts_s = motor->last_i_term_counts_s;
        motor->monitor_d_counts_s = motor->last_d_term_counts_s;
        motor->monitor_correction_counts_s = motor->last_correction_counts_s;
        motor->monitor_command_counts_s = motor->last_velocity_cmd_counts_s;
        motor->monitor_state = motor->profile_state;
        motor->monitor_pending = 1U;
    }
}

void MotorController_Service_1kHz(MotorController_t *motor)
{
    MotorController_UpdateProfileTarget(motor);
    MotorController_Update_1kHz(motor);
    MotorController_UpdateMonitor(motor);
}

uint8_t MotorController_MonitorPending(MotorController_t *motor)
{
    if (motor->monitor_pending == 0U)
    {
        return 0U;
    }

    motor->monitor_pending = 0U;
    return 1U;
}

void MotorController_PrintMonitor(MotorController_t *motor)
{
    printf("motor mon: state=%ld target=%ld actual=%ld err=%ld target_v=%ld speed=%ld accel=%ld p=%ld i=%ld d=%ld corr=%ld cmd=%ld\n",
           (long)motor->monitor_state,
           (long)motor->monitor_target_counts,
           (long)motor->monitor_actual_counts,
           (long)motor->monitor_error_counts,
           (long)motor->monitor_target_velocity_counts_s,
           (long)motor->monitor_speed_counts_s,
           (long)motor->monitor_accel_counts_s2,
           (long)motor->monitor_p_counts_s,
           (long)motor->monitor_i_counts_s,
           (long)motor->monitor_d_counts_s,
           (long)motor->monitor_correction_counts_s,
           (long)motor->monitor_command_counts_s);
}

int32_t MotorController_GetActualPosition(MotorController_t *motor)
{
    return motor->actual_position_counts;
}

float MotorController_GetPositionError(MotorController_t *motor)
{
    return (float)(motor->target_position_counts -
                   motor->actual_position_counts);
}

float MotorController_GetActualSpeed(MotorController_t *motor)
{
    return motor->actual_speed_counts_s;
}

float MotorController_GetTargetSpeed(MotorController_t *motor)
{
    return motor->target_speed_counts_s;
}

MotorFault_t MotorController_GetFault(MotorController_t *motor)
{
    return motor->fault;
}

const char *MotorController_FaultName(MotorFault_t fault)
{
    switch (fault)
    {
        case MOTOR_FAULT_NONE:
            return "NONE";
        case MOTOR_FAULT_FOLLOWING_ERROR:
            return "FOLLOWING_ERROR";
        case MOTOR_FAULT_ENCODER_JUMP:
            return "ENCODER_JUMP";
        case MOTOR_FAULT_COMMAND_LIMIT:
            return "COMMAND_LIMIT";
        case MOTOR_FAULT_ENCODER_NOT_READY:
            return "ENCODER_NOT_READY";
        case MOTOR_FAULT_STALL_GUARD:
            return "STALL_GUARD";
        default:
            return "UNKNOWN";
    }
}

void MotorController_ClearFault(MotorController_t *motor)
{
    motor->fault = MOTOR_FAULT_NONE;
    motor->integral = 0.0f;
    motor->previous_error = 0.0f;
    motor->speed_integral = 0.0f;
    motor->previous_speed_error = 0.0f;
    motor->previous_velocity_cmd_steps_s = 0.0f;
    motor->speed_initialized = 0U;

    MotorController_HoldCurrentPosition(motor);
}
