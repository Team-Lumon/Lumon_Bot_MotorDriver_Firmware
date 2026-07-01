#include "motor_controller.h"
#include "stepper_oc.h"
#include <math.h>

/*
 * ===================== USER EDIT SECTION =====================
 */

/*
 * TODO:
 * Include your AS5600 header here.
 *
 * Example:
 * #include "as5600.h"
 */
#include "as5600.h"

/*
 * TODO:
 * Edit the encoder type if your encoder object has a different type.
 *
 * You said your encoder reading is:
 *      AS5600_Abs(&encoder)
 */
extern AS5600_t encoder;

/*
 * TODO:
 * If your AS5600_Abs() return type is not uint16_t, edit this.
 *
 * Assumption:
 *      AS5600_Abs(&encoder) returns 0 to 4095.
 */
extern int32_t AS5600_Abs(const AS5600_t *enc);

/*
 * AS5600 is 12-bit absolute encoder.
 * Raw range: 0 to 4095.
 */
#define ENCODER_CPR                 4096

/*
 * If the absolute value jumps more than half a revolution in 1 ms,
 * the unwrap logic assumes it crossed the zero point.
 */
#define ENCODER_HALF_CPR            (ENCODER_CPR / 2)

/*
 * =================== END USER EDIT SECTION ===================
 */

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

/*
 * Reads the absolute encoder and converts it into a multi-turn position.
 *
 * AS5600_Abs() only gives 0 to 4095.
 * For a cable robot, the motor will rotate many turns, so we unwrap it.
 */
static int32_t MotorController_ReadEncoderExtended(MotorController_t *motor)
{
    uint16_t raw = AS5600_Abs(&encoder);

    if (!motor->encoder_initialized)
    {
        motor->last_abs_raw = raw;
        motor->actual_position_counts = (int32_t)raw;
        motor->encoder_initialized = 1;
        return motor->actual_position_counts;
    }

    int32_t diff = (int32_t)raw - (int32_t)motor->last_abs_raw;

    /*
     * Handle wraparound:
     *
     * Example forward crossing:
     *      last = 4090, raw = 5
     *      raw - last = -4085
     *      corrected diff = +11
     *
     * Example reverse crossing:
     *      last = 5, raw = 4090
     *      raw - last = +4085
     *      corrected diff = -11
     */
    if (diff > ENCODER_HALF_CPR)
    {
        diff -= ENCODER_CPR;
    }
    else if (diff < -ENCODER_HALF_CPR)
    {
        diff += ENCODER_CPR;
    }

    motor->actual_position_counts += diff;
    motor->last_abs_raw = raw;

    return motor->actual_position_counts;
}

void MotorController_Init(MotorController_t *motor)
{
    /*
     * TODO:
     * Tune these later.
     * Start with Kp only.
     */
    motor->Kp = 5.0f;
    motor->Ki = 0.0f;
    motor->Kd = 0.0f;

    /*
     * 1 kHz controller update.
     */
    motor->dt = 0.001f;

    motor->integral = 0.0f;
    motor->previous_error = 0.0f;

    /*
     * TODO:
     * Tune these based on your motor, driver, pulley, and cable load.
     */
    motor->max_integral = 10000.0f;
    motor->max_velocity_steps_s = 5000.0f;
    motor->max_accel_steps_s2 = 20000.0f;

    /*
     * TODO:
     * Edit this.
     *
     * Example:
     *      motor full steps/rev = 200
     *      microstepping = 16
     *      step pulses/rev = 3200
     *      AS5600 counts/rev = 4096
     *
     *      steps_per_encoder_count = 3200 / 4096 = 0.78125
     */
    motor->steps_per_encoder_count = 1.0f;

    /*
     * TODO:
     * Edit this based on acceptable cable position error.
     */
    motor->following_error_limit_counts = 1000.0f;

    motor->target_position_counts = 0;
    motor->target_velocity_counts_s = 0.0f;

    motor->actual_position_counts = 0;
    motor->last_abs_raw = 0;
    motor->encoder_initialized = 0;

    motor->previous_velocity_cmd_steps_s = 0.0f;

    motor->enabled = 0;
    motor->fault = MOTOR_FAULT_NONE;

    /*
     * Initialize encoder position.
     * Make sure AS5600/I2C is already initialized before calling this.
     */
    int32_t current_pos = MotorController_ReadEncoderExtended(motor);
    motor->target_position_counts = current_pos;
}

void MotorController_Enable(MotorController_t *motor)
{
    motor->integral = 0.0f;
    motor->previous_error = 0.0f;
    motor->previous_velocity_cmd_steps_s = 0.0f;

    motor->fault = MOTOR_FAULT_NONE;

    /*
     * Start from current position to avoid sudden jump.
     */
    int32_t current_pos = MotorController_ReadEncoderExtended(motor);
    motor->target_position_counts = current_pos;
    motor->target_velocity_counts_s = 0.0f;

    Stepper_Enable();

    motor->enabled = 1;
}

void MotorController_Disable(MotorController_t *motor)
{
    motor->enabled = 0;
    Stepper_Stop();
}

void MotorController_SetTarget(MotorController_t *motor,
                               int32_t target_position_counts,
                               float target_velocity_counts_s)
{
    motor->target_position_counts = target_position_counts;
    motor->target_velocity_counts_s = target_velocity_counts_s;
}

void MotorController_SetRelativeTarget(MotorController_t *motor,
                                       int32_t delta_counts,
                                       float target_velocity_counts_s)
{
    motor->target_position_counts += delta_counts;
    motor->target_velocity_counts_s = target_velocity_counts_s;
}

void MotorController_HoldCurrentPosition(MotorController_t *motor)
{
    int32_t current_pos = MotorController_ReadEncoderExtended(motor);

    motor->target_position_counts = current_pos;
    motor->target_velocity_counts_s = 0.0f;
    motor->integral = 0.0f;
    motor->previous_error = 0.0f;
    motor->previous_velocity_cmd_steps_s = 0.0f;

    Stepper_Stop();
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

    int32_t actual_pos = MotorController_ReadEncoderExtended(motor);

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
    float correction_velocity_counts_s =
        (motor->Kp * error_counts) +
        (motor->Ki * motor->integral) +
        (motor->Kd * derivative);

    /*
     * Feed-forward velocity from trajectory generator/CAN command.
     */
    float velocity_cmd_counts_s =
        motor->target_velocity_counts_s + correction_velocity_counts_s;

    /*
     * Convert encoder-count velocity to step-pulse velocity.
     */
    float velocity_cmd_steps_s =
        velocity_cmd_counts_s * motor->steps_per_encoder_count;

    /*
     * Velocity limit.
     */
    velocity_cmd_steps_s =
        clamp_float(velocity_cmd_steps_s,
                    -motor->max_velocity_steps_s,
                     motor->max_velocity_steps_s);

    /*
     * Acceleration limit.
     */
    float max_delta_v =
        motor->max_accel_steps_s2 * motor->dt;

    float delta_v =
        velocity_cmd_steps_s - motor->previous_velocity_cmd_steps_s;

    delta_v = clamp_float(delta_v, -max_delta_v, max_delta_v);

    velocity_cmd_steps_s =
        motor->previous_velocity_cmd_steps_s + delta_v;

    motor->previous_velocity_cmd_steps_s = velocity_cmd_steps_s;

    /*
     * Finally generate step pulses.
     */
    Stepper_SetVelocityStepsPerSec(velocity_cmd_steps_s);
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

MotorFault_t MotorController_GetFault(MotorController_t *motor)
{
    return motor->fault;
}

void MotorController_ClearFault(MotorController_t *motor)
{
    motor->fault = MOTOR_FAULT_NONE;
    motor->integral = 0.0f;
    motor->previous_error = 0.0f;
    motor->previous_velocity_cmd_steps_s = 0.0f;

    MotorController_HoldCurrentPosition(motor);
}