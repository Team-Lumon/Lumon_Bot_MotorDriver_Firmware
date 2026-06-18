#include "stepper.h"
#include "main.h"
#include "stm32f1xx_hal_gpio.h"
#include <math.h>
#include <stdint.h>

static float clampf(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

static void Stepper_SetDirection(Stepper_t *m, int dir)
{
    if (dir >= 0)
    {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_SET);
    }
}

static void Stepper_StartPulse(Stepper_t *m)
{
    HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_SET);
    m->step_pulse_active = 1;
}

static void Stepper_EndPulse(Stepper_t *m)
{
    HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_RESET);
    m->step_pulse_active = 0;
}

void Stepper_Init(Stepper_config_t *config, Stepper_t *m)
{
    m->step_port = config->step_port;
    m->step_pin = config->step_pin;
    m->dir_port = config->dir_port;
    m->dir_pin = config->dir_pin;

    m->MS1_port = config->MS1_port;
    m->MS1_pin = config->MS1_pin;
    m->MS2_port = config->MS2_port;
    m->MS2_pin = config->MS2_pin;
    m->microstepping = config->microstepping;

    m->disable_port = config->disable_port;
    m->disable_pin = config->disable_pin;

    GPIO_PinState ms1_state;
    GPIO_PinState ms2_state;

    m->absolute_position = config->absolute_position;

    switch (m->microstepping) {
        case EIGHTH_STEP:
            ms1_state = GPIO_PIN_RESET;
            ms2_state = GPIO_PIN_RESET;
            m->UART_address = 0x00;
            break;
        case THIRTY_SECOND_STEP:
            ms1_state = GPIO_PIN_SET;
            ms2_state = GPIO_PIN_RESET;
            m->UART_address = 0x01;
            break;
        case SIXTY_FOURTH_STEP:
            ms1_state = GPIO_PIN_RESET;
            ms2_state = GPIO_PIN_SET;
            m->UART_address = 0x02;
            break;
        case SIXTEENTH_STEP:
            ms1_state = GPIO_PIN_SET;
            ms2_state = GPIO_PIN_SET;
            m->UART_address = 0x03;
            break;
        default:
            ms1_state = GPIO_PIN_RESET;
            ms2_state = GPIO_PIN_RESET;
            m->UART_address = 0x00;
            break;
    }

    HAL_GPIO_WritePin(m->MS1_port, MS1_Pin, ms1_state);
    HAL_GPIO_WritePin(m->MS2_port, MS2_Pin, ms2_state);

    HAL_GPIO_WritePin(m->disable_port, m->disable_pin, GPIO_PIN_RESET); // Enable the driver
        

    m->absolute_position = 0;
    m->commanded_position_steps = 0;

    m->ref_position_steps = 0.0f;
    m->ref_velocity_sps = 0.0f;
    m->cmd_velocity_sps = 0.0f;

    m->step_accumulator = 0.0f;

    m->kp = 0.0f;
    m->ki = 0.0f;
    m->kd = 0.0f;

    m->integral = 0.0f;
    m->prev_error = 0.0f;

    m->max_speed_sps = 1000.0f;
    m->max_accel_sps2 = 1000.0f;

    m->step_pulse_active = 0;
}

void Stepper_SetPID(Stepper_t *m, float kp, float ki, float kd)
{
    m->kp = kp;
    m->ki = ki;
    m->kd = kd;
}

void Stepper_SetLimits(Stepper_t *m, float max_speed_sps, float max_accel_sps2)
{
    if (max_speed_sps < 0.0f) max_speed_sps = -max_speed_sps;
    if (max_accel_sps2 < 0.0f) max_accel_sps2 = -max_accel_sps2;

    if (max_speed_sps < 1.0f) max_speed_sps = 1.0f;
    if (max_accel_sps2 < 1.0f) max_accel_sps2 = 1.0f;

    m->max_speed_sps = max_speed_sps;
    m->max_accel_sps2 = max_accel_sps2;
}

void Stepper_Reset(Stepper_t *m)
{
    m->absolute_position = 0;
    m->commanded_position_steps = 0;
    m->ref_position_steps = 0.0f;
    m->ref_velocity_sps = 0.0f;
    m->cmd_velocity_sps = 0.0f;
    m->step_accumulator = 0.0f;
    m->integral = 0.0f;
    m->prev_error = 0.0f;
    m->step_pulse_active = 0;
}

static float PID_Update(Stepper_t *m, float error, float dt)
{
    m->integral += error * dt;

    float derivative = (error - m->prev_error) / dt;
    m->prev_error = error;

    float output = (m->kp * error) + (m->ki * m->integral) + (m->kd * derivative);
    return output;
}

void Stepper_ControlISR(Stepper_t *m, float t, float dt)
{
    if (m->step_pulse_active)
    {
        Stepper_EndPulse(m);
    }

    m->ref_position_steps = Trajectory_Position(t);
    m->ref_velocity_sps = Trajectory_Velocity(t);

    float error = m->ref_position_steps - (float)(*m->absolute_position);

    float raw_velocity_cmd = PID_Update(m, error, dt);

    raw_velocity_cmd = clampf(raw_velocity_cmd, -m->max_speed_sps, m->max_speed_sps);

    float max_dv = m->max_accel_sps2 * dt;
    float dv = raw_velocity_cmd - m->cmd_velocity_sps;

    dv = clampf(dv, -max_dv, max_dv);
    m->cmd_velocity_sps += dv;

    m->step_accumulator += m->cmd_velocity_sps * dt;

    if (m->step_accumulator >= 1.0f)
    {
        Stepper_SetDirection(m, +1);
        Stepper_StartPulse(m);
        m->step_accumulator -= 1.0f;
        m->commanded_position_steps++;
    }
    else if (m->step_accumulator <= -1.0f)
    {
        Stepper_SetDirection(m, -1);
        Stepper_StartPulse(m);
        m->step_accumulator += 1.0f;
        m->commanded_position_steps--;
    }
}

float Trajectory_Position(float t) {
  return 100.0f * t; // 100 steps per second linear trajectory
}
float Trajectory_Velocity(float t) {
    return 100.0f; // Constant velocity of 100 steps per second
}