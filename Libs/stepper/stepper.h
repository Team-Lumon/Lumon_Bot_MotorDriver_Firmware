#ifndef STEPPER_H
#define STEPPER_H

#include "main.h"
#include "stm32g0xx.h"
// #include "stm32f103xb.h"
#include <stdint.h>

typedef enum {
  EIGHTH_STEP = 8,
  SIXTEENTH_STEP = 16,
  THIRTY_SECOND_STEP = 32,
  SIXTY_FOURTH_STEP = 64
} MicrosteppingMode;

typedef struct {
    GPIO_TypeDef *step_port;
    uint16_t step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;

    GPIO_TypeDef *MS1_port;
    uint16_t MS1_pin;
    GPIO_TypeDef *MS2_port;
    uint16_t MS2_pin;

    GPIO_TypeDef *disable_port;
    uint16_t disable_pin;

    MicrosteppingMode microstepping;

    volatile int32_t *absolute_position;

} Stepper_config_t;

typedef struct
{
    GPIO_TypeDef *step_port;
    uint16_t step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;

    GPIO_TypeDef *MS1_port;
    uint16_t MS1_pin;
    GPIO_TypeDef *MS2_port;
    uint16_t MS2_pin;

    GPIO_TypeDef *disable_port;
    uint16_t disable_pin;

    MicrosteppingMode microstepping;

    int UART_address;

    volatile int32_t *absolute_position;
    int32_t commanded_position_steps;

    float ref_position_steps;
    float ref_velocity_sps;
    float cmd_velocity_sps;

    float step_accumulator;

    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;

    float max_speed_sps;
    float max_accel_sps2;

    uint8_t step_pulse_active;
} Stepper_t;

void Stepper_Init(Stepper_config_t *config, Stepper_t *m);

void Stepper_SetPID(Stepper_t *m, float kp, float ki, float kd);
void Stepper_SetLimits(Stepper_t *m, float max_speed_sps, float max_accel_sps2);
void Stepper_Reset(Stepper_t *m);

void Stepper_ControlISR(Stepper_t *m, float t, float dt);

float Trajectory_Position(float t);
float Trajectory_Velocity(float t);

#endif // STEPPER_H