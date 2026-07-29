#ifndef STEPPER_OC_H
#define STEPPER_OC_H

#include "main.h"
#include <stdint.h>

void Stepper_Init(void);
void Stepper_Enable(void);
void Stepper_Disable(void);
void Stepper_Stop(void);

void Stepper_SetVelocityStepsPerSec(float steps_per_sec);

/*
 * Call this inside HAL_TIM_OC_DelayElapsedCallback().
 */
void Stepper_TIM_OC_Callback(TIM_HandleTypeDef *htim);

#endif