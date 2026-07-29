#ifndef TMC_STALL_H
#define TMC_STALL_H

#include "motor_controller.h"
#include "tmc2209.h"
#include <stdint.h>

typedef struct
{
    TMC2209_HandleTypeDef *tmc;
    MotorController_t *motor;
    volatile uint8_t diag_pending;
    uint8_t stall_latched;
} TMC_Stall_t;

HAL_StatusTypeDef TMC_Stall_Init(TMC_Stall_t *stall,
                                 TMC2209_HandleTypeDef *tmc,
                                 MotorController_t *motor);
void TMC_Stall_DiagISR(TMC_Stall_t *stall);
void TMC_Stall_Service(TMC_Stall_t *stall);

#endif
