#include "tmc_stall.h"

#define TMC_STALL_SGTHRS      80U
#define TMC_STALL_TCOOLTHRS   0x000FFFFFUL

HAL_StatusTypeDef TMC_Stall_Init(TMC_Stall_t *stall,
                                 TMC2209_HandleTypeDef *tmc,
                                 MotorController_t *motor)
{
    if ((stall == NULL) || (tmc == NULL) || (motor == NULL))
    {
        return HAL_ERROR;
    }

    stall->tmc = tmc;
    stall->motor = motor;
    stall->diag_pending = 0U;
    stall->stall_latched = 0U;

    if (TMC2209_SetOperationMode(tmc, TMC2209_CHOPPER_STEALTHCHOP) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (TMC2209_SetCoolStepThreshold(tmc, TMC_STALL_TCOOLTHRS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return TMC2209_SetStallGuardThreshold(tmc, TMC_STALL_SGTHRS);
}

void TMC_Stall_DiagISR(TMC_Stall_t *stall)
{
    if (stall != NULL)
    {
        stall->diag_pending = 1U;
    }
}

void TMC_Stall_Service(TMC_Stall_t *stall)
{
    if ((stall == NULL) || (stall->tmc == NULL) || (stall->motor == NULL))
    {
        return;
    }

    if ((stall->stall_latched != 0U) &&
        (MotorController_GetFault(stall->motor) == MOTOR_FAULT_NONE))
    {
        stall->stall_latched = 0U;
    }

    if (stall->diag_pending == 0U)
    {
        return;
    }
    stall->diag_pending = 0U;

    if ((stall->stall_latched != 0U) ||
        (MotorController_GetFault(stall->motor) != MOTOR_FAULT_NONE))
    {
        return;
    }

    stall->stall_latched = 1U;
    MotorController_SetFault(stall->motor, MOTOR_FAULT_STALL_GUARD);
    (void)TMC2209_DisableDriver(stall->tmc);
}
