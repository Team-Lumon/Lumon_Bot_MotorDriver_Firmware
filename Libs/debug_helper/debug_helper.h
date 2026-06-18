#ifndef DEBUG_HELPER_H
#define DEBUG_HELPER_H

#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif


uint32_t GetTimerPeripheralClockHz(const TIM_HandleTypeDef *htim);

void PrintFrequency(const char *label, uint32_t frequency_hz);

void PrintTimerFrequency(const char *name, TIM_HandleTypeDef *htim, uint8_t uses_internal_clock);



#ifdef __cplusplus
}
#endif

#endif
