#include "debug_helper.h"

uint32_t GetTimerPeripheralClockHz(const TIM_HandleTypeDef *htim)
{
  RCC_ClkInitTypeDef clk_config = {0};
  uint32_t flash_latency = 0;
  uint32_t pclk_hz = 0;
  uint32_t apb_divider = RCC_HCLK_DIV1;

  HAL_RCC_GetClockConfig(&clk_config, &flash_latency);

  (void)htim;
  pclk_hz = HAL_RCC_GetPCLK1Freq();
  apb_divider = clk_config.APB1CLKDivider;

  if (apb_divider != RCC_HCLK_DIV1) {
    pclk_hz *= 2U;
  }

  return pclk_hz;
} 

static void PrintScaledFrequency(const char *label,
                                 uint32_t frequency_hz,
                                 uint32_t scale_hz,
                                 const char *unit)
{
  uint32_t whole = frequency_hz / scale_hz;
  uint32_t fraction = ((frequency_hz % scale_hz) * 100U + (scale_hz / 2U)) / scale_hz;

  if (fraction >= 100U) {
    whole++;
    fraction = 0U;
  }

  printf("%s%lu.%02lu%s",
         label,
         (unsigned long)whole,
         (unsigned long)fraction,
         unit);
}

static void PrintTimeMillis(const char *label, uint64_t time_millis_x1000)
{
  uint64_t whole = time_millis_x1000 / 1000ULL;
  uint64_t fraction = time_millis_x1000 % 1000ULL;

  printf("%s%lu.%03lums",
         label,
         (unsigned long)whole,
         (unsigned long)fraction);
}

void PrintFrequency(const char *label, uint32_t frequency_hz)
{
  if (frequency_hz >= 1000000U) {
    PrintScaledFrequency(label, frequency_hz, 1000000U, "MHz");
  } else if (frequency_hz >= 1000U) {
    PrintScaledFrequency(label, frequency_hz, 1000U, "kHz");
  } else {
    printf("%s%luHz", label, (unsigned long)frequency_hz);
  }
}

void PrintTimerFrequency(const char *name,
                                TIM_HandleTypeDef *htim,
                                uint8_t uses_internal_clock)
{
  const uint32_t timer_clock_hz = GetTimerPeripheralClockHz(htim);

  printf("%s: ", name);
  PrintFrequency("tim clk=", timer_clock_hz);

  if (!uses_internal_clock) {
    printf(", counter clock depends on external ETR input\r\n");
    return;
  }

  const uint32_t tick_hz = timer_clock_hz / (htim->Init.Prescaler + 1U);
  const uint64_t update_ticks =
      (uint64_t)(htim->Init.Period + 1U) *
      (uint64_t)(htim->Init.RepetitionCounter + 1U);
  const uint64_t update_millis_x1000 =
      (update_ticks * 1000000ULL) / (uint64_t)tick_hz;

  PrintFrequency(", tick=", tick_hz);
  PrintTimeMillis(", update time=", update_millis_x1000);
  printf("\r\n");
}
