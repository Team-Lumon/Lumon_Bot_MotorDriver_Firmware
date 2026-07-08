#include "stepper_oc.h"
#include "main.h"
#include <math.h>

/*
 * ===================== USER EDIT SECTION =====================
 *
 * Edit these according to your CubeMX names.
 */

/*
 * TODO: Change this if your STEP timer is not TIM2.
 */
extern TIM_HandleTypeDef htim2;

#define STEP_TIMER_HANDLE              htim2
#define STEP_TIMER_INSTANCE            TIM2
#define STEP_TIMER_CHANNEL             TIM_CHANNEL_1
#define STEP_TIMER_ACTIVE_CHANNEL      HAL_TIM_ACTIVE_CHANNEL_1

/*
 * Your timer clock is 64 MHz.
 * If TIM2 prescaler = 63:
 *
 * timer tick = 64 MHz / (63 + 1) = 1 MHz
 */
#define STEP_TIMER_TICK_HZ             1000000UL

/*
 * TODO: Change these pin names to match your CubeMX GPIO names.
 */
#define DIR_GPIO_PORT                  DIR_GPIO_Port
#define DIR_GPIO_PIN                   DIR_Pin
#define DIR_POSITIVE_LEVEL             GPIO_PIN_RESET
#define DIR_NEGATIVE_LEVEL             GPIO_PIN_SET

#define EN_GPIO_PORT                   Driver_disable_GPIO_Port
#define EN_GPIO_PIN                    Driver_disable_Pin

/*
 * TODO:
 * Many stepper drivers have active-low enable.
 * If your driver enables when EN = HIGH, change these.
 */
#define DRIVER_ENABLE_LEVEL            GPIO_PIN_RESET
#define DRIVER_DISABLE_LEVEL           GPIO_PIN_SET

/*
 * TODO:
 * Start conservative. Increase after checking STEP signal with oscilloscope.
 */
#define STEP_MIN_FREQ_HZ               1.0f
#define STEP_MAX_FREQ_HZ               50000.0f

/*
 * =================== END USER EDIT SECTION ===================
 */

static volatile uint32_t half_period_ticks = 0;
static volatile uint8_t stepper_running = 0;
static volatile uint8_t step_pin_level = 0;

static volatile int8_t current_dir = 1;

static float clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static void Stepper_SetDirection(int8_t dir)
{
    if (dir >= 0)
    {
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, DIR_POSITIVE_LEVEL);
        current_dir = 1;
    }
    else
    {
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, DIR_NEGATIVE_LEVEL);
        current_dir = -1;
    }
}

void Stepper_Init(void)
{
    half_period_ticks = 0;
    stepper_running = 0;
    step_pin_level = 0;
    current_dir = 1;

    Stepper_SetDirection(1);
    Stepper_Disable();
}

void Stepper_Enable(void)
{
    HAL_GPIO_WritePin(EN_GPIO_PORT, EN_GPIO_PIN, DRIVER_ENABLE_LEVEL);
}

void Stepper_Disable(void)
{
    Stepper_Stop();
    HAL_GPIO_WritePin(EN_GPIO_PORT, EN_GPIO_PIN, DRIVER_DISABLE_LEVEL);
}

void Stepper_Stop(void)
{
    HAL_TIM_OC_Stop_IT(&STEP_TIMER_HANDLE, STEP_TIMER_CHANNEL);

    stepper_running = 0;
    half_period_ticks = 0;
}

/*
 * Main function used by closed-loop controller.
 *
 * Input:
 *      steps_per_sec > 0  -> forward
 *      steps_per_sec < 0  -> reverse
 *      steps_per_sec = 0  -> stop
 */
void Stepper_SetVelocityStepsPerSec(float steps_per_sec)
{
    float freq;
    uint32_t new_half_period_ticks;
    int8_t new_dir;

    if (fabsf(steps_per_sec) < STEP_MIN_FREQ_HZ)
    {
        Stepper_Stop();
        return;
    }

    steps_per_sec = clamp_float(steps_per_sec,
                                -STEP_MAX_FREQ_HZ,
                                 STEP_MAX_FREQ_HZ);

    new_dir = (steps_per_sec >= 0.0f) ? 1 : -1;
    freq = fabsf(steps_per_sec);

    /*
     * Output compare toggle mode toggles STEP at every compare event.
     *
     * One step pulse period has:
     *      rising edge
     *      falling edge
     *
     * So we toggle at 2x the step frequency.
     */
    new_half_period_ticks =
        (uint32_t)((float)STEP_TIMER_TICK_HZ / (2.0f * freq));

    if (new_half_period_ticks < 2)
    {
        new_half_period_ticks = 2;
    }

    /*
     * If direction changes, stop first.
     * Your closed-loop acceleration limiter should normally bring speed
     * near zero before direction changes.
     */
    if (new_dir != current_dir)
    {
        Stepper_Stop();
        Stepper_SetDirection(new_dir);
    }

    half_period_ticks = new_half_period_ticks;

    if (!stepper_running)
    {
        uint32_t now;

        __HAL_TIM_SET_COUNTER(&STEP_TIMER_HANDLE, 0);
        now = __HAL_TIM_GET_COUNTER(&STEP_TIMER_HANDLE);

        __HAL_TIM_SET_COMPARE(&STEP_TIMER_HANDLE,
                              STEP_TIMER_CHANNEL,
                              now + half_period_ticks);

        step_pin_level = 0;
        stepper_running = 1;

        HAL_TIM_OC_Start_IT(&STEP_TIMER_HANDLE, STEP_TIMER_CHANNEL);
    }
}

/*
 * Call this from HAL_TIM_OC_DelayElapsedCallback().
 */
void Stepper_TIM_OC_Callback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != STEP_TIMER_INSTANCE)
    {
        return;
    }

    if (htim->Channel != STEP_TIMER_ACTIVE_CHANNEL)
    {
        return;
    }

    if (!stepper_running || half_period_ticks == 0)
    {
        return;
    }

    /*
     * Schedule next toggle relative to previous compare value.
     * This gives less jitter accumulation than scheduling from CNT.
     */
    uint32_t old_ccr = __HAL_TIM_GET_COMPARE(htim, STEP_TIMER_CHANNEL);

    __HAL_TIM_SET_COMPARE(htim,
                          STEP_TIMER_CHANNEL,
                          old_ccr + half_period_ticks);

    /*
     * Optional internal tracking only.
     * The encoder is the real position feedback.
     */
    step_pin_level ^= 1U;
}
