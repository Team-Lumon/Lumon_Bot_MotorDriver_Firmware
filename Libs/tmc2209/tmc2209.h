#ifndef TMC2209_H
#define TMC2209_H

#include "stm32g0xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------- Registers ----------------
#define TMC2209_REG_GCONF       0x00
#define TMC2209_REG_GSTAT       0x01
#define TMC2209_REG_IFCNT       0x02
#define TMC2209_REG_NODECONF    0x03
#define TMC2209_REG_IOIN        0x06
#define TMC2209_REG_FACTORY_CONF 0x07
#define TMC2209_REG_IHOLD_IRUN  0x10
#define TMC2209_REG_TPOWERDOWN  0x11
#define TMC2209_REG_TSTEP       0x12
#define TMC2209_REG_TPWMTHRS    0x13
#define TMC2209_REG_TCOOLTHRS   0x14
#define TMC2209_REG_VACTUAL     0x22
#define TMC2209_REG_SGTHRS      0x40
#define TMC2209_REG_SG_RESULT   0x41
#define TMC2209_REG_COOLCONF    0x42
#define TMC2209_REG_MSCNT       0x6A
#define TMC2209_REG_CHOPCONF    0x6C
#define TMC2209_REG_DRV_STATUS  0x6F
#define TMC2209_REG_PWMCONF     0x70

#define TMC2209_SYNC            0x05

typedef enum
{
    TMC2209_CHOPPER_STEALTHCHOP = 0,
    TMC2209_CHOPPER_SPREADCYCLE = 1
} TMC2209_ChopperMode;

typedef enum
{
    TMC2209_MICROSTEP_256 = 0,
    TMC2209_MICROSTEP_128 = 1,
    TMC2209_MICROSTEP_64 = 2,
    TMC2209_MICROSTEP_32 = 3,
    TMC2209_MICROSTEP_16 = 4,
    TMC2209_MICROSTEP_8 = 5,
    TMC2209_MICROSTEP_4 = 6,
    TMC2209_MICROSTEP_2 = 7,
    TMC2209_MICROSTEP_FULL = 8
} TMC2209_MicrostepResolution;

typedef enum
{
    TMC2209_PWM_FREQ_2_1024 = 0,
    TMC2209_PWM_FREQ_2_683 = 1,
    TMC2209_PWM_FREQ_2_512 = 2,
    TMC2209_PWM_FREQ_2_410 = 3
} TMC2209_PwmFrequency;

typedef enum
{
    TMC2209_FREEWHEEL_NORMAL = 0,
    TMC2209_FREEWHEEL_FREEWHEEL = 1,
    TMC2209_FREEWHEEL_BRAKE_LOW_SIDE = 2,
    TMC2209_FREEWHEEL_BRAKE_HIGH_SIDE = 3
} TMC2209_FreewheelMode;

typedef struct
{
    bool use_analog_current_scale;
    bool use_internal_rsense;
    TMC2209_ChopperMode chopper_mode;
    bool invert_direction;
    bool index_otpw;
    bool index_step;
    bool use_uart_microstep_selection;
    bool multistep_filter;
} TMC2209_GeneralConfig;

typedef struct
{
    uint8_t ihold;
    uint8_t irun;
    uint8_t ihold_delay;
} TMC2209_CurrentConfig;

typedef struct
{
    uint8_t toff;
    uint8_t hstrt;
    uint8_t hend;
    uint8_t tbl;
    bool high_sensitivity_vsense;
    TMC2209_MicrostepResolution microsteps;
    bool interpolate_to_256;
    bool double_edge_step;
    bool disable_short_to_gnd_protection;
    bool disable_short_to_vs_protection;
} TMC2209_ChopperConfig;

typedef struct
{
    uint8_t pwm_ofs;
    uint8_t pwm_grad;
    TMC2209_PwmFrequency pwm_freq;
    bool pwm_autoscale;
    bool pwm_autograd;
    TMC2209_FreewheelMode freewheel;
    uint8_t pwm_reg;
    uint8_t pwm_lim;
} TMC2209_StealthChopConfig;

typedef struct
{
    uint8_t semin;
    uint8_t seup;
    uint8_t semax;
    uint8_t sedn;
    bool seimin;
} TMC2209_CoolStepConfig;

typedef struct
{
    bool standstill;
    bool stealth_chop;
    uint8_t current_scale;
    bool overtemperature_prewarning;
    bool overtemperature_120c;
    bool overtemperature_143c;
    bool overtemperature_150c;
    bool overtemperature_157c;
    bool open_load_a;
    bool open_load_b;
    bool short_to_gnd_a;
    bool short_to_gnd_b;
    bool short_to_supply_a;
    bool short_to_supply_b;
    bool overtemperature_shutdown;
} TMC2209_DriverStatus;

typedef struct
{
    uint8_t last_reg;
    uint8_t last_tx[8];
    uint8_t last_tx_len;
    uint8_t last_rx[12];
    uint8_t last_rx_len;
    HAL_StatusTypeDef last_receive_status;
    HAL_StatusTypeDef last_read_status;
    bool last_reply_found;
    uint8_t last_reply_offset;
} TMC2209_UartDebugInfo;

typedef struct
{
    UART_HandleTypeDef *huart;

    GPIO_TypeDef *enn_port;
    uint16_t enn_pin;

    uint8_t slave_addr;   // 0..3 based on MS1/MS2
    TMC2209_UartDebugInfo debug;
} TMC2209_HandleTypeDef;

// Basic API
/**
 * @brief Initializes the driver and enables UART control mode.
 * @param htmc Pointer to the TMC2209 handle.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_Init(TMC2209_HandleTypeDef *htmc);
/**
 * @brief Writes a 32-bit value to a TMC2209 register over UART.
 * @param htmc Pointer to the TMC2209 handle.
 * @param reg Register address.
 * @param value 32-bit register value to write.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_WriteRegister(TMC2209_HandleTypeDef *htmc, uint8_t reg, uint32_t value);
/**
 * @brief Reads a 32-bit value from a TMC2209 register over UART.
 * @param htmc Pointer to the TMC2209 handle.
 * @param reg Register address.
 * @param value Pointer that receives the 32-bit register value.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_ReadRegister(TMC2209_HandleTypeDef *htmc, uint8_t reg, uint32_t *value);

// Low-level helpers
/**
 * @brief Reads the IOIN register.
 * @param htmc Pointer to the TMC2209 handle.
 * @param ioin Pointer that receives the IOIN register value.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_ReadIOIN(TMC2209_HandleTypeDef *htmc, uint32_t *ioin);
/**
 * @brief Checks UART communication by reading IOIN.
 * @param htmc Pointer to the TMC2209 handle.
 * @param ioin Optional pointer that receives the IOIN value. May be NULL.
 * @return HAL_OK when communication succeeds, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_CheckConnection(TMC2209_HandleTypeDef *htmc, uint32_t *ioin);
/**
 * @brief Reads the current TSTEP value.
 * @param htmc Pointer to the TMC2209 handle.
 * @param tstep Pointer that receives the TSTEP register value.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_ReadTSTEP(TMC2209_HandleTypeDef *htmc, uint32_t *tstep);
/**
 * @brief Drives ENN low to enable the motor driver outputs.
 * @param htmc Pointer to the TMC2209 handle.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_EnableDriver(TMC2209_HandleTypeDef *htmc);
/**
 * @brief Drives ENN high to disable the motor driver outputs.
 * @param htmc Pointer to the TMC2209 handle.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_DisableDriver(TMC2209_HandleTypeDef *htmc);
/**
 * @brief Reads the GSTAT flags.
 * @param htmc Pointer to the TMC2209 handle.
 * @param gstat Pointer that receives the low GSTAT flag bits.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_ReadGlobalStatus(TMC2209_HandleTypeDef *htmc, uint8_t *gstat);
/**
 * @brief Clears selected GSTAT flags by writing them back.
 * @param htmc Pointer to the TMC2209 handle.
 * @param gstat_flags Bit mask of GSTAT flags to clear.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_ClearGlobalStatus(TMC2209_HandleTypeDef *htmc, uint8_t gstat_flags);
/**
 * @brief Reads the IFCNT interface counter.
 * @param htmc Pointer to the TMC2209 handle.
 * @param ifcnt Pointer that receives the interface counter.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_ReadInterfaceCounter(TMC2209_HandleTypeDef *htmc, uint8_t *ifcnt);
/**
 * @brief Sets the NODECONF send delay used before UART replies.
 * @param htmc Pointer to the TMC2209 handle.
 * @param send_delay Reply delay value in the range 0..15.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetSendDelay(TMC2209_HandleTypeDef *htmc, uint8_t send_delay);
/**
 * @brief Returns the most recent UART debug snapshot.
 * @param htmc Pointer to the TMC2209 handle.
 * @param debug Pointer that receives the cached debug information.
 * @return None.
 */
void TMC2209_GetUartDebugInfo(const TMC2209_HandleTypeDef *htmc, TMC2209_UartDebugInfo *debug);

// Datasheet feature helpers
/**
 * @brief Applies the high-level GCONF feature configuration.
 * @param htmc Pointer to the TMC2209 handle.
 * @param config Pointer to the general configuration values.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_ConfigureGeneral(TMC2209_HandleTypeDef *htmc, const TMC2209_GeneralConfig *config);
/**
 * @brief Applies hold/run current settings from a config struct.
 * @param htmc Pointer to the TMC2209 handle.
 * @param config Pointer to the current configuration values.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_ApplyCurrentConfig(TMC2209_HandleTypeDef *htmc,
                                             const TMC2209_CurrentConfig *config);
/**
 * @brief Sets IHOLD, IRUN, and IHOLDDELAY in IHOLD_IRUN.
 * @param htmc Pointer to the TMC2209 handle.
 * @param ihold Hold current in the range 0..31.
 * @param irun Run current in the range 0..31.
 * @param ihold_delay Hold delay in the range 0..15.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetCurrent(TMC2209_HandleTypeDef *htmc,
                                     uint8_t ihold,
                                     uint8_t irun,
                                     uint8_t ihold_delay);
/**
 * @brief Reads IHOLD, IRUN, and IHOLDDELAY from IHOLD_IRUN.
 * @param htmc Pointer to the TMC2209 handle.
 * @param config Pointer that receives the decoded current configuration.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_GetCurrent(TMC2209_HandleTypeDef *htmc,
                                     TMC2209_CurrentConfig *config);
/**
 * @brief Sets the automatic power-down delay.
 * @param htmc Pointer to the TMC2209 handle.
 * @param tpowerdown Power-down delay register value.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetPowerDownDelay(TMC2209_HandleTypeDef *htmc, uint8_t tpowerdown);
/**
 * @brief Selects stealthChop or spreadCycle operation mode.
 * @param htmc Pointer to the TMC2209 handle.
 * @param mode Requested chopper mode.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetOperationMode(TMC2209_HandleTypeDef *htmc,
                                           TMC2209_ChopperMode mode);
/**
 * @brief Reads the current stealthChop or spreadCycle selection.
 * @param htmc Pointer to the TMC2209 handle.
 * @param mode Pointer that receives the current chopper mode.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_GetOperationMode(TMC2209_HandleTypeDef *htmc,
                                           TMC2209_ChopperMode *mode);
/**
 * @brief Sets the UART-controlled microstep resolution and interpolation flag.
 * @param htmc Pointer to the TMC2209 handle.
 * @param resolution Requested microstep resolution enum value.
 * @param interpolate_to_256 True to enable interpolation to 256 microsteps.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetMicrostepResolution(TMC2209_HandleTypeDef *htmc,
                                                 TMC2209_MicrostepResolution resolution,
                                                 bool interpolate_to_256);
/**
 * @brief Convenience wrapper for setting the microstep resolution.
 * @param htmc Pointer to the TMC2209 handle.
 * @param resolution Requested microstep resolution enum value.
 * @param interpolate_to_256 True to enable interpolation to 256 microsteps.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetMicrosteps(TMC2209_HandleTypeDef *htmc,
                                        TMC2209_MicrostepResolution resolution,
                                        bool interpolate_to_256);
/**
 * @brief Reads the configured microstep resolution and optional interpolation flag.
 * @param htmc Pointer to the TMC2209 handle.
 * @param resolution Pointer that receives the current microstep resolution.
 * @param interpolate_to_256 Optional pointer that receives the interpolation state. May be NULL.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_GetMicrosteps(TMC2209_HandleTypeDef *htmc,
                                        TMC2209_MicrostepResolution *resolution,
                                        bool *interpolate_to_256);
/**
 * @brief Applies the CHOPCONF chopper configuration.
 * @param htmc Pointer to the TMC2209 handle.
 * @param config Pointer to the chopper configuration values.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_ConfigureChopper(TMC2209_HandleTypeDef *htmc,
                                           const TMC2209_ChopperConfig *config);
/**
 * @brief Applies the PWMCONF stealthChop configuration.
 * @param htmc Pointer to the TMC2209 handle.
 * @param config Pointer to the stealthChop configuration values.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_ConfigureStealthChop(TMC2209_HandleTypeDef *htmc,
                                               const TMC2209_StealthChopConfig *config);
/**
 * @brief Sets TPWMTHRS, commonly used as the spreadCycle threshold.
 * @param htmc Pointer to the TMC2209 handle.
 * @param tpwmthrs Threshold value, limited to the register bit width.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetPwmThreshold(TMC2209_HandleTypeDef *htmc, uint32_t tpwmthrs);
/**
 * @brief Sets TCOOLTHRS, used by CoolStep and StallGuard timing behavior.
 * @param htmc Pointer to the TMC2209 handle.
 * @param tcoolthrs Threshold value, limited to the register bit width.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetCoolThreshold(TMC2209_HandleTypeDef *htmc, uint32_t tcoolthrs);
/**
 * @brief Convenience wrapper for setting the spreadCycle threshold via TPWMTHRS.
 * @param htmc Pointer to the TMC2209 handle.
 * @param threshold Threshold value, limited to the register bit width.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetSpreadCycleThreshold(TMC2209_HandleTypeDef *htmc,
                                                  uint32_t threshold);
/**
 * @brief Reads the spreadCycle threshold from TPWMTHRS.
 * @param htmc Pointer to the TMC2209 handle.
 * @param threshold Pointer that receives the threshold value.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_GetSpreadCycleThreshold(TMC2209_HandleTypeDef *htmc,
                                                  uint32_t *threshold);
/**
 * @brief Convenience wrapper for setting the CoolStep threshold via TCOOLTHRS.
 * @param htmc Pointer to the TMC2209 handle.
 * @param threshold Threshold value, limited to the register bit width.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetCoolStepThreshold(TMC2209_HandleTypeDef *htmc,
                                               uint32_t threshold);
/**
 * @brief Reads the CoolStep threshold from TCOOLTHRS.
 * @param htmc Pointer to the TMC2209 handle.
 * @param threshold Pointer that receives the threshold value.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_GetCoolStepThreshold(TMC2209_HandleTypeDef *htmc,
                                               uint32_t *threshold);
/**
 * @brief Enables or disables basic CoolStep activity by updating COOLCONF.SEMIN.
 * @param htmc Pointer to the TMC2209 handle.
 * @param enabled True to enable CoolStep, false to disable it.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetCoolStepEnabled(TMC2209_HandleTypeDef *htmc, bool enabled);
/**
 * @brief Reports whether CoolStep is enabled based on COOLCONF.SEMIN.
 * @param htmc Pointer to the TMC2209 handle.
 * @param enabled Pointer that receives the CoolStep enabled state.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_GetCoolStepEnabled(TMC2209_HandleTypeDef *htmc, bool *enabled);
/**
 * @brief Sets the StallGuard threshold.
 * @param htmc Pointer to the TMC2209 handle.
 * @param sgthrs StallGuard threshold value.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetStallGuardThreshold(TMC2209_HandleTypeDef *htmc, uint8_t sgthrs);
/**
 * @brief Reads the current StallGuard result value.
 * @param htmc Pointer to the TMC2209 handle.
 * @param sg_result Pointer that receives the StallGuard result.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_ReadStallGuardResult(TMC2209_HandleTypeDef *htmc, uint16_t *sg_result);
/**
 * @brief Applies the COOLCONF CoolStep configuration.
 * @param htmc Pointer to the TMC2209 handle.
 * @param config Pointer to the CoolStep configuration values.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_ConfigureCoolStep(TMC2209_HandleTypeDef *htmc,
                                            const TMC2209_CoolStepConfig *config);
/**
 * @brief Sets the signed VACTUAL velocity register.
 * @param htmc Pointer to the TMC2209 handle.
 * @param vactual Signed velocity value within the supported register range.
 * @return HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef TMC2209_SetVActual(TMC2209_HandleTypeDef *htmc, int32_t vactual);
/**
 * @brief Reads the current microstep counter.
 * @param htmc Pointer to the TMC2209 handle.
 * @param mscnt Pointer that receives the microstep counter.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_ReadMicrostepCounter(TMC2209_HandleTypeDef *htmc, uint16_t *mscnt);
/**
 * @brief Reads the raw DRV_STATUS register.
 * @param htmc Pointer to the TMC2209 handle.
 * @param raw_status Pointer that receives the raw status word.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_ReadDriverStatus(TMC2209_HandleTypeDef *htmc, uint32_t *raw_status);
/**
 * @brief Decodes DRV_STATUS into a structured status object.
 * @param htmc Pointer to the TMC2209 handle.
 * @param status Pointer that receives the decoded driver status.
 * @return HAL_OK on success, otherwise HAL_ERROR or HAL_TIMEOUT.
 */
HAL_StatusTypeDef TMC2209_GetDriverStatus(TMC2209_HandleTypeDef *htmc,
                                          TMC2209_DriverStatus *status);

// Datasheet-inspired starting points from the quick configuration guide.
/**
 * @brief Loads a baseline stealthChop chopper and PWM configuration.
 * @param chopper Pointer that receives the default chopper settings. May be NULL.
 * @param stealth_chop Pointer that receives the default stealthChop settings. May be NULL.
 * @return None.
 */
void TMC2209_LoadStealthChopDefaults(TMC2209_ChopperConfig *chopper,
                                     TMC2209_StealthChopConfig *stealth_chop);
/**
 * @brief Loads a baseline spreadCycle chopper configuration.
 * @param chopper Pointer that receives the default chopper settings. May be NULL.
 * @return None.
 */
void TMC2209_LoadSpreadCycleDefaults(TMC2209_ChopperConfig *chopper);

#ifdef __cplusplus
}
#endif

#endif