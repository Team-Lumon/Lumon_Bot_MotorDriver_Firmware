#include "tmc2209.h"

#include <string.h>

#define TMC2209_GCONF_I_SCALE_ANALOG      (1UL << 0)
#define TMC2209_GCONF_INTERNAL_RSENSE     (1UL << 1)
#define TMC2209_GCONF_EN_SPREADCYCLE      (1UL << 2)
#define TMC2209_GCONF_SHAFT               (1UL << 3)
#define TMC2209_GCONF_INDEX_OTPW          (1UL << 4)
#define TMC2209_GCONF_INDEX_STEP          (1UL << 5)
#define TMC2209_GCONF_PDN_DISABLE         (1UL << 6)
#define TMC2209_GCONF_MSTEP_REG_SELECT    (1UL << 7)
#define TMC2209_GCONF_MULTISTEP_FILT      (1UL << 8)

#define TMC2209_NODECONF_SENDDELAY_MASK   (0x0FUL << 8)

#define TMC2209_CHOPCONF_TOFF_MASK        (0x0FUL << 0)
#define TMC2209_CHOPCONF_HSTRT_MASK       (0x07UL << 4)
#define TMC2209_CHOPCONF_HEND_MASK        (0x0FUL << 7)
#define TMC2209_CHOPCONF_TBL_MASK         (0x03UL << 15)
#define TMC2209_CHOPCONF_VSENSE_MASK      (1UL << 17)
#define TMC2209_CHOPCONF_MRES_MASK        (0x0FUL << 24)
#define TMC2209_CHOPCONF_INTPOL_MASK      (1UL << 28)
#define TMC2209_CHOPCONF_DEDGE_MASK       (1UL << 29)
#define TMC2209_CHOPCONF_DISS2G_MASK      (1UL << 30)
#define TMC2209_CHOPCONF_DISS2VS_MASK     (1UL << 31)

#define TMC2209_PWMCONF_PWM_OFS_MASK      (0xFFUL << 0)
#define TMC2209_PWMCONF_PWM_GRAD_MASK     (0xFFUL << 8)
#define TMC2209_PWMCONF_PWM_FREQ_MASK     (0x03UL << 16)
#define TMC2209_PWMCONF_PWM_AUTOSCALE     (1UL << 18)
#define TMC2209_PWMCONF_PWM_AUTOGRAD      (1UL << 19)
#define TMC2209_PWMCONF_FREEWHEEL_MASK    (0x03UL << 20)
#define TMC2209_PWMCONF_PWM_REG_MASK      (0x0FUL << 24)
#define TMC2209_PWMCONF_PWM_LIM_MASK      (0x0FUL << 28)

#define TMC2209_COOLCONF_SEMIN_MASK       (0x0FUL << 0)
#define TMC2209_COOLCONF_SEUP_MASK        (0x03UL << 5)
#define TMC2209_COOLCONF_SEMAX_MASK       (0x0FUL << 8)
#define TMC2209_COOLCONF_SEDN_MASK        (0x03UL << 13)
#define TMC2209_COOLCONF_SEIMIN_MASK      (1UL << 15)

#define TMC2209_DRV_STATUS_CS_ACTUAL_MASK (0x1FUL << 16)

#define TMC2209_MAX_THRESHOLD             0x000FFFFFUL
#define TMC2209_MAX_VACTUAL               0x007FFFFFL
#define TMC2209_MIN_VACTUAL               (-0x00800000L)

#define TMC2209_WRITE_FRAME_LEN           8U
#define TMC2209_READ_REQUEST_LEN          4U
#define TMC2209_READ_REPLY_LEN            8U
#define TMC2209_READ_BUFFER_LEN           12U

#define TMC2209_UART_TX_TIMEOUT_MS        100U
#define TMC2209_UART_RX_FIRST_TIMEOUT_MS  20U
#define TMC2209_UART_RX_INTER_TIMEOUT_MS  2U

#define TMC2209_INIT_DISABLE_DELAY_MS     10U
#define TMC2209_INIT_POST_WRITE_DELAY_MS  2U
#define TMC2209_INIT_ENABLE_DELAY_MS      10U

static uint8_t TMC2209_CRC(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t current_byte = data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (((crc >> 7) ^ (current_byte & 0x01U)) != 0U)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            }
            else
            {
                crc <<= 1;
            }

            current_byte >>= 1;
        }
    }

    return crc;
}

static bool TMC2209_IsHandleValid(const TMC2209_HandleTypeDef *htmc)
{
    return (htmc != NULL) && (htmc->huart != NULL);
}

static bool TMC2209_IsDriverControlValid(const TMC2209_HandleTypeDef *htmc)
{
    return (htmc != NULL) && (htmc->enn_port != NULL);
}

static void TMC2209_DebugStoreTx(TMC2209_HandleTypeDef *htmc, const uint8_t *data, uint16_t len)
{
    uint16_t copy_len = len;

    if ((htmc == NULL) || (data == NULL))
    {
        return;
    }

    if (copy_len > sizeof(htmc->debug.last_tx))
    {
        copy_len = sizeof(htmc->debug.last_tx);
    }

    memcpy(htmc->debug.last_tx, data, copy_len);
    htmc->debug.last_tx_len = (uint8_t)copy_len;
}

static void TMC2209_DebugStoreRx(TMC2209_HandleTypeDef *htmc,
                                 const uint8_t *data,
                                 uint16_t len,
                                 HAL_StatusTypeDef status)
{
    uint16_t copy_len = len;

    if ((htmc == NULL) || (data == NULL))
    {
        return;
    }

    if (copy_len > sizeof(htmc->debug.last_rx))
    {
        copy_len = sizeof(htmc->debug.last_rx);
    }

    memcpy(htmc->debug.last_rx, data, copy_len);
    htmc->debug.last_rx_len = (uint8_t)copy_len;
    htmc->debug.last_receive_status = status;
}

static void TMC2209_DebugResetRead(TMC2209_HandleTypeDef *htmc, uint8_t reg)
{
    if (htmc == NULL)
    {
        return;
    }

    memset(htmc->debug.last_rx, 0, sizeof(htmc->debug.last_rx));
    htmc->debug.last_rx_len = 0;
    htmc->debug.last_receive_status = HAL_ERROR;
    htmc->debug.last_read_status = HAL_ERROR;
    htmc->debug.last_reply_found = false;
    htmc->debug.last_reply_offset = 0xFFU;
    htmc->debug.last_reg = reg;
}

static void TMC2209_UART_ClearRx(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(huart);

    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
    {
        (void)huart->Instance->RDR;
    }
}

static HAL_StatusTypeDef TMC2209_UART_EnableTransmitter(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_HalfDuplex_EnableTransmitter(huart);
}

static HAL_StatusTypeDef TMC2209_UART_EnableReceiver(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_HalfDuplex_EnableReceiver(huart);
}

static HAL_StatusTypeDef TMC2209_UART_WaitForTc(UART_HandleTypeDef *huart, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef TMC2209_UART_Transmit(TMC2209_HandleTypeDef *htmc,
                                               const uint8_t *data,
                                               uint16_t len)
{
    if (!TMC2209_IsHandleValid(htmc) || (data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    TMC2209_DebugStoreTx(htmc, data, len);

    if (TMC2209_UART_EnableTransmitter(htmc->huart) != HAL_OK)
    {
        return HAL_ERROR;
    }

    TMC2209_UART_ClearRx(htmc->huart);

    if (HAL_UART_Transmit(htmc->huart, (uint8_t *)data, len, TMC2209_UART_TX_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (TMC2209_UART_WaitForTc(htmc->huart, TMC2209_UART_TX_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef TMC2209_UART_ReceiveAvailable(TMC2209_HandleTypeDef *htmc,
                                                       uint8_t *data,
                                                       uint16_t max_len,
                                                       uint16_t *received_len,
                                                       uint32_t first_byte_timeout_ms,
                                                       uint32_t inter_byte_timeout_ms)
{
    uint16_t count = 0;
    uint32_t timeout_ms = first_byte_timeout_ms;

    if (!TMC2209_IsHandleValid(htmc) || (data == NULL) || (received_len == NULL) || (max_len == 0U))
    {
        return HAL_ERROR;
    }

    if (TMC2209_UART_EnableReceiver(htmc->huart) != HAL_OK)
    {
        return HAL_ERROR;
    }

    __HAL_UART_CLEAR_OREFLAG(htmc->huart);

    while (count < max_len)
    {
        HAL_StatusTypeDef status = HAL_UART_Receive(htmc->huart, &data[count], 1, timeout_ms);

        if (status == HAL_OK)
        {
            count++;
            timeout_ms = inter_byte_timeout_ms;
            continue;
        }

        if ((status == HAL_TIMEOUT) && (count > 0U))
        {
            TMC2209_DebugStoreRx(htmc, data, count, HAL_OK);
            *received_len = count;
            return HAL_OK;
        }

        TMC2209_DebugStoreRx(htmc, data, count, status);
        *received_len = count;
        return status;
    }

    TMC2209_DebugStoreRx(htmc, data, count, HAL_OK);
    *received_len = count;
    return HAL_OK;
}

static HAL_StatusTypeDef TMC2209_UpdateRegister(TMC2209_HandleTypeDef *htmc,
                                                uint8_t reg,
                                                uint32_t mask,
                                                uint32_t value)
{
    uint32_t current = 0;

    if (TMC2209_ReadRegister(htmc, reg, &current) != HAL_OK)
    {
        return HAL_ERROR;
    }

    current &= ~mask;
    current |= (value & mask);

    return TMC2209_WriteRegister(htmc, reg, current);
}

static bool TMC2209_IsValidMicrostepResolution(TMC2209_MicrostepResolution resolution)
{
    return resolution <= TMC2209_MICROSTEP_FULL;
}

static bool TMC2209_IsValidChopperMode(TMC2209_ChopperMode mode)
{
    return (mode == TMC2209_CHOPPER_STEALTHCHOP) ||
           (mode == TMC2209_CHOPPER_SPREADCYCLE);
}

HAL_StatusTypeDef TMC2209_EnableDriver(TMC2209_HandleTypeDef *htmc)
{
    if (!TMC2209_IsDriverControlValid(htmc))
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(htmc->enn_port, htmc->enn_pin, GPIO_PIN_RESET);
    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_DisableDriver(TMC2209_HandleTypeDef *htmc)
{
    if (!TMC2209_IsDriverControlValid(htmc))
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(htmc->enn_port, htmc->enn_pin, GPIO_PIN_SET);
    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_WriteRegister(TMC2209_HandleTypeDef *htmc, uint8_t reg, uint32_t value)
{
    uint8_t tx[TMC2209_WRITE_FRAME_LEN];

    if (!TMC2209_IsHandleValid(htmc))
    {
        return HAL_ERROR;
    }

    tx[0] = TMC2209_SYNC;
    tx[1] = htmc->slave_addr & 0x03U;
    tx[2] = reg | 0x80U;
    tx[3] = (uint8_t)((value >> 24) & 0xFFU);
    tx[4] = (uint8_t)((value >> 16) & 0xFFU);
    tx[5] = (uint8_t)((value >> 8) & 0xFFU);
    tx[6] = (uint8_t)(value & 0xFFU);
    tx[7] = TMC2209_CRC(tx, TMC2209_WRITE_FRAME_LEN - 1U);

    return TMC2209_UART_Transmit(htmc, tx, sizeof(tx));
}

HAL_StatusTypeDef TMC2209_ReadRegister(TMC2209_HandleTypeDef *htmc, uint8_t reg, uint32_t *value)
{
    uint8_t tx[TMC2209_READ_REQUEST_LEN];
    uint8_t rx[TMC2209_READ_BUFFER_LEN];
    uint8_t *reply = NULL;
    uint16_t rx_len = 0;

    if (!TMC2209_IsHandleValid(htmc) || (value == NULL))
    {
        return HAL_ERROR;
    }

    TMC2209_DebugResetRead(htmc, reg);

    tx[0] = TMC2209_SYNC;
    tx[1] = htmc->slave_addr & 0x03U;
    tx[2] = reg & 0x7FU;
    tx[3] = TMC2209_CRC(tx, TMC2209_READ_REQUEST_LEN - 1U);

    if (TMC2209_UART_Transmit(htmc, tx, sizeof(tx)) != HAL_OK)
    {
        htmc->debug.last_read_status = HAL_ERROR;
        return HAL_ERROR;
    }

    htmc->debug.last_read_status = TMC2209_UART_ReceiveAvailable(htmc,
                                                                 rx,
                                                                 sizeof(rx),
                                                                 &rx_len,
                                                                 TMC2209_UART_RX_FIRST_TIMEOUT_MS,
                                                                 TMC2209_UART_RX_INTER_TIMEOUT_MS);
    if (htmc->debug.last_read_status != HAL_OK)
    {
        return htmc->debug.last_read_status;
    }

    if (rx_len < TMC2209_READ_REPLY_LEN)
    {
        htmc->debug.last_read_status = HAL_ERROR;
        return HAL_ERROR;
    }

    for (uint8_t i = 0; i <= (rx_len - TMC2209_READ_REPLY_LEN); i++)
    {
        if ((rx[i + 0] == TMC2209_SYNC) &&
            (rx[i + 1] == 0xFFU) &&
            ((rx[i + 2] & 0x7FU) == (reg & 0x7FU)) &&
            (TMC2209_CRC(&rx[i], TMC2209_READ_REPLY_LEN - 1U) == rx[i + 7]))
        {
            reply = &rx[i];
            htmc->debug.last_reply_found = true;
            htmc->debug.last_reply_offset = i;
            break;
        }
    }

    if (reply == NULL)
    {
        htmc->debug.last_read_status = HAL_ERROR;
        return HAL_ERROR;
    }

    *value = ((uint32_t)reply[3] << 24) |
             ((uint32_t)reply[4] << 16) |
             ((uint32_t)reply[5] << 8) |
             ((uint32_t)reply[6]);

    htmc->debug.last_read_status = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_ReadIOIN(TMC2209_HandleTypeDef *htmc, uint32_t *ioin)
{
    return TMC2209_ReadRegister(htmc, TMC2209_REG_IOIN, ioin);
}

HAL_StatusTypeDef TMC2209_CheckConnection(TMC2209_HandleTypeDef *htmc, uint32_t *ioin)
{
    uint32_t readback = 0;

    if (ioin == NULL)
    {
        ioin = &readback;
    }

    return TMC2209_ReadIOIN(htmc, ioin);
}

HAL_StatusTypeDef TMC2209_ReadTSTEP(TMC2209_HandleTypeDef *htmc, uint32_t *tstep)
{
    return TMC2209_ReadRegister(htmc, TMC2209_REG_TSTEP, tstep);
}

HAL_StatusTypeDef TMC2209_Init(TMC2209_HandleTypeDef *htmc)
{
    uint32_t gconf = 0;

    if (!TMC2209_IsHandleValid(htmc) || !TMC2209_IsDriverControlValid(htmc))
    {
        return HAL_ERROR;
    }

    TMC2209_DisableDriver(htmc);
    HAL_Delay(TMC2209_INIT_DISABLE_DELAY_MS);

    if (TMC2209_ReadRegister(htmc, TMC2209_REG_GCONF, &gconf) != HAL_OK)
    {
        gconf = 0;
    }

    gconf |= TMC2209_GCONF_PDN_DISABLE;

    if (TMC2209_WriteRegister(htmc, TMC2209_REG_GCONF, gconf) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(TMC2209_INIT_POST_WRITE_DELAY_MS);

    TMC2209_EnableDriver(htmc);
    HAL_Delay(TMC2209_INIT_ENABLE_DELAY_MS);

    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_ReadGlobalStatus(TMC2209_HandleTypeDef *htmc, uint8_t *gstat)
{
    uint32_t value = 0;

    if ((htmc == NULL) || (gstat == NULL))
    {
        return HAL_ERROR;
    }

    if (TMC2209_ReadRegister(htmc, TMC2209_REG_GSTAT, &value) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *gstat = (uint8_t)(value & 0x07U);
    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_ClearGlobalStatus(TMC2209_HandleTypeDef *htmc, uint8_t gstat_flags)
{
    if (htmc == NULL)
    {
        return HAL_ERROR;
    }

    return TMC2209_WriteRegister(htmc, TMC2209_REG_GSTAT, (uint32_t)(gstat_flags & 0x07U));
}

HAL_StatusTypeDef TMC2209_ReadInterfaceCounter(TMC2209_HandleTypeDef *htmc, uint8_t *ifcnt)
{
    uint32_t value = 0;

    if ((htmc == NULL) || (ifcnt == NULL))
    {
        return HAL_ERROR;
    }

    if (TMC2209_ReadRegister(htmc, TMC2209_REG_IFCNT, &value) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *ifcnt = (uint8_t)(value & 0xFFU);
    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_SetSendDelay(TMC2209_HandleTypeDef *htmc, uint8_t send_delay)
{
    if ((htmc == NULL) || (send_delay > 15U))
    {
        return HAL_ERROR;
    }

    return TMC2209_UpdateRegister(htmc,
                                  TMC2209_REG_NODECONF,
                                  TMC2209_NODECONF_SENDDELAY_MASK,
                                  ((uint32_t)send_delay << 8) & TMC2209_NODECONF_SENDDELAY_MASK);
}

void TMC2209_GetUartDebugInfo(const TMC2209_HandleTypeDef *htmc, TMC2209_UartDebugInfo *debug)
{
    if ((htmc == NULL) || (debug == NULL))
    {
        return;
    }

    *debug = htmc->debug;
}

HAL_StatusTypeDef TMC2209_ConfigureGeneral(TMC2209_HandleTypeDef *htmc,
                                           const TMC2209_GeneralConfig *config)
{
    uint32_t mask = 0;
    uint32_t value = 0;

    if ((htmc == NULL) || (config == NULL))
    {
        return HAL_ERROR;
    }

    mask = TMC2209_GCONF_I_SCALE_ANALOG |
           TMC2209_GCONF_INTERNAL_RSENSE |
           TMC2209_GCONF_EN_SPREADCYCLE |
           TMC2209_GCONF_SHAFT |
           TMC2209_GCONF_INDEX_OTPW |
           TMC2209_GCONF_INDEX_STEP |
           TMC2209_GCONF_MSTEP_REG_SELECT |
           TMC2209_GCONF_MULTISTEP_FILT;

    if (config->use_analog_current_scale)
    {
        value |= TMC2209_GCONF_I_SCALE_ANALOG;
    }
    if (config->use_internal_rsense)
    {
        value |= TMC2209_GCONF_INTERNAL_RSENSE;
    }
    if (config->chopper_mode == TMC2209_CHOPPER_SPREADCYCLE)
    {
        value |= TMC2209_GCONF_EN_SPREADCYCLE;
    }
    if (config->invert_direction)
    {
        value |= TMC2209_GCONF_SHAFT;
    }
    if (config->index_otpw)
    {
        value |= TMC2209_GCONF_INDEX_OTPW;
    }
    if (config->index_step)
    {
        value |= TMC2209_GCONF_INDEX_STEP;
    }
    if (config->use_uart_microstep_selection)
    {
        value |= TMC2209_GCONF_MSTEP_REG_SELECT;
    }
    if (config->multistep_filter)
    {
        value |= TMC2209_GCONF_MULTISTEP_FILT;
    }

    return TMC2209_UpdateRegister(htmc, TMC2209_REG_GCONF, mask, value);
}

HAL_StatusTypeDef TMC2209_ApplyCurrentConfig(TMC2209_HandleTypeDef *htmc,
                                             const TMC2209_CurrentConfig *config)
{
    if ((htmc == NULL) || (config == NULL))
    {
        return HAL_ERROR;
    }

    return TMC2209_SetCurrent(htmc, config->ihold, config->irun, config->ihold_delay);
}

HAL_StatusTypeDef TMC2209_SetCurrent(TMC2209_HandleTypeDef *htmc,
                                     uint8_t ihold,
                                     uint8_t irun,
                                     uint8_t ihold_delay)
{
    uint32_t value = 0;

    if ((htmc == NULL) || (ihold > 31U) || (irun > 31U) || (ihold_delay > 15U))
    {
        return HAL_ERROR;
    }

    value = ((uint32_t)ihold) |
            ((uint32_t)irun << 8) |
            ((uint32_t)ihold_delay << 16);

    return TMC2209_WriteRegister(htmc, TMC2209_REG_IHOLD_IRUN, value);
}

HAL_StatusTypeDef TMC2209_GetCurrent(TMC2209_HandleTypeDef *htmc,
                                     TMC2209_CurrentConfig *config)
{
    uint32_t value = 0;

    if ((htmc == NULL) || (config == NULL))
    {
        return HAL_ERROR;
    }

    if (TMC2209_ReadRegister(htmc, TMC2209_REG_IHOLD_IRUN, &value) != HAL_OK)
    {
        return HAL_ERROR;
    }

    config->ihold = (uint8_t)(value & 0x1FU);
    config->irun = (uint8_t)((value >> 8) & 0x1FU);
    config->ihold_delay = (uint8_t)((value >> 16) & 0x0FU);

    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_SetPowerDownDelay(TMC2209_HandleTypeDef *htmc, uint8_t tpowerdown)
{
    if (htmc == NULL)
    {
        return HAL_ERROR;
    }

    return TMC2209_WriteRegister(htmc, TMC2209_REG_TPOWERDOWN, (uint32_t)tpowerdown);
}

HAL_StatusTypeDef TMC2209_SetOperationMode(TMC2209_HandleTypeDef *htmc,
                                           TMC2209_ChopperMode mode)
{
    if ((htmc == NULL) || !TMC2209_IsValidChopperMode(mode))
    {
        return HAL_ERROR;
    }

    return TMC2209_UpdateRegister(htmc,
                                  TMC2209_REG_GCONF,
                                  TMC2209_GCONF_EN_SPREADCYCLE,
                                  (mode == TMC2209_CHOPPER_SPREADCYCLE) ?
                                      TMC2209_GCONF_EN_SPREADCYCLE :
                                      0U);
}

HAL_StatusTypeDef TMC2209_GetOperationMode(TMC2209_HandleTypeDef *htmc,
                                           TMC2209_ChopperMode *mode)
{
    uint32_t gconf = 0;

    if ((htmc == NULL) || (mode == NULL))
    {
        return HAL_ERROR;
    }

    if (TMC2209_ReadRegister(htmc, TMC2209_REG_GCONF, &gconf) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *mode = (gconf & TMC2209_GCONF_EN_SPREADCYCLE) != 0U ?
                TMC2209_CHOPPER_SPREADCYCLE :
                TMC2209_CHOPPER_STEALTHCHOP;

    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_SetMicrostepResolution(TMC2209_HandleTypeDef *htmc,
                                                 TMC2209_MicrostepResolution resolution,
                                                 bool interpolate_to_256)
{
    uint32_t chopconf_value = 0;

    if ((htmc == NULL) || !TMC2209_IsValidMicrostepResolution(resolution))
    {
        return HAL_ERROR;
    }

    if (TMC2209_UpdateRegister(htmc,
                               TMC2209_REG_GCONF,
                               TMC2209_GCONF_MSTEP_REG_SELECT,
                               TMC2209_GCONF_MSTEP_REG_SELECT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    chopconf_value = ((uint32_t)resolution << 24);
    if (interpolate_to_256)
    {
        chopconf_value |= TMC2209_CHOPCONF_INTPOL_MASK;
    }

    return TMC2209_UpdateRegister(htmc,
                                  TMC2209_REG_CHOPCONF,
                                  TMC2209_CHOPCONF_MRES_MASK | TMC2209_CHOPCONF_INTPOL_MASK,
                                  chopconf_value);
}

HAL_StatusTypeDef TMC2209_SetMicrosteps(TMC2209_HandleTypeDef *htmc,
                                        TMC2209_MicrostepResolution resolution,
                                        bool interpolate_to_256)
{
    return TMC2209_SetMicrostepResolution(htmc, resolution, interpolate_to_256);
}

HAL_StatusTypeDef TMC2209_GetMicrosteps(TMC2209_HandleTypeDef *htmc,
                                        TMC2209_MicrostepResolution *resolution,
                                        bool *interpolate_to_256)
{
    uint32_t chopconf = 0;
    TMC2209_MicrostepResolution mres = TMC2209_MICROSTEP_256;

    if ((htmc == NULL) || (resolution == NULL))
    {
        return HAL_ERROR;
    }

    if (TMC2209_ReadRegister(htmc, TMC2209_REG_CHOPCONF, &chopconf) != HAL_OK)
    {
        return HAL_ERROR;
    }

    mres = (TMC2209_MicrostepResolution)((chopconf & TMC2209_CHOPCONF_MRES_MASK) >> 24);
    if (!TMC2209_IsValidMicrostepResolution(mres))
    {
        return HAL_ERROR;
    }

    *resolution = mres;
    if (interpolate_to_256 != NULL)
    {
        *interpolate_to_256 = (chopconf & TMC2209_CHOPCONF_INTPOL_MASK) != 0U;
    }

    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_ConfigureChopper(TMC2209_HandleTypeDef *htmc,
                                           const TMC2209_ChopperConfig *config)
{
    uint32_t value = 0;
    uint32_t mask = 0;

    if ((htmc == NULL) || (config == NULL))
    {
        return HAL_ERROR;
    }

    if ((config->toff > 15U) ||
        (config->hstrt > 7U) ||
        (config->hend > 15U) ||
        (config->tbl > 3U) ||
        !TMC2209_IsValidMicrostepResolution(config->microsteps))
    {
        return HAL_ERROR;
    }

    value = ((uint32_t)config->toff << 0) |
            ((uint32_t)config->hstrt << 4) |
            ((uint32_t)config->hend << 7) |
            ((uint32_t)config->tbl << 15) |
            ((uint32_t)config->microsteps << 24);

    if (config->high_sensitivity_vsense)
    {
        value |= TMC2209_CHOPCONF_VSENSE_MASK;
    }
    if (config->interpolate_to_256)
    {
        value |= TMC2209_CHOPCONF_INTPOL_MASK;
    }
    if (config->double_edge_step)
    {
        value |= TMC2209_CHOPCONF_DEDGE_MASK;
    }
    if (config->disable_short_to_gnd_protection)
    {
        value |= TMC2209_CHOPCONF_DISS2G_MASK;
    }
    if (config->disable_short_to_vs_protection)
    {
        value |= TMC2209_CHOPCONF_DISS2VS_MASK;
    }

    mask = TMC2209_CHOPCONF_TOFF_MASK |
           TMC2209_CHOPCONF_HSTRT_MASK |
           TMC2209_CHOPCONF_HEND_MASK |
           TMC2209_CHOPCONF_TBL_MASK |
           TMC2209_CHOPCONF_VSENSE_MASK |
           TMC2209_CHOPCONF_MRES_MASK |
           TMC2209_CHOPCONF_INTPOL_MASK |
           TMC2209_CHOPCONF_DEDGE_MASK |
           TMC2209_CHOPCONF_DISS2G_MASK |
           TMC2209_CHOPCONF_DISS2VS_MASK;

    return TMC2209_UpdateRegister(htmc, TMC2209_REG_CHOPCONF, mask, value);
}

HAL_StatusTypeDef TMC2209_ConfigureStealthChop(TMC2209_HandleTypeDef *htmc,
                                               const TMC2209_StealthChopConfig *config)
{
    uint32_t value = 0;

    if ((htmc == NULL) || (config == NULL))
    {
        return HAL_ERROR;
    }

    if ((config->pwm_reg > 15U) || (config->pwm_lim > 15U))
    {
        return HAL_ERROR;
    }

    value = ((uint32_t)config->pwm_ofs << 0) |
            ((uint32_t)config->pwm_grad << 8) |
            ((uint32_t)config->pwm_freq << 16) |
            ((uint32_t)config->freewheel << 20) |
            ((uint32_t)config->pwm_reg << 24) |
            ((uint32_t)config->pwm_lim << 28);

    if (config->pwm_autoscale)
    {
        value |= TMC2209_PWMCONF_PWM_AUTOSCALE;
    }
    if (config->pwm_autograd)
    {
        value |= TMC2209_PWMCONF_PWM_AUTOGRAD;
    }

    return TMC2209_UpdateRegister(htmc,
                                  TMC2209_REG_PWMCONF,
                                  TMC2209_PWMCONF_PWM_OFS_MASK |
                                      TMC2209_PWMCONF_PWM_GRAD_MASK |
                                      TMC2209_PWMCONF_PWM_FREQ_MASK |
                                      TMC2209_PWMCONF_PWM_AUTOSCALE |
                                      TMC2209_PWMCONF_PWM_AUTOGRAD |
                                      TMC2209_PWMCONF_FREEWHEEL_MASK |
                                      TMC2209_PWMCONF_PWM_REG_MASK |
                                      TMC2209_PWMCONF_PWM_LIM_MASK,
                                  value);
}

HAL_StatusTypeDef TMC2209_SetPwmThreshold(TMC2209_HandleTypeDef *htmc, uint32_t tpwmthrs)
{
    if ((htmc == NULL) || (tpwmthrs > TMC2209_MAX_THRESHOLD))
    {
        return HAL_ERROR;
    }

    return TMC2209_WriteRegister(htmc, TMC2209_REG_TPWMTHRS, tpwmthrs);
}

HAL_StatusTypeDef TMC2209_SetSpreadCycleThreshold(TMC2209_HandleTypeDef *htmc,
                                                  uint32_t threshold)
{
    return TMC2209_SetPwmThreshold(htmc, threshold);
}

HAL_StatusTypeDef TMC2209_GetSpreadCycleThreshold(TMC2209_HandleTypeDef *htmc,
                                                  uint32_t *threshold)
{
    uint32_t value = 0;

    if ((htmc == NULL) || (threshold == NULL))
    {
        return HAL_ERROR;
    }

    if (TMC2209_ReadRegister(htmc, TMC2209_REG_TPWMTHRS, &value) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *threshold = value & TMC2209_MAX_THRESHOLD;
    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_SetCoolThreshold(TMC2209_HandleTypeDef *htmc, uint32_t tcoolthrs)
{
    if ((htmc == NULL) || (tcoolthrs > TMC2209_MAX_THRESHOLD))
    {
        return HAL_ERROR;
    }

    return TMC2209_WriteRegister(htmc, TMC2209_REG_TCOOLTHRS, tcoolthrs);
}

HAL_StatusTypeDef TMC2209_SetCoolStepThreshold(TMC2209_HandleTypeDef *htmc,
                                               uint32_t threshold)
{
    return TMC2209_SetCoolThreshold(htmc, threshold);
}

HAL_StatusTypeDef TMC2209_GetCoolStepThreshold(TMC2209_HandleTypeDef *htmc,
                                               uint32_t *threshold)
{
    uint32_t value = 0;

    if ((htmc == NULL) || (threshold == NULL))
    {
        return HAL_ERROR;
    }

    if (TMC2209_ReadRegister(htmc, TMC2209_REG_TCOOLTHRS, &value) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *threshold = value & TMC2209_MAX_THRESHOLD;
    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_SetCoolStepEnabled(TMC2209_HandleTypeDef *htmc, bool enabled)
{
    if (htmc == NULL)
    {
        return HAL_ERROR;
    }

    return TMC2209_UpdateRegister(htmc,
                                  TMC2209_REG_COOLCONF,
                                  TMC2209_COOLCONF_SEMIN_MASK,
                                  enabled ? 1U : 0U);
}

HAL_StatusTypeDef TMC2209_GetCoolStepEnabled(TMC2209_HandleTypeDef *htmc, bool *enabled)
{
    uint32_t coolconf = 0;

    if ((htmc == NULL) || (enabled == NULL))
    {
        return HAL_ERROR;
    }

    if (TMC2209_ReadRegister(htmc, TMC2209_REG_COOLCONF, &coolconf) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *enabled = (coolconf & TMC2209_COOLCONF_SEMIN_MASK) != 0U;
    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_SetStallGuardThreshold(TMC2209_HandleTypeDef *htmc, uint8_t sgthrs)
{
    if (htmc == NULL)
    {
        return HAL_ERROR;
    }

    return TMC2209_WriteRegister(htmc, TMC2209_REG_SGTHRS, (uint32_t)sgthrs);
}

HAL_StatusTypeDef TMC2209_ReadStallGuardResult(TMC2209_HandleTypeDef *htmc, uint16_t *sg_result)
{
    uint32_t value = 0;

    if ((htmc == NULL) || (sg_result == NULL))
    {
        return HAL_ERROR;
    }

    if (TMC2209_ReadRegister(htmc, TMC2209_REG_SG_RESULT, &value) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *sg_result = (uint16_t)(value & 0x03FFU);
    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_ConfigureCoolStep(TMC2209_HandleTypeDef *htmc,
                                            const TMC2209_CoolStepConfig *config)
{
    uint32_t value = 0;

    if ((htmc == NULL) || (config == NULL))
    {
        return HAL_ERROR;
    }

    if ((config->semin > 15U) ||
        (config->seup > 3U) ||
        (config->semax > 15U) ||
        (config->sedn > 3U))
    {
        return HAL_ERROR;
    }

    value = ((uint32_t)config->semin << 0) |
            ((uint32_t)config->seup << 5) |
            ((uint32_t)config->semax << 8) |
            ((uint32_t)config->sedn << 13);

    if (config->seimin)
    {
        value |= TMC2209_COOLCONF_SEIMIN_MASK;
    }

    return TMC2209_UpdateRegister(htmc,
                                  TMC2209_REG_COOLCONF,
                                  TMC2209_COOLCONF_SEMIN_MASK |
                                      TMC2209_COOLCONF_SEUP_MASK |
                                      TMC2209_COOLCONF_SEMAX_MASK |
                                      TMC2209_COOLCONF_SEDN_MASK |
                                      TMC2209_COOLCONF_SEIMIN_MASK,
                                  value);
}

HAL_StatusTypeDef TMC2209_SetVActual(TMC2209_HandleTypeDef *htmc, int32_t vactual)
{
    if ((htmc == NULL) || (vactual < TMC2209_MIN_VACTUAL) || (vactual > TMC2209_MAX_VACTUAL))
    {
        return HAL_ERROR;
    }

    return TMC2209_WriteRegister(htmc,
                                 TMC2209_REG_VACTUAL,
                                 ((uint32_t)vactual) & 0x00FFFFFFUL);
}

HAL_StatusTypeDef TMC2209_ReadMicrostepCounter(TMC2209_HandleTypeDef *htmc, uint16_t *mscnt)
{
    uint32_t value = 0;

    if ((htmc == NULL) || (mscnt == NULL))
    {
        return HAL_ERROR;
    }

    if (TMC2209_ReadRegister(htmc, TMC2209_REG_MSCNT, &value) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *mscnt = (uint16_t)(value & 0x03FFU);
    return HAL_OK;
}

HAL_StatusTypeDef TMC2209_ReadDriverStatus(TMC2209_HandleTypeDef *htmc, uint32_t *raw_status)
{
    return TMC2209_ReadRegister(htmc, TMC2209_REG_DRV_STATUS, raw_status);
}

HAL_StatusTypeDef TMC2209_GetDriverStatus(TMC2209_HandleTypeDef *htmc,
                                          TMC2209_DriverStatus *status)
{
    uint32_t raw_status = 0;

    if ((htmc == NULL) || (status == NULL))
    {
        return HAL_ERROR;
    }

    if (TMC2209_ReadDriverStatus(htmc, &raw_status) != HAL_OK)
    {
        return HAL_ERROR;
    }

    memset(status, 0, sizeof(*status));
    status->standstill = (raw_status & (1UL << 31)) != 0U;
    status->stealth_chop = (raw_status & (1UL << 30)) != 0U;
    status->current_scale = (uint8_t)((raw_status & TMC2209_DRV_STATUS_CS_ACTUAL_MASK) >> 16);
    status->overtemperature_157c = (raw_status & (1UL << 11)) != 0U;
    status->overtemperature_150c = (raw_status & (1UL << 10)) != 0U;
    status->overtemperature_143c = (raw_status & (1UL << 9)) != 0U;
    status->overtemperature_120c = (raw_status & (1UL << 8)) != 0U;
    status->open_load_b = (raw_status & (1UL << 7)) != 0U;
    status->open_load_a = (raw_status & (1UL << 6)) != 0U;
    status->short_to_supply_b = (raw_status & (1UL << 5)) != 0U;
    status->short_to_supply_a = (raw_status & (1UL << 4)) != 0U;
    status->short_to_gnd_b = (raw_status & (1UL << 3)) != 0U;
    status->short_to_gnd_a = (raw_status & (1UL << 2)) != 0U;
    status->overtemperature_shutdown = (raw_status & (1UL << 1)) != 0U;
    status->overtemperature_prewarning = (raw_status & (1UL << 0)) != 0U;

    return HAL_OK;
}

void TMC2209_LoadStealthChopDefaults(TMC2209_ChopperConfig *chopper,
                                     TMC2209_StealthChopConfig *stealth_chop)
{
    if (chopper != NULL)
    {
        chopper->toff = 5U;
        chopper->hstrt = 4U;
        chopper->hend = 0U;
        chopper->tbl = 2U;
        chopper->high_sensitivity_vsense = false;
        chopper->microsteps = TMC2209_MICROSTEP_256;
        chopper->interpolate_to_256 = true;
        chopper->double_edge_step = false;
        chopper->disable_short_to_gnd_protection = false;
        chopper->disable_short_to_vs_protection = false;
    }

    if (stealth_chop != NULL)
    {
        stealth_chop->pwm_ofs = 36U;
        stealth_chop->pwm_grad = 14U;
        stealth_chop->pwm_freq = TMC2209_PWM_FREQ_2_1024;
        stealth_chop->pwm_autoscale = true;
        stealth_chop->pwm_autograd = true;
        stealth_chop->freewheel = TMC2209_FREEWHEEL_NORMAL;
        stealth_chop->pwm_reg = 8U;
        stealth_chop->pwm_lim = 12U;
    }
}

void TMC2209_LoadSpreadCycleDefaults(TMC2209_ChopperConfig *chopper)
{
    if (chopper == NULL)
    {
        return;
    }

    chopper->toff = 5U;
    chopper->hstrt = 0U;
    chopper->hend = 0U;
    chopper->tbl = 2U;
    chopper->high_sensitivity_vsense = false;
    chopper->microsteps = TMC2209_MICROSTEP_256;
    chopper->interpolate_to_256 = true;
    chopper->double_edge_step = false;
    chopper->disable_short_to_gnd_protection = false;
    chopper->disable_short_to_vs_protection = false;
}