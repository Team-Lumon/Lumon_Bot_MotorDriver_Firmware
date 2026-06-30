#ifndef AS5600_H
#define AS5600_H

#include "main.h"
#include <stdint.h>
#include "stm32g0xx_hal.h"

#define AS5600_ADDR       (0x36 << 1)
#define AS5600_CONF       0x07
#define AS5600_STATUS     0x0B
#define AS5600_RAW_ANGLE  0x0C
#define AS5600_ANGLE      0x0E
#define AS5600_AGC        0x1A
#define AS5600_MAGNITUDE 0x1B

/**
@brief  Read the magnet status register of the AS5600 encoder.
  * @retval 0-127 : Magnet detected, ideal should be 64
*/
uint8_t AS5600_ReadStatus(I2C_HandleTypeDef *encoder_i2c);

/**
@brief  Read the configuration register of the AS5600 encoder.
  * @retval The value of the configuration register.
*/
uint16_t AS5600_ReadConf(I2C_HandleTypeDef *encoder_i2c);

uint16_t Encoder_ReadAnalog(I2C_HandleTypeDef *encoder_i2c);
uint8_t AS5600_ReadAgc(I2C_HandleTypeDef *encoder_i2c);
uint16_t AS5600_ReadMagnitude(I2C_HandleTypeDef *encoder_i2c);
uint16_t AS5600_ReadRawAngle(I2C_HandleTypeDef *encoder_i2c);

#endif
