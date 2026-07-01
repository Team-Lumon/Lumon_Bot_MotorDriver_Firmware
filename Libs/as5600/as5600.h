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
#define AS5600_CPR        4096
#define AS5600_WRAP_HALF  (AS5600_CPR / 2)

typedef struct {
  I2C_HandleTypeDef *i2c;
  uint8_t rx[2];
  uint16_t raw;
  uint16_t prev;
  int32_t abs;
  int32_t turns;
  uint8_t ready;
  uint8_t busy;
  uint8_t error;
} AS5600_t;

/**
@brief  Read the magnet status register of the AS5600 encoder.
  * @retval 0-127 : Magnet detected, ideal should be 64
*/
uint8_t AS5600_Status(I2C_HandleTypeDef *i2c);

/**
@brief  Read the configuration register of the AS5600 encoder.
  * @retval The value of the configuration register.
*/
uint16_t AS5600_Conf(I2C_HandleTypeDef *i2c);

uint8_t AS5600_Agc(I2C_HandleTypeDef *i2c);
uint16_t AS5600_Mag(I2C_HandleTypeDef *i2c);
uint16_t AS5600_RawRead(I2C_HandleTypeDef *i2c);
void AS5600_Init(AS5600_t *enc, I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef AS5600_Read(AS5600_t *enc);
void AS5600_Done(AS5600_t *enc, I2C_HandleTypeDef *i2c);
void AS5600_Fail(AS5600_t *enc, I2C_HandleTypeDef *i2c);
uint16_t AS5600_Raw(const AS5600_t *enc);
int32_t AS5600_Abs(const AS5600_t *enc);
uint8_t AS5600_Ready(const AS5600_t *enc);
uint8_t AS5600_Busy(const AS5600_t *enc);
uint8_t AS5600_Error(const AS5600_t *enc);

#endif
