#include "as5600.h"

static uint8_t AS5600_ReadRegister8(I2C_HandleTypeDef *encoder_i2c, uint8_t reg) {
  uint8_t data = 0xFFU;

  if (HAL_I2C_Mem_Read(encoder_i2c,
                       AS5600_ADDR,
                       reg,
                       I2C_MEMADD_SIZE_8BIT,
                       &data,
                       1,
                       100) != HAL_OK) {
    return 0xFFU;
  }

  return data;
}

static uint16_t AS5600_ReadRegister16(I2C_HandleTypeDef *encoder_i2c, uint8_t reg) {
  uint8_t data[2] = {0xFFU, 0xFFU};

  if (HAL_I2C_Mem_Read(encoder_i2c,
                       AS5600_ADDR,
                       reg,
                       I2C_MEMADD_SIZE_8BIT,
                       data,
                       2,
                       100) != HAL_OK) {
    return 0xFFFFU;
  }

  return ((uint16_t)data[0] << 8) | data[1];
}

uint16_t AS5600_ReadConf(I2C_HandleTypeDef *encoder_i2c) {
  return AS5600_ReadRegister16(encoder_i2c, AS5600_CONF);
}

uint8_t AS5600_ReadStatus(I2C_HandleTypeDef *encoder_i2c) {
  return AS5600_ReadRegister8(encoder_i2c, AS5600_STATUS);
}

uint8_t AS5600_ReadAgc(I2C_HandleTypeDef *encoder_i2c) {
  return AS5600_ReadRegister8(encoder_i2c, AS5600_AGC);
}

uint16_t AS5600_ReadMagnitude(I2C_HandleTypeDef *encoder_i2c) {
  return AS5600_ReadRegister16(encoder_i2c, AS5600_MAGNITUDE);
}

uint16_t AS5600_ReadRawAngle(I2C_HandleTypeDef *encoder_i2c)
{
    return AS5600_ReadRegister16(encoder_i2c, AS5600_RAW_ANGLE);
}
