#include "as5600.h"

static uint8_t AS5600_Reg8(I2C_HandleTypeDef *i2c, uint8_t reg) {
  uint8_t data = 0xFFU;

  if (HAL_I2C_Mem_Read(i2c,
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

static uint16_t AS5600_Reg16(I2C_HandleTypeDef *i2c, uint8_t reg) {
  uint8_t data[2] = {0xFFU, 0xFFU};

  if (HAL_I2C_Mem_Read(i2c,
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

static uint16_t AS5600_Norm(uint16_t raw) {
  return raw & 0x0FFFU;
}

static int32_t AS5600_UpdateAbs(AS5600 *enc, uint16_t raw) {
  uint16_t norm = AS5600_Norm(raw);

  if (enc->ready == 0U) {
    enc->raw = norm;
    enc->prev = norm;
    enc->abs = (int32_t)norm;
    enc->turns = 0;
    enc->ready = 1U;
    return enc->abs;
  }

  int32_t delta = (int32_t)norm - (int32_t)enc->prev;

  if (delta > AS5600_WRAP_HALF) {
    enc->turns--;
  } else if (delta < -AS5600_WRAP_HALF) {
    enc->turns++;
  }

  enc->raw = norm;
  enc->prev = norm;
  enc->abs = (enc->turns * AS5600_CPR) + (int32_t)norm;

  return enc->abs;
}

uint16_t AS5600_Conf(I2C_HandleTypeDef *i2c) {
  return AS5600_Reg16(i2c, AS5600_CONF);
}

uint8_t AS5600_Status(I2C_HandleTypeDef *i2c) {
  return AS5600_Reg8(i2c, AS5600_STATUS);
}

uint8_t AS5600_Agc(I2C_HandleTypeDef *i2c) {
  return AS5600_Reg8(i2c, AS5600_AGC);
}

uint16_t AS5600_Mag(I2C_HandleTypeDef *i2c) {
  return AS5600_Reg16(i2c, AS5600_MAGNITUDE);
}

uint16_t AS5600_RawRead(I2C_HandleTypeDef *i2c) {
  return AS5600_Norm(AS5600_Reg16(i2c, AS5600_RAW_ANGLE));
}

void AS5600_Init(AS5600 *enc, I2C_HandleTypeDef *i2c) {
  if (enc == NULL) {
    return;
  }

  enc->i2c = i2c;
  enc->rx[0] = 0U;
  enc->rx[1] = 0U;
  enc->raw = 0U;
  enc->prev = 0U;
  enc->abs = 0;
  enc->turns = 0;
  enc->ready = 0U;
  enc->busy = 0U;
  enc->error = 0U;
}

HAL_StatusTypeDef AS5600_Read(AS5600 *enc) {
  if ((enc == NULL) || (enc->i2c == NULL) || (enc->busy != 0U)) {
    return HAL_BUSY;
  }

  enc->busy = 1U;
  enc->error = 0U;

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(enc->i2c,
                                                  AS5600_ADDR,
                                                  AS5600_RAW_ANGLE,
                                                  I2C_MEMADD_SIZE_8BIT,
                                                  enc->rx,
                                                  2);

  if (status != HAL_OK) {
    enc->busy = 0U;
    enc->error = 1U;
  }

  return status;
}

void AS5600_Done(AS5600 *enc, I2C_HandleTypeDef *i2c) {
  if ((enc == NULL) || (i2c != enc->i2c)) {
    return;
  }

  uint16_t raw = ((uint16_t)enc->rx[0] << 8) | enc->rx[1];
  (void)AS5600_UpdateAbs(enc, raw);
  enc->busy = 0U;
  enc->error = 0U;
}

void AS5600_Fail(AS5600 *enc, I2C_HandleTypeDef *i2c) {
  if ((enc == NULL) || (i2c != enc->i2c)) {
    return;
  }

  enc->busy = 0U;
  enc->error = 1U;
}

uint16_t AS5600_Raw(const AS5600 *enc) {
  if (enc == NULL) {
    return 0U;
  }

  return enc->raw;
}

int32_t AS5600_Abs(const AS5600 *enc) {
  if (enc == NULL) {
    return 0;
  }

  return enc->abs;
}

uint8_t AS5600_Ready(const AS5600 *enc) {
  return (enc != NULL) ? enc->ready : 0U;
}

uint8_t AS5600_Busy(const AS5600 *enc) {
  return (enc != NULL) ? enc->busy : 0U;
}

uint8_t AS5600_Error(const AS5600 *enc) {
  return (enc != NULL) ? enc->error : 1U;
}
