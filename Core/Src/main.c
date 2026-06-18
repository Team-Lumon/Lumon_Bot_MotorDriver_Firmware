/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_bus.h"
#include "debug_helper.h"
#include "stm32g0xx_hal_gpio.h"
#include "tmc2209.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TMC2209_SLAVE_ADDR  0x00
#define STEPPER_PULSE_WIDTH_MS  1U
#define STEPPER_STEP_DELAY_MS   2U
#define STEPPER_DIRECTION GPIO_PIN_SET

#define AS5600_ADDR       (0x36 << 1)
#define AS5600_CONF       0x07
#define AS5600_STATUS     0x0B
#define AS5600_RAW_ANGLE  0x0C
#define AS5600_ANGLE      0x0E
#define AS5600_AGC        0x1A
#define AS5600_MAGNITUDE  0x1B
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RUN_EVERY(interval, counter, func) \
  do { \
    (counter)++; \
    if ((counter) >= (interval)) { \
      (counter) = 0; \
      func(); \
    } \
  } while (0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint32_t LED_counter;
static uint32_t CAN_counter;
static volatile uint8_t can_send_pending;
static volatile uint32_t absolute_position;

static TMC2209_HandleTypeDef tmc = {0};




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Stepper_InitPins(void);
static void Stepper_Pulse(void);
static void PrintCanMessage(const char *prefix, const CAN_BusMessage_t *message);
static uint16_t Encoder_ReadAnalog(void);
static uint8_t AS5600_ReadRegister8(uint8_t reg);
static uint16_t AS5600_ReadRegister16(uint8_t reg);
static uint16_t AS5600_ReadConf(void);
static uint8_t AS5600_ReadStatus(void);
static uint8_t AS5600_ReadAgc(void);
static uint16_t AS5600_ReadMagnitude(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t id = 16;

int _write(int file, char *ptr, int len) {
  // Retarget printf to USART2
  HAL_UART_Transmit(&Debug_Uart, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}
void LED_Toggle(void) {
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
uint32_t canValue = 0;
void REQUEST_SEND_CAN(void) {
  can_send_pending = 1U;
}

void SEND_CAN(void) {
  if(CAN_Bus_SendU32(&CAN, CAN_ID_DEBUG, canValue) != HAL_OK) {
    FDCAN_ProtocolStatusTypeDef protocol_status = {0};
    (void)HAL_FDCAN_GetProtocolStatus(&CAN, &protocol_status);
    printf("Failed to send CAN message err=0x%08lX lec=%lu bus_off=%lu\r\n",
           (unsigned long)HAL_FDCAN_GetError(&CAN),
           (unsigned long)protocol_status.LastErrorCode,
           (unsigned long)protocol_status.BusOff);
  } else {
    printf("Sent CAN message with value: %lu\r\n", canValue);
    canValue++;
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    RUN_EVERY(1000, LED_counter, LED_Toggle);
    if(id == 0) {
      RUN_EVERY(1000, CAN_counter, REQUEST_SEND_CAN);
    }

  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *fdcan_handle, uint32_t RxFifo0ITs)
{
    if ((fdcan_handle == &CAN) && ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)) {
      CAN_BusMessage_t message = {0};

      if (CAN_Bus_Receive(&CAN, &message) == HAL_OK) {
        PrintCanMessage("CAN RX", &message);

        switch (message.id) {
          case CAN_ID_ADC_REPORT:
            uint32_t absolute_position = CAN_Bus_ReadU32(&message);
            printf(absolute_position ? "ADC value: %lu\n" : "Failed to read ADC value\n", absolute_position);
            break;
          case CAN_ID_STATUS: {
            char status = CAN_Bus_ReadU8(&message);
            switch (status) {
              case 'R':
                printf("Resetting ....");
                HAL_NVIC_SystemReset();
                break;
              default:
                break;
            }
            break;
          }
          case CAN_ID_DEBUG:
            printf("Debug value: %lu\r\n", (unsigned long)CAN_Bus_ReadU32(&message));
            break;
          default:
            printf("Received message with unhandled ID: 0x%03lX\r\n", (unsigned long)message.id);
            break;
        }
      }
    }
}

static void PrintCanMessage(const char *prefix, const CAN_BusMessage_t *message) {
  if ((prefix == NULL) || (message == NULL)) {
    return;
  }

  printf("%s id=0x%03lX dlc=%u data:",
         prefix,
         (unsigned long)message->id,
         message->dlc);

  for (uint8_t i = 0; i < message->dlc; i++) {
    printf(" %02X", message->data[i]);
  }

  if (message->dlc >= 4U) {
    printf(" value_u32=%lu", (unsigned long)CAN_Bus_ReadU32(message));
  } else if (message->dlc >= 2U) {
    printf(" value_u16=%u", CAN_Bus_ReadU16(message));
  } else if (message->dlc >= 1U) {
    printf(" value_u8=%u", CAN_Bus_ReadU8(message));
  }

  printf("\r\n");
}

static void PrintHexBytes(const char *label, const uint8_t *data, uint8_t len) {
  printf("%s[%u]:", label, len);
  for (uint8_t i = 0; i < len; i++) {
    printf(" %02X", data[i]);
  }
  printf("\r\n");
}

static void PrintTmcUartDebug(const TMC2209_UartDebugInfo *debug) {
  printf("TMC DBG reg=0x%02X rx_status=%d read_status=%d reply=%u ofs=%u\r\n",
         debug->last_reg,
         debug->last_receive_status,
         debug->last_read_status,
         debug->last_reply_found ? 1 : 0,
         debug->last_reply_offset);
  PrintHexBytes("TMC TX", debug->last_tx, debug->last_tx_len);
  PrintHexBytes("TMC RX", debug->last_rx, debug->last_rx_len);
}

static void TMC2209_ConfigUartHandle(void) {
  tmc.huart = &tmcUart;
  tmc.enn_port = Driver_disable_GPIO_Port;
  tmc.enn_pin = Driver_disable_Pin;
  tmc.slave_addr = TMC2209_SLAVE_ADDR;
}

static void Stepper_InitPins(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_WritePin(step_GPIO_Port, step_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(step_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);
}

static void Stepper_Pulse(void) {
  HAL_GPIO_WritePin(step_GPIO_Port, step_Pin, GPIO_PIN_SET);
  HAL_Delay(STEPPER_PULSE_WIDTH_MS);
  HAL_GPIO_WritePin(step_GPIO_Port, step_Pin, GPIO_PIN_RESET);
  HAL_Delay(STEPPER_STEP_DELAY_MS);
}

static uint16_t Encoder_ReadAnalog(void) {
  uint16_t adc_value = 0U;

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK) {
    adc_value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);

  return adc_value;
}

static uint8_t AS5600_ReadRegister8(uint8_t reg) {
  uint8_t data = 0xFFU;

  if (HAL_I2C_Mem_Read(&encoder_i2c,
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

static uint16_t AS5600_ReadRegister16(uint8_t reg) {
  uint8_t data[2] = {0xFFU, 0xFFU};

  if (HAL_I2C_Mem_Read(&encoder_i2c,
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

static uint16_t AS5600_ReadConf(void) {
  return AS5600_ReadRegister16(AS5600_CONF);
}

static uint8_t AS5600_ReadStatus(void) {
  return AS5600_ReadRegister8(AS5600_STATUS);
}

static uint8_t AS5600_ReadAgc(void) {
  return AS5600_ReadRegister8(AS5600_AGC);
}

static uint16_t AS5600_ReadMagnitude(void) {
  return AS5600_ReadRegister16(AS5600_MAGNITUDE);
}

uint16_t AS5600_ReadRawAngle(void)
{
    return AS5600_ReadRegister16(AS5600_RAW_ANGLE);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  id = (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) << 3) |
       (HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) << 2) |
       (HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) << 1) |
       (HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin));

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  Stepper_InitPins();

  printf("\n####################### SYSTEM DETAILS ########################\n");
  uint32_t sys_clock_hz = HAL_RCC_GetSysClockFreq();
  uint32_t sys_clock_mhz = sys_clock_hz / 1000000U;
  uint32_t sys_clock_mhz_fraction =
      ((sys_clock_hz % 1000000U) * 100U + 500000U) / 1000000U;

  printf("Sys clock: %lu.%02lu MHz\n",
         (unsigned long)sys_clock_mhz,
         (unsigned long)sys_clock_mhz_fraction);
  PrintTimerFrequency("TIM2(step_timer)", &htim2, 1U);
  // PrintTimerFrequency("TIM3(ms_Timer)", &htim3, 1U);
  printf("#################################################################\n");
  HAL_Delay(500);

  printf("\n####################### SYSTEM INIT ########################\n");
  printf("Controller ID : ");
  printf("%01X\n", id);

  printf("ms timer init : ");
  printf(HAL_TIM_Base_Start_IT(&ms_timer) ? "Failed\n" : "Success\n");
  
  printf("CAN init : ");
  printf(CAN_Bus_Init(&CAN) ? "Failed\n" : "Success\n");

  printf("User UART init : ");
  // printf(HAL_UART_Receive_IT(&huart2, &rx, 1) ? "Failed\n" : "Success\n");

  TMC2209_ConfigUartHandle();
  printf("TMC UART init : ");
  printf(TMC2209_Init(&tmc) ? "Failed\n" : "Success\n");

  printf("TMC send delay : ");
  printf(TMC2209_SetSendDelay(&tmc, 8) ? "Failed\n" : "Success\n");

  uint32_t ioin = 0;
  printf("TMC Check Connection : ");
  printf(TMC2209_CheckConnection(&tmc, &ioin) ? "Failed\n" : "Success\n");

  printf("\n############################################################\n\n");
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Driver_disable_GPIO_Port, Driver_disable_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, STEPPER_DIRECTION);
  HAL_Delay(1);
    
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // printf("System initializing... can_send_pending=%u\n", can_send_pending);
    if (can_send_pending != 0U) {
      can_send_pending = 0U;
      SEND_CAN();
    }

    // uint16_t analog_angle = Encoder_ReadAnalog();
    // uint16_t i2c_angle = AS5600_ReadRawAngle();
    // uint16_t conf = AS5600_ReadConf();
    // uint8_t status = AS5600_ReadStatus();
    // uint8_t agc = AS5600_ReadAgc();
    // uint16_t magnitude = AS5600_ReadMagnitude();

    // printf("analog : %u\t", analog_angle);
    // printf("I2c : %u\t", i2c_angle);
    // printf("CONF : 0x%04X\t", conf);
    // printf("STATUS : 0x%02X\t", status);
    // printf("AGC : %u\t", agc);
    // printf("MAG : %u\n", magnitude);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Stepper_Pulse();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10B17DB5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DIR_Pin|MS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MS1_Pin|Driver_disable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : S1_Pin */
  GPIO_InitStruct.Pin = S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(S1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S2_Pin S3_Pin S4_Pin */
  GPIO_InitStruct.Pin = S2_Pin|S3_Pin|S4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_Pin */
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Diagnose_Pin */
  GPIO_InitStruct.Pin = Diagnose_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Diagnose_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MS2_Pin */
  GPIO_InitStruct.Pin = MS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MS1_Pin Driver_disable_Pin */
  GPIO_InitStruct.Pin = MS1_Pin|Driver_disable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
