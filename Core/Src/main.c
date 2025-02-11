/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  READ = 1,
  WRITE = 0,
} ReadWrite;

typedef enum {
  TYPE_GPIO = 0x00,
  TYPE_PWM = 0x02,
  TYPE_CAN = 0x03,
  TYPE_MISC = 0x04
} PeripheralType;

typedef enum {
  INDEX_PB4 = 0x00,    // LED 1
  INDEX_PB5 = 0x01,    // LED 2
} GPIOIndex;

typedef enum {
  INDEX_PA0 = 0x00,    // PCB 1
  INDEX_PA1 = 0x01,    // PCB 2
  INDEX_PA2 = 0x02,    // PCB 3
  INDEX_PA3 = 0x03,    // PCB 4
  INDEX_PA4 = 0x04,    // PCB 5
  INDEX_PA5 = 0x05,    // PCB 6
  INDEX_PA6 = 0x06,    // PCB 7
  INDEX_PA7 = 0x07,    // PCB 8
  INDEX_PB0 = 0x08,    // PCB 9
  INDEX_PB1 = 0x09,    // PCB 10
  INDEX_PB2 = 0x0A,    // PCB 11
  INDEX_PB10 = 0x0B,   // PCB 12
  INDEX_PA8 = 0x0C,    // PCB 32
  INDEX_PB15 = 0x0D,   // PCB 31
} PWMIndex;

typedef enum {
  INDEX_LDC1 = 0x00,   // Load Cell 1
  INDEX_LDC2 = 0x01,   // Load Cell 2
  INDEX_LDC3 = 0x02,   // Load Cell 3
  INDEX_LDC4 = 0x03,   // Load Cell 4
  INDEX_LDC5 = 0x04,   // Load Cell 5
  INDEX_LDC6 = 0x05,   // Load Cell 6
  INDEX_LDC7 = 0x06,   // Load Cell 7
  INDEX_LDC8 = 0x07,   // Load Cell 8
  INDEX_LDC9 = 0x08,   // Load Cell 9
  INDEX_LDC10 = 0x09,  // Load Cell 10
  INDEX_LDC11 = 0x0A,  // Load Cell 11
  INDEX_LDC12 = 0x0B,  // Load Cell 12
} CANIndex;

typedef enum {
  INDEX_STATUS = 0x00, // STM32 Status
} MISCIndex;

typedef struct {
  uint8_t read_write : 1;  // Read/Write Bit
  uint8_t type : 7;        // Peripheral Type
  uint8_t index;           // Peripheral Index
  uint16_t value;          // Message Value
} RPiMessage;

typedef enum {
  NORMAL = 1,
  I2C_ERROR = 2,
} Status;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RPI_TX_BUFFER_SIZE 4  // bytes
#define RPI_RX_BUFFER_SIZE 4  // bytes

#define STATUS_PERIOD 1000    // milliseconds
#define DATA_INTERVAL 300     // milliseconds
#define BLINK_SPEED   100     // milliseconds

#define MAX_PERIPHERAL_TYPE 8
#define MAX_PERIPHERAL_INDEX 32

#define DEFAULT_DATA_VALUE 0xFFFF

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rpiTask */
osThreadId_t rpiTaskHandle;
const osThreadAttr_t rpiTask_attributes = {
  .name = "rpiTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dataTask */
osThreadId_t dataTaskHandle;
const osThreadAttr_t dataTask_attributes = {
  .name = "dataTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for I2CMutex */
osMutexId_t I2CMutexHandle;
const osMutexAttr_t I2CMutex_attributes = {
  .name = "I2CMutex"
};
/* USER CODE BEGIN PV */

uint16_t data_table[MAX_PERIPHERAL_TYPE][MAX_PERIPHERAL_INDEX];

volatile uint8_t rpi_tx_buffer[RPI_TX_BUFFER_SIZE];
volatile uint8_t rpi_rx_buffer[RPI_RX_BUFFER_SIZE];

RPiMessage rpi_message;

volatile uint16_t adc1_result_dma[6];
volatile uint16_t adc2_result_dma[5];

const int adc1_channel_count = sizeof (adc1_result_dma) / sizeof (adc1_result_dma[0]);
const int adc2_channel_count = sizeof (adc2_result_dma) / sizeof (adc2_result_dma[0]);

volatile int adc1_complete = 0;
volatile int adc2_complete = 0;

uint8_t status = NORMAL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void StartRpiTask(void *argument);
void StartDataTask(void *argument);

/* USER CODE BEGIN PFP */

void ResetI2C(I2C_HandleTypeDef *rev_i2c);

void InitializeData();

void BlinkStatus(uint8_t count, uint16_t delay);
void ParsePRiMessage(RPiMessage *msg);
void ProcessRPiMessage();\

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Re-initialize the I2C communication bus.
  */
void ResetI2C(I2C_HandleTypeDef *rev_i2c)
{
  HAL_I2C_DeInit(rev_i2c);
  HAL_I2C_Init(rev_i2c);
}

/**
  * @brief  Populate data lookup tables with default data.
  */
void InitializeData() {
  // Initialize all entries with 0xFFFF (invalid value)
  for (size_t type = 0; type < MAX_PERIPHERAL_TYPE; type++) {
    for (size_t index = 0; index < MAX_PERIPHERAL_INDEX; index++) {
      data_table[type][index] = DEFAULT_DATA_VALUE;
    }
  }
}

/**
  * @brief  Flash the status LED every 'delay' milliseconds for 'count' times.
  */
void BlinkStatus(uint8_t count, uint16_t delay)
{
  for (int i = 0; i < (count * 2) ; i++)
  {
    // Toggle 'status' LED
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    osDelay(delay);
  }
}

/**
  * @brief  Parse 4-bytes into a RPiMessage.
  */
void ParseRPiMessage(RPiMessage *msg)
{
    msg->read_write = (rpi_rx_buffer[0] >> 7) & 0x01;         // bit 7
    msg->type = rpi_rx_buffer[0] & 0x7F;                      // bits 0-6
    msg->index = rpi_rx_buffer[1];                            // second byte
    msg->value = (rpi_rx_buffer[2] << 8) | rpi_rx_buffer[3];  // third & fourth bytes
}

/**
  * @brief  Logic for handling RPi messages.
  */
void ProcessRPiMessage()
{
  // Read the 'rpi_rx_buffer' into 'rpi_message'
  ParseRPiMessage(&rpi_message);

  uint16_t return_value = DEFAULT_DATA_VALUE;

  // Process 'read' commands
  if (rpi_message.read_write == READ)
  {
    return_value = data_table[rpi_message.type][rpi_message.index];
  }

  // Process 'write' commands
  if (rpi_message.read_write == WRITE)
  {
    data_table[rpi_message.type][rpi_message.index] = rpi_message.value;
    return_value = data_table[rpi_message.type][rpi_message.index];
  }

  // Update the 'tx_buffer'
  rpi_tx_buffer[0] = rpi_rx_buffer[0];            // Echo back peripheral type
  rpi_tx_buffer[1] = rpi_rx_buffer[1];            // Echo back peripheral index
  rpi_tx_buffer[2] = (return_value >> 8) & 0xFF;  // High byte of 'return_value'
  rpi_tx_buffer[3] = return_value & 0xFF;         // Low byte of 'return_value'

  // Respond to RPi
  HAL_I2C_Slave_Transmit_IT(&hi2c1, (uint8_t*)&rpi_tx_buffer, RPI_TX_BUFFER_SIZE);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    adc1_complete = 1;
  }
  if (hadc == &hadc2)
  {
    adc2_complete = 2;
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1)
  {
    ProcessRPiMessage();
  }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1)
  {
    HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&rpi_rx_buffer, RPI_RX_BUFFER_SIZE);
  }
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1)
  {
    status = I2C_ERROR;
    ResetI2C(hi2c);
  }
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of I2CMutex */
  I2CMutexHandle = osMutexNew(&I2CMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of rpiTask */
  rpiTaskHandle = osThreadNew(StartRpiTask, NULL, &rpiTask_attributes);

  /* creation of dataTask */
  dataTaskHandle = osThreadNew(StartDataTask, NULL, &dataTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 5;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 32;
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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    data_table[TYPE_MISC][INDEX_STATUS] = status;
    // Flash 'status' LED
    BlinkStatus(status, BLINK_SPEED);
    osDelay(STATUS_PERIOD);
  }
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRpiTask */
/**
* @brief Function implementing the rpiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRpiTask */
void StartRpiTask(void *argument)
{
  /* USER CODE BEGIN StartRpiTask */
  HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&rpi_rx_buffer, RPI_RX_BUFFER_SIZE);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  osThreadTerminate(NULL);
  /* USER CODE END StartRpiTask */
}

/* USER CODE BEGIN Header_StartDataTask */
/**
* @brief Function implementing the dataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDataTask */
void StartDataTask(void *argument)
{
  /* USER CODE BEGIN StartDataTask */
  InitializeData();
  /* Infinite loop */
  for(;;)
  {
    // Read ADC1
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_result_dma, adc1_channel_count) != HAL_OK)
    {
      Error_Handler();
    }
    // Read ADC2
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_result_dma, adc2_channel_count) != HAL_OK)
    {
      Error_Handler();
    }
    // Wait for DMA to finish
    while (!adc1_complete && !adc2_complete) {}
    adc1_complete = 0;
    adc2_complete = 0;
    // Update Values
    data_table[TYPE_PWM][INDEX_PA0] = adc1_result_dma[0];
    data_table[TYPE_PWM][INDEX_PA1] = adc1_result_dma[1];
    data_table[TYPE_PWM][INDEX_PA2] = adc1_result_dma[2];
    data_table[TYPE_PWM][INDEX_PA3] = adc1_result_dma[3];
    data_table[TYPE_PWM][INDEX_PB0] = adc1_result_dma[4];
    data_table[TYPE_PWM][INDEX_PB1] = adc1_result_dma[5];
    data_table[TYPE_PWM][INDEX_PA4] = adc2_result_dma[0];
    data_table[TYPE_PWM][INDEX_PA5] = adc2_result_dma[1];
    data_table[TYPE_PWM][INDEX_PA6] = adc2_result_dma[2];
    data_table[TYPE_PWM][INDEX_PA7] = adc2_result_dma[3];
    data_table[TYPE_PWM][INDEX_PB2] = adc2_result_dma[4];

    // todo: CAN

    // Toggle LED 0
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, data_table[TYPE_GPIO][INDEX_PB5]);

    osDelay(DATA_INTERVAL);
  }
  osThreadTerminate(NULL);
  /* USER CODE END StartDataTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
