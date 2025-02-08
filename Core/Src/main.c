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
#include <string.h>
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MAX_STRING_LENGTH  50
#define MAX_COMMAND_LENGTH 5

typedef enum {
  READ = 1,
  WRITE = 0,
} ReadWrite;

typedef enum {
  TYPE_GPIO = 0x00,
  TYPE_ADC = 0x01,
  TYPE_DAC = 0x02,
  TYPE_PWM = 0x03,
  TYPE_CAN = 0x04,
  TYPE_MISC = 0x05
} PeripheralType;

typedef enum {
  INDEX_PB4 = 0x00,    // LED 1
  INDEX_PB5 = 0x01,    // LED 2
} GPIOIndex;

typedef enum {
  INDEX_VOUT0 = 0x00,  // PCB 13
  INDEX_VOUT1 = 0x01,  // N.C.
  INDEX_VOUT2 = 0x02,  // PCB 14
  INDEX_VOUT3 = 0x03,  // PCB 17
  INDEX_VOUT4 = 0x04,  // PCB 15
  INDEX_VOUT5 = 0x05,  // PCB 16
  INDEX_VOUT6 = 0x06,  // N.C.
  INDEX_VOUT7 = 0x07,  // N.C.
} DACIndex;

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
  INDEX_HB = 0x00,     // STM32 Heartbeat
} MISCIndex;

typedef struct {
  char command[MAX_COMMAND_LENGTH];
  uint8_t read_write : 1;
  uint8_t type;
  uint8_t index;
  char description[MAX_STRING_LENGTH];
} MessageMapping;

typedef struct {
    uint8_t read_write : 1;  // read/write bit
    uint8_t type : 7;        // peripheral type
    uint8_t index;           // peripheral index
    uint16_t value;          // message value
} I2CMessage;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TX_BUFFER_SIZE 4     // transmit buffer size
#define RX_BUFFER_SIZE 4     // receive buffer size

#define HEARTBEAT_MAX  9999  // max heart beat count

#define LOG_INTERVAL   5000  // heart beat period [ms]
#define DATA_INTERVAL  100   // read task period  [ms]

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* Definitions for logTask */
osThreadId_t logTaskHandle;
const osThreadAttr_t logTask_attributes = {
  .name = "logTask",
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
/* Definitions for rpiTask */
osThreadId_t rpiTaskHandle;
const osThreadAttr_t rpiTask_attributes = {
  .name = "rpiTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DataMutex */
osMutexId_t DataMutexHandle;
const osMutexAttr_t DataMutex_attributes = {
  .name = "DataMutex"
};
/* USER CODE BEGIN PV */

// I2C Message Constants
const MessageMapping message_table[] = {
  { "APA", WRITE, TYPE_DAC,  INDEX_VOUT2, "Auger Pivot Angle"    },
  { "APU", READ,  TYPE_PWM,  INDEX_PB0,   "Auger Pivot Up"       },
  { "APD", READ,  TYPE_PWM,  INDEX_PA4,   "Auger Pivot Down"     },
  { "AFA", WRITE, TYPE_DAC,  INDEX_VOUT5, "Auger Fold Angle"     },
  { "AFU", READ,  TYPE_PWM,  INDEX_PB10,  "Auger Unfold"         },
  { "AFF", READ,  TYPE_PWM,  INDEX_PB2,   "Auger Fold"           },
  { "STU", READ,  TYPE_PWM,  INDEX_PA1,   "Spout Tilt Up"        },
  { "STD", READ,  TYPE_PWM,  INDEX_PA3,   "Spout Tilt Down"      },
  { "SRA", WRITE, TYPE_DAC,  INDEX_VOUT0, "Spout Rotation Angle" },
  { "SRC", READ,  TYPE_PWM,  INDEX_PA6,   "Spout Rotate CW"      },
  { "SRW", READ,  TYPE_PWM,  INDEX_PA7,   "Spout Rotate CCW"     },
  { "GTA", WRITE, TYPE_DAC,  INDEX_VOUT4, "Gate Angle"           },
  { "GTO", READ,  TYPE_PWM,  INDEX_PA5,   "Gate Open"            },
  { "GTC", READ,  TYPE_PWM,  INDEX_PB1,   "Gate Close"           },
  { "PTO", WRITE, TYPE_PWM,  INDEX_PB15,  "PTO Speed"            },
  { "WFL", WRITE, TYPE_CAN,  INDEX_LDC1,  "Weight Front Left"    },
  { "WFR", WRITE, TYPE_CAN,  INDEX_LDC2,  "Weight Front Right"   },
  { "WRL", WRITE, TYPE_CAN,  INDEX_LDC3,  "Weight Rear Left"     },
  { "WRR", WRITE, TYPE_CAN,  INDEX_LDC4,  "Weight Rear Right"    },
  { "WHH", WRITE, TYPE_CAN,  INDEX_LDC5,  "Weight Hitch"         },
  { "TDF", READ,  TYPE_PWM,  INDEX_PA0,   "Tandem Float"         },
  { "TDC", READ,  TYPE_PWM,  INDEX_PA0,   "Tandem Cutoff"        },
  { "WSD", WRITE, TYPE_PWM,  INDEX_PA8,   "Wheel Speed"          },
  { "HBT", READ,  TYPE_MISC, INDEX_HB,    "Heart Beat"           },
};
const size_t MESSAGE_TABLE_SIZE = sizeof(message_table) / sizeof(MessageMapping);

// I2C TX/RX Buffers
volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
I2CMessage i2c_message;
HAL_StatusTypeDef i2c_status;

// UART Messages
char uart_heart_beat[MAX_STRING_LENGTH];
char uart_msg_recv[MAX_STRING_LENGTH];

uint16_t heart_beat_count = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
void StartLogTask(void *argument);
void StartDataTask(void *argument);
void StartRpiTask(void *argument);

/* USER CODE BEGIN PFP */
void ResetI2C(I2C_HandleTypeDef* rev_i2c);
//void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void ProcessI2CData();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ResetI2C(I2C_HandleTypeDef* rev_i2c)
{
  HAL_I2C_DeInit(rev_i2c);
  HAL_I2C_Init(rev_i2c);
}

void parseI2CMessage(I2CMessage *msg) {
    msg->read_write = (rx_buffer[0] & 0x80) >> 7;  // bit 7
    msg->type = (rx_buffer[0] & 0x7F);             // bits 0-6
    msg->index = rx_buffer[1];                     // second byte
    msg->value = (rx_buffer[2] << 8) | rx_buffer[3];    // third & fourth bytes
}

void ProcessI2CMessage()
{
  // Read RX Buffer
  parseI2CMessage(&i2c_message);

  // UART Send
  sprintf(uart_msg_recv, "RX: %02X %02X %02X %02X\n\r", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg_recv, strlen(uart_msg_recv), HAL_MAX_DELAY);

  // Update TX Buffer
  tx_buffer[0] = rx_buffer[0];
  tx_buffer[1] = rx_buffer[1];
  tx_buffer[2] = rx_buffer[2];
  tx_buffer[3] = rx_buffer[3];

  // I2C Send
  HAL_I2C_Slave_Transmit(&hi2c1, (uint8_t*)&tx_buffer, TX_BUFFER_SIZE, HAL_MAX_DELAY); // blocking
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of DataMutex */
  DataMutexHandle = osMutexNew(&DataMutex_attributes);

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
  /* creation of logTask */
  logTaskHandle = osThreadNew(StartLogTask, NULL, &logTask_attributes);

  /* creation of dataTask */
  dataTaskHandle = osThreadNew(StartDataTask, NULL, &dataTask_attributes);

  /* creation of rpiTask */
  rpiTaskHandle = osThreadNew(StartRpiTask, NULL, &rpiTask_attributes);

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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

/* USER CODE BEGIN Header_StartLogTask */
/**
  * @brief  Function implementing the logTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLogTask */
void StartLogTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    // Toggle 'status' LED
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Nucleo
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);  // PCB

    // UART Send
    sprintf(uart_heart_beat, "STM32 Online (%u)\n\r", heart_beat_count);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_heart_beat, strlen(uart_heart_beat), HAL_MAX_DELAY);

    // Update Heart Beat
    heart_beat_count++;
    heart_beat_count = heart_beat_count % HEARTBEAT_MAX;

    osDelay(LOG_INTERVAL);
  }
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(DATA_INTERVAL);
  }
  osThreadTerminate(NULL);
  /* USER CODE END StartDataTask */
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
  /* Infinite loop */
  for(;;)
  {
    i2c_status = HAL_I2C_Slave_Receive(&hi2c1, (uint8_t*)&rx_buffer, RX_BUFFER_SIZE, HAL_MAX_DELAY);  // blocking
    if (i2c_status == HAL_OK) {
      ProcessI2CMessage();
    } else {
      ResetI2C(&hi2c1);
    }
  }
  /* USER CODE END StartRpiTask */
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
