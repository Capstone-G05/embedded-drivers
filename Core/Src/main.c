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
  uint8_t read_write : 1;
  uint8_t type : 7;
  uint8_t index : 8;
  uint16_t value : 16;
} RPiMessage;

typedef struct {
  uint8_t priority : 3;
  uint8_t reserved : 1;
  uint8_t data_page : 1;
  uint8_t pdu_format : 8;
  uint8_t pdu_specific : 8;
  uint8_t source_address : 8;
  uint32_t pgn : 18;
  uint8_t data[8];
} J1939Message;

typedef enum {
  NORMAL = 1,
  I2C_ERROR = 2,
  CAN_ERROR = 3,
  OS_ERROR = 4,
} Status;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define I2C_TX_BUFFER_SIZE 4  // bytes
#define I2C_RX_BUFFER_SIZE 4  // bytes
#define CAN_TX_BUFFER_SIZE 8  // bytes
#define CAN_RX_BUFFER_SIZE 8  // bytes

#define STATUS_PERIOD 1000    // milliseconds
#define DATA_INTERVAL 300     // milliseconds
#define BLINK_SPEED   100     // milliseconds

#define MAX_PERIPHERAL_TYPE  4
#define MAX_PERIPHERAL_INDEX 14

#define DEFAULT_DATA_VALUE 0xFFFF

#define FALCON_ADDRESS       0x31
#define MERLIN_EXCITATION    0x02
#define MERLIN_RESOLUTION    0x02
#define MERLIN_SAMPLING_RATE 0x06
#define MERLIN_FILETER_WIDTH 0x01
#define MERLIN_GAIN       0x??

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* Definitions for loggingTask */
osThreadId_t loggingTaskHandle;
const osThreadAttr_t loggingTask_attributes = {
  .name = "loggingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for pollingTask */
osThreadId_t pollingTaskHandle;
const osThreadAttr_t pollingTask_attributes = {
  .name = "pollingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for realTimeTask */
osThreadId_t realTimeTaskHandle;
const osThreadAttr_t realTimeTask_attributes = {
  .name = "realTimeTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE BEGIN PV */

osMessageQueueId_t canQueueHandle;
const osMessageQueueAttr_t canQueue_attributes = {
  .name = "canQueue"
};

uint16_t data_table[MAX_PERIPHERAL_TYPE][MAX_PERIPHERAL_INDEX];

uint8_t i2c_tx_buffer[I2C_TX_BUFFER_SIZE];
uint8_t i2c_rx_buffer[I2C_RX_BUFFER_SIZE];

RPiMessage rpi_message;

volatile uint16_t adc1_result_dma[6];
volatile uint16_t adc2_result_dma[5];

const int adc1_channel_count = sizeof (adc1_result_dma) / sizeof (adc1_result_dma[0]);
const int adc2_channel_count = sizeof (adc2_result_dma) / sizeof (adc2_result_dma[0]);

volatile int adc1_complete = 0;
volatile int adc2_complete = 0;

volatile uint8_t can_transport_active = 0;

//CAN_RxHeaderTypeDef can_rx_header;
CAN_TxHeaderTypeDef can_tx_header;

//uint8_t can_rx_buffer[CAN_RX_BUFFER_SIZE];
uint8_t can_tx_buffer[CAN_TX_BUFFER_SIZE];

uint32_t can_tx_mailbox;

//J1939Message can_message;

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
static void MX_CAN_Init(void);
void StartLoggingTask(void *argument);
void StartPollingTask(void *argument);
void StartRealTimeTask(void *argument);

/* USER CODE BEGIN PFP */

void ResetI2C(I2C_HandleTypeDef *rev_i2c);

void InitializeData();

void BlinkStatus(uint8_t count, uint16_t delay);

void ParseRPiMessage(uint8_t *i2c_data, RPiMessage *msg);
void ParseJ1939Message(uint32_t can_id, uint8_t *can_data, J1939Message *msg);

void ProcessRPiMessage();
void ProcessJ1939Message(J1939Message j1939_message);

void StartMerlinData(uint8_t address);
void SendMerlinData(uint8_t address);
void SendMerlinConfig(uint8_t address);
void ClaimJ1939Address(uint8_t address);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ResetI2C(I2C_HandleTypeDef *rev_i2c)
{
  HAL_I2C_DeInit(rev_i2c);
  HAL_I2C_Init(rev_i2c);
}

void InitializeData() {
  // Initialize all entries with 0xFFFF (invalid value)
  for (size_t type = 0; type < MAX_PERIPHERAL_TYPE; type++) {
    for (size_t index = 0; index < MAX_PERIPHERAL_INDEX; index++) {
      data_table[type][index] = DEFAULT_DATA_VALUE;
    }
  }
}

void BlinkStatus(uint8_t count, uint16_t delay)
{
  for (int i = 0; i < (count * 2) ; i++)
  {
    // Toggle 'status' LED
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    osDelay(delay);
  }
}

void ParseRPiMessage(uint8_t *i2c_data, RPiMessage *msg)
{
  msg->read_write = (i2c_data[0] >> 7) & 0x01;
  msg->type = i2c_data[0] & 0x7F;
  msg->index = i2c_data[1];
  msg->value = (i2c_data[2] << 8) | i2c_data[3];
}

void ParseJ1939Message(uint32_t can_id, uint8_t *can_data, J1939Message *msg)
{
  msg->priority = (can_id >> 26) & 0x07;
  msg->reserved = (can_id >> 25) & 0x01;
  msg->data_page = (can_id >> 24) & 0x01;
  msg->pdu_format = (can_id >> 16) & 0xFF;
  msg->pdu_specific = (can_id >> 8) & 0xFF;
  msg->source_address = can_id & 0xFF;
  msg->pgn = (can_id >> 8) & 0x3FFFF;
  memcpy(msg->data, can_data, 8);
}

/**
  * @brief  Logic for handling RPi messages.
  */
void ProcessRPiMessage()
{
  ParseRPiMessage(i2c_rx_buffer, &rpi_message);
  uint16_t return_value = DEFAULT_DATA_VALUE;

  if (rpi_message.read_write == READ)
  {
    return_value = data_table[rpi_message.type][rpi_message.index];
  }

  if (rpi_message.read_write == WRITE)
  {
    data_table[rpi_message.type][rpi_message.index] = rpi_message.value;
    return_value = data_table[rpi_message.type][rpi_message.index];
  }

  i2c_tx_buffer[0] = i2c_rx_buffer[0];
  i2c_tx_buffer[1] = i2c_rx_buffer[1];
  i2c_tx_buffer[2] = (return_value >> 8) & 0xFF;
  i2c_tx_buffer[3] = return_value & 0xFF;
  HAL_I2C_Slave_Transmit_IT(&hi2c1, (uint8_t*)&i2c_tx_buffer, I2C_TX_BUFFER_SIZE);
}

void ProcessJ1939Message(J1939Message j1939_message)
{
  // Device Addressing
  if (j1939_message.pgn == 0xFED8)
  {
    // TODO: handle device addressing
  }
  // Sensor Configuration
  else if (j1939_message.pdu_format == 0xEF)
  {
    // TODO: process and save configuration options
    SendMerlinConfig(j1939_message.pdu_specific);
  }
  // Transport Protocol
  else if (j1939_message.pdu_format == 0xEC)
  {
    if (j1939_message.data[0] == 0x11) // start
    {
      SendMerlinData(j1939_message.pdu_specific);
    }
    if (j1939_message.data[0] == 0x13) // stop
    {
      // ignore
    }
  }
  // Merlin Sensor Reading
  else if (j1939_message.pdu_format == 0xEA)
  {
    StartMerlinData(j1939_message.pdu_specific);
  }
}

void StartMerlinData(uint8_t address)
{
  can_tx_header.ExtId = 0x1CEC0000 | (FALCON_ADDRESS << 8) | address;
  can_tx_header.IDE = CAN_ID_EXT;
  can_tx_header.RTR = CAN_RTR_DATA;
  can_tx_header.DLC = CAN_RX_BUFFER_SIZE;
  can_tx_header.TransmitGlobalTime = DISABLE;

  can_tx_buffer[0] = 0x10;
  can_tx_buffer[1] = 0x16;
  can_tx_buffer[2] = 0x00;
  can_tx_buffer[3] = 0x04;
  can_tx_buffer[4] = 0xFF;
  can_tx_buffer[5] = 0x80;
  can_tx_buffer[6] = 0xFF;
  can_tx_buffer[7] = 0x00;

  if (HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_buffer, &can_tx_mailbox) != HAL_OK)
  {
    status = CAN_ERROR;
  }
}

void SendMerlinData(uint8_t address)
{
  uint8_t offset = (address - 0xB5) * 4;
  int32_t weight_1 = (int32_t)data_table[TYPE_CAN][INDEX_LDC1 + offset];
  int32_t weight_2 = (int32_t)data_table[TYPE_CAN][INDEX_LDC2 + offset];
  int32_t weight_3 = (int32_t)data_table[TYPE_CAN][INDEX_LDC3 + offset];
  int32_t weight_4 = (int32_t)data_table[TYPE_CAN][INDEX_LDC4 + offset];

  // gain * excitation = 15,000,000
  uint8_t gain = 100;
  int32_t excitation_voltage = 150000;

  can_tx_header.ExtId = 0x1CEB0000 | (FALCON_ADDRESS << 8) | address;
  can_tx_header.IDE = CAN_ID_EXT;
  can_tx_header.RTR = CAN_RTR_DATA;
  can_tx_header.DLC = CAN_RX_BUFFER_SIZE;
  can_tx_header.TransmitGlobalTime = DISABLE;

  can_tx_buffer[0] = 0x01;
  can_tx_buffer[1] = (MERLIN_RESOLUTION << 4) | MERLIN_EXCITATION;
  can_tx_buffer[2] = gain;
  can_tx_buffer[3] = excitation_voltage & 0xFF;
  can_tx_buffer[4] = (excitation_voltage >> 8) & 0xFF;
  can_tx_buffer[5] = (excitation_voltage >> 16) & 0xFF;
  can_tx_buffer[6] = (excitation_voltage >> 24) & 0xFF;
  can_tx_buffer[7] = weight_1 & 0xFF;

  if (HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_buffer, &can_tx_mailbox) != HAL_OK)
  {
    status = CAN_ERROR;
  }

  can_tx_buffer[0] = 0x02;
  can_tx_buffer[1] = (weight_1 >> 8) & 0xFF;
  can_tx_buffer[2] = (weight_1 >> 16) & 0xFF;
  can_tx_buffer[3] = (weight_1 >> 24) & 0xFF;
  can_tx_buffer[4] = weight_2 & 0xFF;
  can_tx_buffer[5] = (weight_2 >> 8) & 0xFF;
  can_tx_buffer[6] = (weight_2 >> 16) & 0xFF;
  can_tx_buffer[7] = (weight_2 >> 24) & 0xFF;

  while (HAL_CAN_IsTxMessagePending(&hcan, can_tx_mailbox) == SET) {}
  if (HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_buffer, &can_tx_mailbox) != HAL_OK)
  {
    status = CAN_ERROR;
  }

  can_tx_buffer[0] = 0x03;
  can_tx_buffer[1] = weight_3 & 0xFF;
  can_tx_buffer[2] = (weight_3 >> 8) & 0xFF;
  can_tx_buffer[3] = (weight_3 >> 16) & 0xFF;
  can_tx_buffer[4] = (weight_3 >> 24) & 0xFF;
  can_tx_buffer[5] = weight_4 & 0xFF;
  can_tx_buffer[6] = (weight_4 >> 8) & 0xFF;
  can_tx_buffer[7] = (weight_4 >> 16) & 0xFF;

  while (HAL_CAN_IsTxMessagePending(&hcan, can_tx_mailbox) == SET) {}
  if (HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_buffer, &can_tx_mailbox) != HAL_OK)
  {
    status = CAN_ERROR;
  }

  can_tx_buffer[0] = 0x04;
  can_tx_buffer[1] = (weight_4 >> 24) & 0xFF;
  can_tx_buffer[2] = 0xFF;
  can_tx_buffer[3] = 0xFF;
  can_tx_buffer[4] = 0xFF;
  can_tx_buffer[5] = 0xFF;
  can_tx_buffer[6] = 0xFF;
  can_tx_buffer[7] = 0xFF;

  while (HAL_CAN_IsTxMessagePending(&hcan, can_tx_mailbox) == SET) {}
  if (HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_buffer, &can_tx_mailbox) != HAL_OK)
  {
    status = CAN_ERROR;
  }
}

void SendMerlinConfig(uint8_t address)
{
  can_tx_header.ExtId = 0x18EF0000 | (FALCON_ADDRESS << 8) | address;
  can_tx_header.IDE = CAN_ID_EXT;
  can_tx_header.RTR = CAN_RTR_DATA;
  can_tx_header.DLC = CAN_RX_BUFFER_SIZE;
  can_tx_header.TransmitGlobalTime = DISABLE;

  can_tx_buffer[0] = 0x01; // must be set to 1
  can_tx_buffer[1] = 0x0F; // response (9-12)
  can_tx_buffer[2] = 0xFF; // unused
  can_tx_buffer[3] = 0xFF; // unused
  can_tx_buffer[4] = 0xFF; // unused
  can_tx_buffer[5] = 0xFF; // unused
  can_tx_buffer[6] = 0xFF; // unused
  can_tx_buffer[7] = 0xFF; // unused

  if (HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_buffer, &can_tx_mailbox) != HAL_OK)
  {
    status = CAN_ERROR;
  }
}

void ClaimJ1939Address(uint8_t address)
{
  can_tx_header.ExtId = 0x18EEFF00 | (FALCON_ADDRESS << 8) | address;
  can_tx_header.IDE = CAN_ID_EXT;
  can_tx_header.RTR = CAN_RTR_DATA;
  can_tx_header.DLC = CAN_RX_BUFFER_SIZE;
  can_tx_header.TransmitGlobalTime = DISABLE;

  can_tx_buffer[0] = 0xFE;
  can_tx_buffer[1] = 0x2A;
  can_tx_buffer[2] = 0x60;
  can_tx_buffer[3] = 0x49;
  can_tx_buffer[4] = 0x00;
  can_tx_buffer[5] = 0x87;
  can_tx_buffer[6] = 0x22;
  can_tx_buffer[7] = 0xA0;

  if (HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_buffer, &can_tx_mailbox) != HAL_OK)
  {
    status = CAN_ERROR;
  }

  // beyond this... I'm not too sure what's going on

//  can_tx_header.ExtId = 0x18E80000 | (FALCON_ADDRESS << 8) | address;
//
//  can_tx_buffer[0] = 0x01;
//  can_tx_buffer[1] = 0x00;
//  can_tx_buffer[2] = 0xFF;
//  can_tx_buffer[3] = 0xFF;
//  can_tx_buffer[4] = 0xFF;
//  can_tx_buffer[5] = 0x80;
//  can_tx_buffer[6] = 0xFF;
//  can_tx_buffer[7] = 0x00;
//
//  osDelay(250);
//  while (HAL_CAN_IsTxMessagePending(&hcan, can_tx_mailbox) == SET) {}
//  if (HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_buffer, &can_tx_mailbox) != HAL_OK)
//  {
//    status = CAN_ERROR;
//  }
//
//  osDelay(250);
//  while (HAL_CAN_IsTxMessagePending(&hcan, can_tx_mailbox) == SET) {}
//  if (HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_buffer, &can_tx_mailbox) != HAL_OK)
//  {
//    status = CAN_ERROR;
//  }
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
    HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&i2c_rx_buffer, I2C_RX_BUFFER_SIZE);
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef can_rx_header;
  uint8_t can_rx_buffer[CAN_RX_BUFFER_SIZE];
  J1939Message can_message;

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_buffer) != HAL_OK)
  {
    status = CAN_ERROR;
  }

  ParseJ1939Message(can_rx_header.ExtId, can_rx_buffer, &can_message);

  if (osMessageQueuePut(canQueueHandle, &can_message, 0, 0) != osOK)
  {
    status = OS_ERROR;
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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

  canQueueHandle = osMessageQueueNew (16, sizeof(J1939Message), &canQueue_attributes);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of loggingTask */
  loggingTaskHandle = osThreadNew(StartLoggingTask, NULL, &loggingTask_attributes);

  /* creation of pollingTask */
  pollingTaskHandle = osThreadNew(StartPollingTask, NULL, &pollingTask_attributes);

  /* creation of realTimeTask */
  realTimeTaskHandle = osThreadNew(StartRealTimeTask, NULL, &realTimeTask_attributes);

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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  CAN_FilterTypeDef can_filter_config1, can_filter_config2, can_filter_config3;

  can_filter_config1.FilterBank = 0;
  can_filter_config1.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_config1.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_config1.FilterIdHigh = (0x18EAB631 >> 13) & 0xFFFF;
  can_filter_config1.FilterIdLow = ( 0x18EAB631 << 3) & 0xFFF8;
  can_filter_config1.FilterMaskIdHigh = (0x1FFFF000 >> 13) & 0xFFFF;
  can_filter_config1.FilterMaskIdLow =(0x1FFFF000 << 3) & 0xFFF8;
  can_filter_config1.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter_config1.FilterActivation = ENABLE;

  can_filter_config2.FilterBank = 1;
  can_filter_config2.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_config2.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_config2.FilterIdHigh = (0x1CECB631 >> 13) & 0xFFFF;
  can_filter_config2.FilterIdLow = (0x1CECB631 << 3) & 0xFFF8;
  can_filter_config2.FilterMaskIdHigh = (0x1FFFF000 >> 13) & 0xFFFF;
  can_filter_config2.FilterMaskIdLow = (0x1FFFF000 << 3) & 0xFFF8;
  can_filter_config2.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter_config2.FilterActivation = ENABLE;

  can_filter_config3.FilterBank = 2;
  can_filter_config3.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_config3.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_config3.FilterIdHigh = (0x18EFB631 >> 13) & 0xFFFF;
  can_filter_config3.FilterIdLow = (0x18EFB631 << 3) & 0xFFF8;
  can_filter_config3.FilterMaskIdHigh = (0x1FFFF000 >> 13) & 0xFFFF;
  can_filter_config3.FilterMaskIdLow = (0x1FFFF000 << 3) & 0xFFF8;
  can_filter_config3.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter_config3.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan, &can_filter_config1) != HAL_OK)
  {
    status = CAN_ERROR;
    Error_Handler();
  }

  if (HAL_CAN_ConfigFilter(&hcan, &can_filter_config2) != HAL_OK)
  {
    status = CAN_ERROR;
    Error_Handler();
  }

  if (HAL_CAN_ConfigFilter(&hcan, &can_filter_config3) != HAL_OK)
  {
    status = CAN_ERROR;
    Error_Handler();
  }

  /* USER CODE END CAN_Init 2 */

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

/* USER CODE BEGIN Header_StartLoggingTask */
/**
  * @brief  Function implementing the loggingTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLoggingTask */
void StartLoggingTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    data_table[TYPE_MISC][INDEX_STATUS] = status;
    BlinkStatus(status, BLINK_SPEED);
    osDelay(STATUS_PERIOD);
  }
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPollingTask */
/**
* @brief Function implementing the pollingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPollingTask */
void StartPollingTask(void *argument)
{
  /* USER CODE BEGIN StartPollingTask */

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

    // Toggle LED 0
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, data_table[TYPE_GPIO][INDEX_PB5]);

    osDelay(DATA_INTERVAL);
  }
  osThreadTerminate(NULL);
  /* USER CODE END StartPollingTask */
}

/* USER CODE BEGIN Header_StartRealTimeTask */
/**
* @brief Function implementing the realTimeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRealTimeTask */
void StartRealTimeTask(void *argument)
{
  /* USER CODE BEGIN StartRealTimeTask */

  J1939Message message;

  HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&i2c_rx_buffer, I2C_RX_BUFFER_SIZE);

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  ClaimJ1939Address(0xB5);
  ClaimJ1939Address(0xB6);

  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(canQueueHandle, &message, NULL, 0) == osOK)
    {
      ProcessJ1939Message(message);
    }
    osDelay(1);
  }
  osThreadTerminate(NULL);
  /* USER CODE END StartRealTimeTask */
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
