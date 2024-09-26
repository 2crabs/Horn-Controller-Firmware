/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "Display.h"
#include "TCA6424.h"
#include "WS2812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HORN_MODE_THREE_MINUTE 1
#define HORN_MODE_FIVE_MINUTE 0
#define HORN_MODE_ROLLING 0b10
#define HORN_MODE_STOP 0b11

#define RELAY_OK 0
#define RELAY_ERR 1
#define RELAY_TWO 1
#define RELAY_ONE 0

#define RGB_BAT 0
#define RGB_REMOTE 1
#define RGB_MODE 2

#define ADC_CUR 0
#define ADC_BAT 1

#define CAN_REMOTE_ID ((0xBB)<<3)
#define CAN_CONTROLLER_ID ((0xAA)<<3)
#define CAN_MSG_START 0b010
#define CAN_MSG_HORN 0b011
#define CAN_MSG_PERIODIC 0b000
#define CAN_MSG_TIME 0b001
#define CAN_MSG_ERROR 0b111
#define CAN_MSG_BUZZER 0b100

#define ERROR_CODE_VOLT 1
#define ERROR_CODE_CUR 2


#define RUNNING 1
#define STOPPED 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 64 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId displayTaskHandle;
uint32_t displayTaskBuffer[ 80 ];
osStaticThreadDef_t displayTaskControlBlock;
osThreadId hornTaskHandle;
uint32_t HornTaskBuffer[ 80 ];
osStaticThreadDef_t HornTaskControlBlock;
osThreadId CANReceiveTaskHandle;
uint32_t CANReceiveTaskBuffer[ 80 ];
osStaticThreadDef_t CANReceiveTaskControlBlock;
osThreadId hornCheckTaskHandle;
uint32_t hornCheckTaskBuffer[ 80 ];
osStaticThreadDef_t hornCheckTaskControlBlock;
osMessageQId hornSequenceQueueHandle;
uint8_t hornSequenceQueueBuffer[ 1 * sizeof( uint8_t ) ];
osStaticMessageQDef_t hornSequenceQueueControlBlock;
osMessageQId canReceiveQueueHandle;
uint8_t canReceiveQueueBuffer[ 1 * sizeof( uint8_t ) ];
osStaticMessageQDef_t canReceiveQueueControlBlock;
osMessageQId hornCheckQueueHandle;
uint8_t hornCheckQueueBuffer[ 1 * sizeof( uint8_t ) ];
osStaticMessageQDef_t hornCheckQueueControlBlock;
/* USER CODE BEGIN PV */
TCA6424 ioexpander;

WS2812 rgbLeds;
uint8_t rgbBuffer[WS2812_BUF_LEN];
uint8_t rgbData[9] = {20,0,0,20,0,0,20,0,0};

uint8_t relay1Condition;
uint8_t relay2Condition;
//relay used to power horn(defaults to RELAY_2)
uint8_t currentRelay = RELAY_TWO;

uint32_t adcResults[2];

uint8_t dummyData = 0;

uint8_t hornMode = HORN_MODE_THREE_MINUTE;

const TickType_t hornSequence_Five[4] = {60000, 120000, 300000, 360000};
const TickType_t hornLengths_Five[4] = {900, 900, 2000, 900};
const TickType_t sequenceLength_Five = 360000;

const TickType_t hornSequence_Three[12] = {20000, 20500, 21000, 21500, 22000, 30000, 31500, 33000, 90000, 91500, 150000, 210000};
const TickType_t hornLengths_Three[12] = {250, 250, 250, 250, 250, 900, 900, 900, 900, 900, 900, 900};
const TickType_t sequenceLength_Three = 210000;

TickType_t timerStartTick;
uint8_t isRunning = STOPPED;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void StartDisplayTask(void const * argument);
void StartHornTask(void const * argument);
void StartCANReceiveTask(void const * argument);
void StartHornCheckTask(void const * argument);

/* USER CODE BEGIN PFP */
static uint16_t CalculatePercentage(uint32_t adcReading);
static void setRelay(GPIO_PinState state);
static void updateHornMode();
static void updateDisplays(uint8_t minutes, uint8_t seconds);
static void updateRGB();
static void sendError(uint8_t errorCode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
  WS2812_StopDMA(&rgbLeds);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xQueueSendFromISR(canReceiveQueueHandle, &dummyData,  &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
  MX_ADC_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  WS2812_Reset_Buf(rgbBuffer);
  WS2812_Init(&rgbLeds, &htim2, TIM_CHANNEL_2);
  WS2812_Send(&rgbLeds, rgbBuffer);

  TCA6424_Init(&ioexpander, &hi2c1, GPIOB, IO_RST_Pin);
  TCA6424_SetAsOutputs(&ioexpander);

  updateHornMode();
  updateRGB();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of hornSequenceQueue */
  osMessageQStaticDef(hornSequenceQueue, 1, uint8_t, hornSequenceQueueBuffer, &hornSequenceQueueControlBlock);
  hornSequenceQueueHandle = osMessageCreate(osMessageQ(hornSequenceQueue), NULL);

  /* definition and creation of canReceiveQueue */
  osMessageQStaticDef(canReceiveQueue, 1, uint8_t, canReceiveQueueBuffer, &canReceiveQueueControlBlock);
  canReceiveQueueHandle = osMessageCreate(osMessageQ(canReceiveQueue), NULL);

  /* definition and creation of hornCheckQueue */
  osMessageQStaticDef(hornCheckQueue, 1, uint8_t, hornCheckQueueBuffer, &hornCheckQueueControlBlock);
  hornCheckQueueHandle = osMessageCreate(osMessageQ(hornCheckQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 64, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of displayTask */
  osThreadStaticDef(displayTask, StartDisplayTask, osPriorityNormal, 0, 80, displayTaskBuffer, &displayTaskControlBlock);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

  /* definition and creation of hornTask */
  osThreadStaticDef(hornTask, StartHornTask, osPriorityAboveNormal, 0, 80, HornTaskBuffer, &HornTaskControlBlock);
  hornTaskHandle = osThreadCreate(osThread(hornTask), NULL);

  /* definition and creation of CANReceiveTask */
  osThreadStaticDef(CANReceiveTask, StartCANReceiveTask, osPriorityRealtime, 0, 80, CANReceiveTaskBuffer, &CANReceiveTaskControlBlock);
  CANReceiveTaskHandle = osThreadCreate(osThread(CANReceiveTask), NULL);

  /* definition and creation of hornCheckTask */
  osThreadStaticDef(hornCheckTask, StartHornCheckTask, osPriorityHigh, 0, 80, hornCheckTaskBuffer, &hornCheckTaskControlBlock);
  hornCheckTaskHandle = osThreadCreate(osThread(hornCheckTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc);
  /* USER CODE END ADC_Init 2 */

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
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  CAN_FilterTypeDef filter;
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterMaskIdHigh = 0x0;
  filter.FilterMaskIdLow = 0x0;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &filter);
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
  hi2c1.Init.Timing = 0x0000020B;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAY1_Pin|RELAY2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|IO_RST_Pin|RELAY1_LED_Pin|RELAY2_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MODE_SWITCH_Pin CUR_FAULT_Pin START_SWITCH_Pin */
  GPIO_InitStruct.Pin = MODE_SWITCH_Pin|CUR_FAULT_Pin|START_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY1_Pin RELAY2_Pin */
  GPIO_InitStruct.Pin = RELAY1_Pin|RELAY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_TEST_Pin */
  GPIO_InitStruct.Pin = RELAY_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RELAY_TEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HORN_SWITCH_Pin */
  GPIO_InitStruct.Pin = HORN_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HORN_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin IO_RST_Pin RELAY1_LED_Pin RELAY2_LED_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|IO_RST_Pin|RELAY1_LED_Pin|RELAY2_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static uint16_t CalculatePercentage(uint32_t adcReading){
  int32_t percentage = (adcReading*3363)/10000 - 923;
  uint8_t result;
  if (percentage<0){
    result = 0;
  }
  else if (percentage>100){
    result = 100;
  } else {
    result = percentage;
  }
  result = (result/5) * 5;
  return result;
}

static void setRelay(GPIO_PinState state){
  if (currentRelay == RELAY_ONE){
    HAL_GPIO_WritePin(GPIOA, RELAY1_Pin, state);
    HAL_GPIO_WritePin(RELAY1_LED_GPIO_Port, RELAY1_LED_Pin, state);
  } else {
    HAL_GPIO_WritePin(GPIOA, RELAY2_Pin, state);
    HAL_GPIO_WritePin(RELAY2_LED_GPIO_Port, RELAY2_LED_Pin, state);
  }
  if (state == GPIO_PIN_SET) {
    xQueueSendToBack(hornCheckQueueHandle, &dummyData, 0);
  }
}

static void updateHornMode(){
  hornMode = HAL_GPIO_ReadPin(GPIOA, MODE_SWITCH_Pin);
  if (hornMode == HORN_MODE_FIVE_MINUTE){
    rgbData[6] = 0;
    rgbData[7] = 0;
    rgbData[8] = 20;
  } else {
    rgbData[6] = 0;
    rgbData[7] = 10;
    rgbData[8] = 20;
  }
}

static void updateDisplays(uint8_t minutes, uint8_t seconds){
  static uint32_t mailbox;
  static uint8_t timerCanData[3];
  static CAN_TxHeaderTypeDef CANHeader;
  CANHeader.StdId = CAN_CONTROLLER_ID | CAN_MSG_TIME;
  CANHeader.DLC = 0x03;
  CANHeader.IDE = CAN_ID_STD;
  CANHeader.RTR = CAN_RTR_DATA;

  timerCanData[0] = minutes;
  timerCanData[1] = seconds;
  timerCanData[2] = (isRunning << 1) | hornMode;

  Display_SetMinSec(&ioexpander, minutes, seconds);
  HAL_CAN_AddTxMessage(&hcan, &CANHeader, timerCanData, &mailbox);
}

static void updateRGB(){
  WS2812_Write_Buf(rgbBuffer, rgbData[0], rgbData[1], rgbData[2], RGB_BAT);
  WS2812_Write_Buf(rgbBuffer, rgbData[3], rgbData[4], rgbData[5], RGB_REMOTE);
  WS2812_Write_Buf(rgbBuffer, rgbData[6], rgbData[7], rgbData[8], RGB_MODE);
  WS2812_Send(&rgbLeds, rgbBuffer);
}

static void sendError(uint8_t errorCode){
  static uint32_t mailbox;
  static uint8_t sendData;
  static CAN_TxHeaderTypeDef CANHeader;
  CANHeader.StdId = CAN_CONTROLLER_ID | CAN_MSG_ERROR;
  CANHeader.DLC = 0x01;
  CANHeader.IDE = CAN_ID_STD;
  CANHeader.RTR = CAN_RTR_DATA;

  sendData = errorCode;

  HAL_CAN_AddTxMessage(&hcan, &CANHeader, &sendData, &mailbox);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
   vTaskDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  /* Infinite loop */
  TickType_t taskStartTick = xTaskGetTickCount();

  //show voltage for 5 seconds
  while((xTaskGetTickCount() - taskStartTick) < 7000){
    HAL_ADC_Start_DMA(&hadc, adcResults, 2);
    vTaskDelay(10);
    uint8_t percentage = CalculatePercentage(adcResults[ADC_BAT]);
    rgbData[0] = 20-(percentage/5);
    rgbData[1] = percentage/5;
    updateDisplays(percentage/100, percentage%100);
    updateRGB();
    vTaskDelay(125);
  }

  for(;;)
  {
    if(!isRunning){
      updateHornMode();
      timerStartTick = xTaskGetTickCount();
    }

    // 5 minute horn has stopped
    if((sequenceLength_Five+1000+timerStartTick) < xTaskGetTickCount()){
      timerStartTick = xTaskGetTickCount() - 61000;
    }

    //three minute horn has ended
    if((hornMode == HORN_MODE_THREE_MINUTE) && (sequenceLength_Three+1000+timerStartTick) < xTaskGetTickCount()){
      isRunning = STOPPED;
    }


    if (!isRunning && (hornMode == HORN_MODE_FIVE_MINUTE)){
      updateDisplays(6, 0);
    } else if (!isRunning && (hornMode == HORN_MODE_THREE_MINUTE)){
      updateDisplays(3, 30);
    } else {
      uint16_t secondsToZero = ((sequenceLength_Five+1000)+timerStartTick-xTaskGetTickCount())/1000;
      if(hornMode == HORN_MODE_THREE_MINUTE){
        secondsToZero = ((sequenceLength_Three+1000)+timerStartTick-xTaskGetTickCount())/1000;
      }
      uint8_t minutes = secondsToZero/60;
      uint8_t seconds = secondsToZero%60;
      updateDisplays(minutes, seconds);
    }
    updateRGB();
    vTaskDelay(10);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartHornTask */
/**
* @brief starts horn sequence when data is added to queue. Will stop the sequence and wait for more data if data is sent while running sequence of horns
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHornTask */
void StartHornTask(void const * argument)
{
  /* USER CODE BEGIN StartHornTask */
  /* Infinite loop */
  static uint8_t settings;
  static uint8_t sequenceStopped = 0;;
  static uint8_t rolling = HORN_MODE_ROLLING;
  for(;;)
  {
    xQueueReceive(hornSequenceQueueHandle, &settings, portMAX_DELAY);

    if (settings == HORN_MODE_THREE_MINUTE){

      /* 3 minute horns */
      for (uint8_t i = 0; i < (sizeof(hornLengths_Three)/4); i++) {
        TickType_t delayAmount = hornSequence_Three[i]+timerStartTick-xTaskGetTickCount();
        if (xQueueReceive(hornSequenceQueueHandle, &settings, delayAmount) == pdTRUE) {break;};
        setRelay(GPIO_PIN_SET);
        if (xQueueReceive(hornSequenceQueueHandle, &settings, hornLengths_Three[i]) == pdTRUE) {break;}
        setRelay(GPIO_PIN_RESET);
      }
      /* 3 minute horns */

    } else if (settings == HORN_MODE_FIVE_MINUTE){

      /* 5 minute horns */
      for (uint8_t i = 0; i < (sizeof(hornLengths_Five)/4); i++) {
        TickType_t delayAmount = hornSequence_Five[i]+timerStartTick-xTaskGetTickCount();
        if (xQueueReceive(hornSequenceQueueHandle, &settings, delayAmount) == pdTRUE) {sequenceStopped = 1; break;};
        setRelay(GPIO_PIN_SET);
        if (xQueueReceive(hornSequenceQueueHandle, &settings, hornLengths_Five[i]) == pdTRUE) {sequenceStopped = 1; break;};
        setRelay(GPIO_PIN_RESET);
      }
      if(!sequenceStopped){
        xQueueSendToBack(hornSequenceQueueHandle, &rolling, 5);
      }
      /* 5 minute horns */

    } else if (settings == HORN_MODE_ROLLING){

      /* rolling horns */
      for (uint8_t i = 1; i < (sizeof(hornLengths_Five)/4); i++) {
        TickType_t delayAmount = hornSequence_Five[i]+timerStartTick-xTaskGetTickCount();
        if (xQueueReceive(hornSequenceQueueHandle, &settings, delayAmount) == pdTRUE) {sequenceStopped = 1; break;};
        setRelay(GPIO_PIN_SET);
        if (xQueueReceive(hornSequenceQueueHandle, &settings, hornLengths_Five[i]) == pdTRUE) {sequenceStopped = 1; break;};
        setRelay(GPIO_PIN_RESET);
      }
      if(!sequenceStopped){
        xQueueSendToBack(hornSequenceQueueHandle, &rolling, 5);
      }
     /* rolling horns */

    }
    sequenceStopped = 0;
    setRelay(GPIO_PIN_RESET);
  }
  /* USER CODE END StartHornTask */
}

/* USER CODE BEGIN Header_StartCANReceiveTask */
/**
* @brief Function implementing the CANReceiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANReceiveTask */
void StartCANReceiveTask(void const * argument)
{
  /* USER CODE BEGIN StartCANReceiveTask */
  /* Infinite loop */
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  static uint8_t stop = HORN_MODE_STOP;
  static CAN_RxHeaderTypeDef canHeader;
  static uint8_t canData[8];

  for(;;)
  {
    if (xQueueReceive(canReceiveQueueHandle, &dummyData, 1000) == pdFALSE){
      isRunning = STOPPED;
      rgbData[3] = 20;
      rgbData[4] = 0;
      xQueueSendToBack(hornSequenceQueueHandle, &stop, 0);
    } else {
      HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &canHeader, canData);
      rgbData[3] = 0;
      rgbData[4] = 20;

      if (canHeader.StdId == (CAN_REMOTE_ID | CAN_MSG_START)){
        if (!isRunning){
          isRunning = RUNNING;
          timerStartTick = xTaskGetTickCount();
          xQueueSendToBack(hornSequenceQueueHandle, &hornMode, 0);
        } else {
          isRunning = STOPPED;
          xQueueSendToBack(hornSequenceQueueHandle, &stop, 0);
        }
      }

      if (canHeader.StdId == (CAN_REMOTE_ID | CAN_MSG_BUZZER)){
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, canData[0]);
      }

      HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }

  }
  /* USER CODE END StartCANReceiveTask */
}

/* USER CODE BEGIN Header_StartHornCheckTask */
/**
* @brief Function implementing the hornCheckTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHornCheckTask */
void StartHornCheckTask(void const * argument)
{
  /* USER CODE BEGIN StartHornCheckTask */
  /* Infinite loop */
  for(;;)
  {
    xQueueReceive(hornCheckQueueHandle, &dummyData, portMAX_DELAY);
    //wait for relay to close and horn to start
    vTaskDelay(245);
    HAL_ADC_Start_DMA(&hadc, adcResults, 2);
    vTaskDelay(5);

    if (adcResults[ADC_CUR] < 2250) {
      if (HAL_GPIO_ReadPin(RELAY_TEST_GPIO_Port, RELAY_TEST_Pin) != GPIO_PIN_RESET) {
        sendError(ERROR_CODE_VOLT);
        setRelay(GPIO_PIN_RESET);
        currentRelay = !currentRelay;
        setRelay(GPIO_PIN_SET);
      } else {
        sendError(ERROR_CODE_CUR);
      }
    }
  }
  /* USER CODE END StartHornCheckTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
