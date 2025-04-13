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
#include "mbedtls.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "../../ECUAL/LCD16X2/LCD16X2.h"
#include "mbedtls/aes.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

  uint16_t adc_res[5];

  char adc_res_string [15];

  uint16_t input_rpm = 600;

  uint16_t coolant_value = 0;

  uint8_t fan_display_status = 0 ;//0: fan off , 1: fan low on , 2: fan high on

  uint8_t uart_buff[248];

  int _hour , _minute , _seconds;

  mbedtls_aes_context aes;

  unsigned char key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                           0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

  // Initialization Vector (IV, 16 bytes)
  unsigned char iv[16] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x18,
                          0x29, 0x3A, 0x4B, 0x5C, 0x6D, 0x7E, 0x8F, 0x90};

  char temp_result[16];

  static volatile int _engine_start_valid_ = 0;  //start engine or not!

  char get_encrypted_immo_data[16]; //data which recived from can bus

  const char * immo_code = "2C00F64060FA";

  uint8_t immo_read_ok = 0;

 uint8_t read_first_byte = 0;
 uint8_t read_second_byte = 0;

  //coolant level based on adc to convert

  const float coolant_level1_low = 0.3;

  const float coolant_level1_high = 0.5;

  const float coolant_level2_low = 0.6;

  const float coolant_level2_high = 1.1;

  const float coolant_level3_low = 1.6;

  const float coolant_level3_high = 2.1;


  const float coolant_level4_low = 2.2;

  const float coolant_level4_high = 2.7;

  const float coolant_level5_low = 2.8;

  const float coolant_level5_high = 3.3;

  //*********************************************

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NO_PERMISSION 0  //engine start permission by immoblizer
#define OK_PERMISSION 1 ///engine valid start permission by immoblizer

#define MyLCD LCD16X2_1
 void handle_rpm_input();
 void handle_rpm_increase();
 void handle_rpm_decrease();
 void handle_coolant();
uint16_t make_coolant_value(float _input_);
void make_rpm_can_data(uint16_t _rpm_);
void make_coolant_can_data(uint16_t _coolant_);
void make_fan_can_data(uint16_t _fan_);
void write_fan_status(); //write fan ststus on LCD
void get_time(int *hour , int *minute , int *seconds);
char * make_string_time(int hour , int minute);
char * make_encrypted_data(char *plain_text);
char *make_decrypted_data(char *cipher_text);
uint8_t convert_can_message_immo(char *res);
void send_encrypted_immo_data(char *chiper_text);
void handle_valid_switch_start();

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lcd_drive */
osThreadId_t lcd_driveHandle;
uint32_t lcd_driveBuffer[ 512 ];
osStaticThreadDef_t lcd_driveControlBlock;
const osThreadAttr_t lcd_drive_attributes = {
  .name = "lcd_drive",
  .cb_mem = &lcd_driveControlBlock,
  .cb_size = sizeof(lcd_driveControlBlock),
  .stack_mem = &lcd_driveBuffer[0],
  .stack_size = sizeof(lcd_driveBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can_control */
osThreadId_t can_controlHandle;
const osThreadAttr_t can_control_attributes = {
  .name = "can_control",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for input */
osThreadId_t inputHandle;
const osThreadAttr_t input_attributes = {
  .name = "input",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for calculate */
osThreadId_t calculateHandle;
const osThreadAttr_t calculate_attributes = {
  .name = "calculate",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for make_immo_data */
osThreadId_t make_immo_dataHandle;
uint32_t make_immo_dataBuffer[ 4096 ];
osStaticThreadDef_t make_immo_dataControlBlock;
const osThreadAttr_t make_immo_data_attributes = {
  .name = "make_immo_data",
  .cb_mem = &make_immo_dataControlBlock,
  .cb_size = sizeof(make_immo_dataControlBlock),
  .stack_mem = &make_immo_dataBuffer[0],
  .stack_size = sizeof(make_immo_dataBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

uint32_t rx_headr;

uint8_t rx_data[8] = {0};

uint32_t tx_header =0x321;

uint8_t coolant_data[8] = {0XFF, 0XFF, 0XFF , 0XFF , 0XFF , 0XFF , 0XFF , 0XFF};

uint8_t rpm_data[8] = {0XFF, 0XFF, 0XFF , 0XFF , 0XFF , 0XFF , 0XFF , 0XFF};

uint8_t fan_data[8] = {0XFF, 0XFF, 0XFF , 0XFF , 0XFF , 0XFF , 0XFF , 0XFF};

#define RX_BUFF_SIZE  248


char UART_RX_BUFF[RX_BUFF_SIZE];


uint8_t UART_BUF_INDEX = 0;


uint8_t get_message = 0;

int i;

char data_to_send[45];

CAN_TxHeaderTypeDef engine_rpm_header;
CAN_TxHeaderTypeDef coolant_rpm_header;
CAN_TxHeaderTypeDef fan_rpm_header;
CAN_TxHeaderTypeDef immo_1; //fisrt byte of immoblizer (since aes makes 16 byte data we send data in two messages)
CAN_TxHeaderTypeDef immo_2; //second byte of immoblizer (since aes makes 16 byte data we send data in two messages)
CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef cluster_filter;
uint32_t tx_mail_box = 0;

uint8_t get_data[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void display_data(void *argument);
void can_send_data(void *argument);
void get_input_data(void *argument);
void make_engine_data(void *argument);
void make_immo_data_func(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  HAL_StatusTypeDef RES =0;
  HAL_StatusTypeDef UART_RES =0;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* Call PreOsInit function */
  MX_MBEDTLS_Init();
  /* USER CODE BEGIN 2 */

  cluster_filter.FilterActivation = CAN_FILTER_ENABLE;

  cluster_filter.FilterBank = 0;

  cluster_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

  cluster_filter.FilterIdHigh = 0;

  cluster_filter.FilterIdLow  =0 ;

  cluster_filter.FilterMaskIdHigh = 0;

  cluster_filter.FilterMaskIdLow = 0;

  cluster_filter.FilterMode  =  CAN_FILTERMODE_IDMASK;

  cluster_filter.FilterScale = CAN_FILTERSCALE_32BIT;

  RES = HAL_CAN_ConfigFilter(&hcan1, &cluster_filter);

 RES = HAL_CAN_Start(&hcan1);

 RES = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//*************************************************************
  engine_rpm_header.DLC =8;
  engine_rpm_header.IDE = CAN_ID_EXT;
  engine_rpm_header.ExtId = 0XCF00401;
  //**************************************************
  coolant_rpm_header.DLC =8;
  coolant_rpm_header.IDE = CAN_ID_EXT;
  coolant_rpm_header.ExtId = 0X18FE63FE;
  //***************************************************
  coolant_rpm_header.DLC =8;
  coolant_rpm_header.IDE = CAN_ID_EXT;
  coolant_rpm_header.ExtId = 0X18FEBDFE;
  //***************************************************
  immo_1.DLC =8;
  immo_1.IDE = CAN_ID_EXT;
  immo_1.ExtId = 0X1CABBAEE;
  //***************************************************
  immo_2.DLC =8;
  immo_2.IDE = CAN_ID_EXT;
  immo_2.ExtId = 0X1CABBBEE;
  //***************************************************


  HAL_UART_Receive_IT(&huart1, (uint8_t *)get_data, 1);





     memset(adc_res_string , 0 , 15);
     memset(adc_res , 0 , 5 * sizeof(uint16_t));

  //  HAL_ADC_Start_DMA(&hadc2, &adc_res, 1);

     HAL_StatusTypeDef adc_hal_res ;

     uint32_t adc_channel_one = 0;

     adc_hal_res =  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_res, 5);

     HAL_UART_Receive_DMA(&huart3,  uart_buff , 12); //tag code are 12 digits!


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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of lcd_drive */
  lcd_driveHandle = osThreadNew(display_data, NULL, &lcd_drive_attributes);

  /* creation of can_control */
  can_controlHandle = osThreadNew(can_send_data, NULL, &can_control_attributes);

  /* creation of input */
  inputHandle = osThreadNew(get_input_data, NULL, &input_attributes);

  /* creation of calculate */
  calculateHandle = osThreadNew(make_engine_data, NULL, &calculate_attributes);

  /* creation of make_immo_data */
  make_immo_dataHandle = osThreadNew(make_immo_data_func, NULL, &make_immo_data_attributes);

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
    //}


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : inc_rpm_Pin */
  GPIO_InitStruct.Pin = inc_rpm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(inc_rpm_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD12 PD13
                           PD14 PD15 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : dec_rpm_Pin */
  GPIO_InitStruct.Pin = dec_rpm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(dec_rpm_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void handle_rpm(){

	 handle_rpm_increase();
	 handle_rpm_decrease();
}

void handle_rpm_increase(){


    if(HAL_GPIO_ReadPin(GPIOE, inc_rpm_Pin) == 0){
     HAL_Delay(200);
     input_rpm += 100;
    }
    if(input_rpm>=2500)
   	 input_rpm = 2500;

}
void handle_rpm_decrease(){


    if(HAL_GPIO_ReadPin(GPIOE, dec_rpm_Pin) == 0){
     HAL_Delay(200);
     input_rpm -= 100;
    }
    if(input_rpm<=600)
   	 input_rpm = 600;

}

uint16_t make_coolant_value(float _input_){

	if(_input_ >= coolant_level1_low && _input_ <= coolant_level1_high)
		return 60;

	if(_input_ >= coolant_level2_low && _input_ <= coolant_level3_high)
			return 70;

	if(_input_ >= coolant_level3_low && _input_ <= coolant_level3_high)
			return 80;

	if(_input_ >= coolant_level4_low && _input_ <= coolant_level4_high)
				return 90;

	if(_input_ >= coolant_level5_low && _input_ <= coolant_level5_high)
					return 100;

	return 0;


}

void handle_coolant(){


	 coolant_value = make_coolant_value((float)((adc_res[0] * 3.3) /4095));

}

void make_rpm_can_data(uint16_t _rpm){

	_rpm *=8; //based on j1939, factor is divied by 8, so we multiple in 8(https://www.csselectronics.com/pages/j1939-explained-simple-intro-tutorial)
	uint8_t rpm_high_byte = (_rpm >> 8) & 0xFF; //convert 16bit to two  bit numbers
	uint8_t rpm_low_byte = _rpm & 0xFF;

	rpm_data[3] = rpm_low_byte;
	rpm_data[4] = rpm_high_byte;


}

 void make_coolant_can_data(uint16_t _coolant_){

	 _coolant_ -=40; //based on j1939 dbc: https://github.com/nberlette/canbus/blob/main/dbc/j1939.dbc
  uint8_t coolant_high_byte = (_coolant_ >> 8) & 0xFF; //convert 16bit to two  bit numbers
  uint8_t coolant_low_byte = _coolant_ & 0xFF;

  coolant_data[2] = coolant_low_byte;
  coolant_data[3] = coolant_high_byte;

 }

 void make_fan_can_data(uint16_t _fan_){

	  if(_fan_ <70) {//fan off

		_fan_ = 0;

		fan_display_status = 0;

	  }

	  else if(_fan_>=70 && _fan_<=90){  //low fan start

		_fan_ = 600;

		fan_display_status = 1;

	  }

	  else if( _fan_>90){ //high fan start

		_fan_ = 1500;

		fan_display_status = 2;

	  }



	    _fan_ *=8; //based on j1939, factor is divied by 8, so we multiple in 8(https://www.csselectronics.com/pages/j1939-explained-simple-intro-tutorial)
	 	uint8_t fan_rpm_high_byte = (_fan_ >> 8) & 0xFF; //convert 16bit to two  bit numbers
	 	uint8_t fan_rpm_low_byte = _fan_ & 0xFF;

	 	fan_data[2] = fan_rpm_low_byte;
	 	fan_data[3] = fan_rpm_high_byte;


 }

void  write_fan_status(){

	 LCD16X2_Set_Cursor(MyLCD, 1, 14);

	  if(fan_display_status == 0)
	  LCD16X2_Write_String(MyLCD, "OFF");

	  else if(fan_display_status == 1)
		  LCD16X2_Write_String(MyLCD, "LOW");

	  else if(fan_display_status == 2)
	 		  LCD16X2_Write_String(MyLCD, "HIGH");



}

void get_time(int *hours , int *minutes , int *seconds){

	uint32_t totalSeconds = HAL_GetTick() /1000;

	   *hours = totalSeconds / 3600;          // Calculate hours
	    *minutes = (totalSeconds % 3600) / 60; // Calculate minutes
	    *seconds = totalSeconds % 60;

}

char * make_string_time(int hour , int minute){

	char __hour[5] , __minute[5];

	static char result [15];
	memset(result , 0 , 15);

	itoa(hour , __hour , 10);
	itoa(minute , __minute , 10);

	strcpy(result , __hour);
	strcat(result , __minute);

	return result;

}
  char * make_encrypted_data(char *plain_text){

	static char result[16];
	memset(result , 0 , 16);

	//add padding!
	if(strlen(plain_text)!=16){

		for(int i= strlen(plain_text); i<16; i++)

			plain_text[i]='F';
	}

    mbedtls_aes_init(&aes);
     mbedtls_aes_setkey_enc(&aes, key, 128);
	//int __index = strlen(plain_text); //since we have data less than 16 bytes we must complete it


	 // mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, 16, iv, (unsigned char *)plain_text, (unsigned char *)result);

	//mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, 16, (unsigned char *)iv, (unsigned char *)plain_text, (unsigned char *)result);

	mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, (unsigned char *) plain_text, (unsigned char *)result);

 	 strcpy(temp_result , result);
 	 //for(int i=0; i<16; i++)
 	// sprintf("%02X " , temp_result[i]);
 	 HAL_UART_Transmit(&huart1, (uint8_t *)"input data:", 12, HAL_MAX_DELAY);
 	 	HAL_Delay(500);
 	 	 HAL_UART_Transmit(&huart1, (uint8_t *)plain_text, strlen(plain_text), HAL_MAX_DELAY);
 	 	 	 	 HAL_Delay(500);
 	 	 HAL_UART_Transmit(&huart1, (uint8_t *)"\n", 1, HAL_MAX_DELAY);
 	 	 HAL_Delay(500);
 	 	 HAL_UART_Transmit(&huart1, (uint8_t *)"coded data:", 12, HAL_MAX_DELAY);
 	 	 	 	HAL_Delay(500);
 	 HAL_UART_Transmit(&huart1, (uint8_t *)temp_result, 16, HAL_MAX_DELAY);
 	 HAL_Delay(500);
 	HAL_UART_Transmit(&huart1, (uint8_t *)"\n", 1, HAL_MAX_DELAY);
 	 HAL_Delay(500);
	 mbedtls_aes_free(&aes);
    return result;
	}

  char *make_decrypted_data(char *cipher_text){

	 static char result[16];

	    mbedtls_aes_init(&aes);
	     mbedtls_aes_setkey_dec(&aes, key, 128);

	 mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, (unsigned char *) cipher_text, (unsigned char *)result);
	//mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, 16, (unsigned char *)iv, (unsigned char *)cipher_text, (unsigned char *)result);


   HAL_UART_Transmit(&huart1, (uint8_t *)"decoded data:", 13, HAL_MAX_DELAY);
   	// HAL_Delay(500);

  HAL_UART_Transmit(&huart1, (uint8_t *)result, strlen(result), HAL_MAX_DELAY);
  	 //HAL_Delay(500);
  	 HAL_UART_Transmit(&huart1, (uint8_t *)"\n", 1, HAL_MAX_DELAY);
  	 //HAL_Delay(500);

  	 mbedtls_aes_free(&aes);

  return result;

  }

  void send_encrypted_immo_data(char *cipher_text){

	  uint8_t first_byte[8];
	  uint8_t second_byte[8];


	  for(int i=0; i<8 ; i++)
		  first_byte[i] = (uint8_t)cipher_text[i];

	  int j=0;
	  for(int i=8; i<16 ; i++){
	 		  second_byte[j] = (uint8_t)cipher_text[i];
	 		  j++;
	  }

	  HAL_CAN_AddTxMessage(&hcan1, &immo_1, first_byte, &tx_mail_box);
	  HAL_Delay(500);
	  HAL_CAN_AddTxMessage(&hcan1, &immo_2, second_byte, &tx_mail_box);
	  HAL_Delay(500);

  }

  uint8_t convert_can_message_immo(char *res){

	  int j= 8;

	  uint8_t result = 0;

	  const char *m1 = "first immo received\n";
	  const char *m2 = "second immo received\n";


	  if(RxHeader.ExtId == 0X1CABBAEE){
        for(int i=0; i<8; i++)
        	get_encrypted_immo_data[i] = rx_data[i];
        result++;
       // HAL_UART_Transmit(&huart1, (uint8_t *)rx_data, 8, HAL_MAX_DELAY);
       // HAL_UART_Transmit(&huart1, (uint8_t *)"\n", 1, HAL_MAX_DELAY);
       // HAL_Delay(500);
        read_first_byte = 1;

	  }


	  if(RxHeader.ExtId == 0X1CABBBEE){

		  result++;

		 // HAL_UART_Transmit(&huart1, (uint8_t *)rx_data, 8, HAL_MAX_DELAY);
		  //HAL_UART_Transmit(&huart1, (uint8_t *)"\n", 1, HAL_MAX_DELAY);


		  for(int i = 0; i<8; i++){

			  get_encrypted_immo_data[j] = rx_data[i];
			  j++;
		  }
		  read_second_byte = 1;
	 	  }

	  return result;
  }

  void handle_valid_switch_start(){


		 char ecu_time[16] = {0}, immo_time[16] = {0};
		 char decrypted_data[16]= {0};

		 get_time(&_hour, &_minute, &_seconds);
		 strcpy(ecu_time ,make_string_time(_hour, _minute));

	    strcpy(decrypted_data , make_decrypted_data(get_encrypted_immo_data));

	    for(int i=0; i<16;i++){

	    	    if(decrypted_data[i] == 'F')
	    	    	break;

	   			 immo_time[i] = decrypted_data[i];

	   		 }
	    HAL_UART_Transmit(&huart1, (uint8_t *)"immo time:", 10, HAL_MAX_DELAY);
	    HAL_UART_Transmit(&huart1, (uint8_t *)immo_time, strlen(immo_time), HAL_MAX_DELAY);
	    HAL_UART_Transmit(&huart1, (uint8_t *)"\n", 1, HAL_MAX_DELAY);
	    if(strcmp(ecu_time , immo_time)==0){

	    	_engine_start_valid_ = OK_PERMISSION;
	    HAL_UART_Transmit(&huart1, (uint8_t *)"VALID SWITCH\n", 13, HAL_MAX_DELAY);
	    }



  }

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){



   if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK){
	   get_message = 1;
   convert_can_message_immo("salam");
   if(read_first_byte == 1 && read_second_byte == 1)
	   handle_valid_switch_start();
}


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart->Instance == USART1)
	{

	UART_BUF_INDEX++;

	if(UART_BUF_INDEX>=  RX_BUFF_SIZE)
		UART_BUF_INDEX = 0;

	UART_RX_BUFF[UART_BUF_INDEX] = (char)get_data[0];

	HAL_UART_Receive_IT(&huart1, get_data, 1);

	}
}

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
	//  if(immo_read_ok ==2)
	//make_decrypted_data(get_encrypted_immo_data);
	 // HAL_UART_Transmit(&huart1, (uint8_t *)uart_buff, 12, HAL_MAX_DELAY);
	  //HAL_Delay(500);

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_display_data */
/**
* @brief Function implementing the lcd_drive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_display_data */
void display_data(void *argument)
{
  /* USER CODE BEGIN display_data */
	 LCD16X2_Init(MyLCD);
	 LCD16X2_Clear(MyLCD);
  /* Infinite loop */
  for(;;)
  {
	    while(_engine_start_valid_ == NO_PERMISSION){

	    	LCD16X2_Set_Cursor(MyLCD, 1, 1);
	    		  	  LCD16X2_Write_String(MyLCD, "NO VALID SWITCH!");
	    	HAL_Delay(500);
	    }

	   // else  if(_engine_start_valid_ == OK_PERMISSION)
	    {

	  	  LCD16X2_Set_Cursor(MyLCD, 1, 1);
	  	  LCD16X2_Write_String(MyLCD, "RPM:");

	  	  LCD16X2_Set_Cursor(MyLCD, 2, 1);
	  	  LCD16X2_Write_String(MyLCD, "Coolant:");

	  	 LCD16X2_Set_Cursor(MyLCD, 1, 10);
	     LCD16X2_Write_String(MyLCD, "FAN:");

	  	  LCD16X2_Set_Cursor(MyLCD, 1, 5);
	  	  itoa(input_rpm , adc_res_string , 10);
	  	  LCD16X2_Write_String(MyLCD, adc_res_string);

	  	  LCD16X2_Set_Cursor(MyLCD, 2, 10);
	  	  itoa(coolant_value  , adc_res_string , 10);
	  	 // snprintf(adc_res_string, sizeof(adc_res_string), "%.1f", (float)((coolant_value * 3.3) /4095));
	  	  LCD16X2_Write_String(MyLCD, adc_res_string);

	  	  write_fan_status();

	  	 osDelay(100);

	  	  LCD16X2_Clear(MyLCD);

	    }
  }
  /* USER CODE END display_data */
}

/* USER CODE BEGIN Header_can_send_data */
/**
* @brief Function implementing the can_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_send_data */
void can_send_data(void *argument)
{
  /* USER CODE BEGIN can_send_data */
  /* Infinite loop */
  for(;;)
  {
	   while(_engine_start_valid_ == NO_PERMISSION){
	    	   HAL_Delay(50);
	    }
	   //(_engine_start_valid_ == OK_PERMISSION)
			   {
	  /*RES = */HAL_CAN_AddTxMessage(&hcan1, &engine_rpm_header, rpm_data, &tx_mail_box);
		     HAL_Delay(50);

		     HAL_CAN_AddTxMessage(&hcan1, &coolant_rpm_header, coolant_data, &tx_mail_box);
		     HAL_Delay(50);


		     HAL_CAN_AddTxMessage(&hcan1, &fan_rpm_header, fan_data, &tx_mail_box);
		     HAL_Delay(50);

		     if(get_message == 1)
		     {
		     get_message = rx_data[0];
		     memset(data_to_send , 0 , 45);
		     char temp_data[1];

		     for(i=0; i<8; i++){

		     itoa(rx_data[i] , temp_data , 10);
		     strcat(data_to_send , temp_data);
		     strcat(data_to_send , ",");
		     }
		     strcat(data_to_send, "\n");
		     }
		    //  if(strstr(UART_RX_BUFF , "SALAM")){
		    // HAL_UART_Transmit(&huart1, (uint8_t *) "can ok\n", 7, HAL_MAX_DELAY);
		     //HAL_Delay(100);

		     /* UART_RES =HAL_UART_Transmit(&huart1, (uint8_t *) UART_RX_BUFF, sizeof(UART_RX_BUFF), HAL_MAX_DELAY);
			  HAL_Delay(100);
			  */
		     }
  }
  /* USER CODE END can_send_data */
}

/* USER CODE BEGIN Header_get_input_data */
/**
* @brief Function implementing the input thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_get_input_data */
void get_input_data(void *argument)
{
  /* USER CODE BEGIN get_input_data */
  /* Infinite loop */
  for(;;)
  {
     handle_rpm();
     handle_coolant();

  }
  /* USER CODE END get_input_data */
}

/* USER CODE BEGIN Header_make_engine_data */
/**
* @brief Function implementing the calculate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_make_engine_data */
void make_engine_data(void *argument)
{
  /* USER CODE BEGIN make_engine_data */
  /* Infinite loop */
  for(;;)
  {

    make_coolant_can_data(coolant_value);
    make_rpm_can_data(input_rpm);
    make_fan_can_data(coolant_value);
  }
  /* USER CODE END make_engine_data */
}

/* USER CODE BEGIN Header_make_immo_data_func */
/**
* @brief Function implementing the make_immo_data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_make_immo_data_func */
void make_immo_data_func(void *argument)
{
  /* USER CODE BEGIN make_immo_data_func */

  /* Infinite loop */
  for(;;)
  {
     if(strcmp(uart_buff , immo_code) == 0){ //valid tag
	 get_time(&_hour, &_minute, &_seconds);
	 char time_string[15];
	 char data_to_send[64];
	 strcpy(time_string , make_string_time(_hour , _minute));
	 strcpy(data_to_send ,make_encrypted_data(time_string));
	// make_decrypted_data(data_to_send);
	 send_encrypted_immo_data(data_to_send);
     }
	// HAL_UART_Transmit(&huart1, (uint8_t *)data_to_send, strlen(data_to_send), HAL_MAX_DELAY);
	// HAL_Delay(1000);
    osDelay(1);
  }
  /* USER CODE END make_immo_data_func */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
