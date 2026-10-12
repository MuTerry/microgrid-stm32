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
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_AVG_SAMPLES 32U
#define PWM_ARR_VALUE   1399U
#define PI_KP            0.5f
#define PI_KI            0.1f
#define PI_INTEGRAL_MAX  10.0f
#define PI_DUTY_MIN      0.05f
#define PI_DUTY_MAX      0.95f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
char msg[128];
uint32_t last_heartbeat_tick;
uint8_t heartbeat_timeout_reported;
CAN_TxHeaderTypeDef feedbackHeader;
uint32_t feedbackMailbox;
uint8_t feedbackData[8];
uint8_t safe_mode = 0;
uint8_t fault_code = 0;
volatile int32_t pv_power_w = 0;
volatile float vbus_v = 24.0f;
volatile float ibus_a = 0.0f;
static float pi_integral = 0.0f;
static uint32_t last_pi_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Read_Adc_Feedback(void)
{
  uint32_t vbus_raw;
  uint32_t ibus_raw;
  uint32_t vbus10;
  uint32_t ibus10;

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc1);
    return;
  }
  vbus_raw = HAL_ADC_GetValue(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc1);
    return;
  }
  ibus_raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  /* PA0: 0-3.3 V is used as a 0-33.0 V simulated bus voltage. */
  vbus10 = (vbus_raw * 330U + 2047U) / 4095U;
  vbus_v = (float)vbus10 / 10.0f;
  if (vbus_v < 1.0f)
  {
    vbus_v = 1.0f;
  }

  /* PA1: 0-3.3 V is used as a 0-10.0 A simulated current. */
  ibus10 = (ibus_raw * 100U + 2047U) / 4095U;
  ibus_a = (float)ibus10 / 10.0f;
}

static void Update_Single_Channel_PI(void)
{
  uint32_t now;
  float target_current;
  float error;
  float duty;

  now = HAL_GetTick();
  if ((now - last_pi_tick) < 1U)
  {
    return;
  }
  last_pi_tick = now;

  Read_Adc_Feedback();

  if ((safe_mode != 0U) || (pv_power_w <= 0))
  {
    pi_integral = 0.0f;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    return;
  }

  target_current = (float)pv_power_w / vbus_v;
  if (target_current > 10.0f)
  {
    target_current = 10.0f;
  }

  error = target_current - ibus_a;
  pi_integral += error * 0.001f;
  if (pi_integral > PI_INTEGRAL_MAX)
  {
    pi_integral = PI_INTEGRAL_MAX;
  }
  if (pi_integral < -PI_INTEGRAL_MAX)
  {
    pi_integral = -PI_INTEGRAL_MAX;
  }

  duty = PI_KP * error + PI_KI * pi_integral;
  if (duty < PI_DUTY_MIN)
  {
    duty = PI_DUTY_MIN;
  }
  if (duty > PI_DUTY_MAX)
  {
    duty = PI_DUTY_MAX;
  }

  __HAL_TIM_SET_COMPARE(&htim3,
                        TIM_CHANNEL_1,
                        (uint32_t)(duty * (float)PWM_ARR_VALUE));
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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  last_heartbeat_tick = HAL_GetTick();
  heartbeat_timeout_reported = 0;
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0;
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = 0;
  filter.FilterMaskIdLow = 0;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    strcpy(msg, "FILTER ERROR\r\n");
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)msg,
                      strlen(msg),
                      1000);
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    strcpy(msg, "CAN START ERROR\r\n");
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)msg,
                      strlen(msg),
                      1000);
  }

  strcpy(msg, "RECEIVER READY\r\n");
  HAL_UART_Transmit(&huart2,
                    (uint8_t *)msg,
                    strlen(msg),
                    1000);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static uint32_t last_200_tick = 0;
	 	  uint32_t now = HAL_GetTick();

	 	  now = HAL_GetTick();
	 	 if (safe_mode == 0)
	 	 {
	 	  // 原来的接收0x100、0x101、0x102代码
	 	  if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
	 	    {
	 	      if (HAL_CAN_GetRxMessage(&hcan1,
	 	                               CAN_RX_FIFO0,
	 	                               &rxHeader,
	 	                               rxData) == HAL_OK)
	 	      {
	 	        if (rxHeader.StdId == 0x100)
	 	        {

	 	          int16_t code_batt;
	 	          int16_t code_fc;
	 	          int16_t code_el;
	 	          int32_t p_batt;
	 	          int32_t p_fc;
	 	          int32_t p_el;

	 	          code_batt = (int16_t)(((uint16_t)rxData[0] << 8) | rxData[1]);
	 	          code_fc   = (int16_t)(((uint16_t)rxData[2] << 8) | rxData[3]);
	 	          code_el   = (int16_t)(((uint16_t)rxData[4] << 8) | rxData[5]);

	 	          p_batt = ((int32_t)code_batt * 200) / 30000;
	 	          p_fc   = ((int32_t)code_fc * 150) / 30000;
	 	          p_el   = ((int32_t)code_el * 200) / 30000;

	 	         int32_t batt_abs;
	 	         uint32_t ccr_batt;
	 	         uint32_t ccr_fc;

	 	        batt_abs = p_batt;

	 	        if (batt_abs < 0)
	 	        {
	 	          batt_abs = 0;
	 	        }

	 	         if (batt_abs > 200)
	 	         {
	 	           batt_abs = 200;
	 	         }

	 	         if (p_fc < 0)
	 	         {
	 	           p_fc = 0;
	 	         }

	 	         if (p_fc > 150)
	 	         {
	 	           p_fc = 150;
	 	         }

	 	         ccr_batt = ((uint32_t)batt_abs * 1399U) / 200U;
	 	         ccr_fc = ((uint32_t)p_fc * 1399U) / 150U;

	 	         /* CH2=燃料电池，CH3=电池 */
	 	         __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,ccr_fc);
	 	         __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, ccr_batt);

	 	          snprintf(msg,
	 	                   sizeof(msg),
	 	                   "0x100 Batt=%ldW FC=%ldW EL=%ldW Seq=%u\r\n",
	 	                   (long)p_batt,
	 	                   (long)p_fc,
	 	                   (long)p_el,
	 	                   (unsigned int)rxData[7]);

	 	          HAL_UART_Transmit(&huart2,
	 	                            (uint8_t *)msg,
	 	                            strlen(msg),
	 	                            1000);
	 	        }
	 	        else if (rxHeader.StdId == 0x102)
	 	        {
	 	          int16_t code_pv;
	 	          int16_t code_wind;
	 	          int32_t p_pv;
	 	          int32_t p_wind;

	 	          code_pv = (int16_t)(((uint16_t)rxData[0] << 8) | rxData[1]);
	 	          code_wind = (int16_t)(((uint16_t)rxData[2] << 8) | rxData[3]);

	 	          p_pv = ((int32_t)code_pv * 300) / 30000;
	 	          p_wind = ((int32_t)code_wind * 200) / 30000;

	 	         if (p_pv < 0)
	 	         {
	 	           p_pv = 0;
	 	         }
	 	         if (p_pv > 300)
	 	         {
	 	           p_pv = 300;
	 	         }

	 	         if (p_wind < 0)
	 	         {
	 	           p_wind = 0;
	 	         }
	 	         if (p_wind > 200)
	 	         {
	 	           p_wind = 200;
	 	         }

	 	         /* 弃光、弃风标志 */
	 	         if (rxData[6] & 0x01)
	 	         {
	 	           p_pv = 0;
	 	         }

	 	         if (rxData[6] & 0x02)
	 	         {
	 	           p_wind = 0;
	 	         }

	 	        /* CH1=光伏，由单路PI控制；CH4=风电，暂时保持开环 */
	 	        pv_power_w = p_pv;
	 	        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,((uint32_t)p_wind * 1399U) / 200U);

	 	          snprintf(msg,
	 	                   sizeof(msg),
	 	                   "0x102 PV=%ldW Wind=%ldW Curt=0x%02X Seq=%u\r\n",
	 	                   (long)p_pv,
	 	                   (long)p_wind,
	 	                   (unsigned int)rxData[6],
	 	                   (unsigned int)rxData[7]);

	 	          HAL_UART_Transmit(&huart2,
	 	                            (uint8_t *)msg,
	 	                            strlen(msg),
	 	                            1000);
	 	        }
	 	        else if ((rxHeader.StdId == 0x101) &&
	 	                 (rxData[3] == 0xAA))
	 	        {
	 	        	last_heartbeat_tick = HAL_GetTick();
	 	        	heartbeat_timeout_reported = 0;

	 	        	safe_mode = 0;
	 	        	fault_code = 0;

	 	        	strcpy(msg, "HEARTBEAT OK\r\n");
	 	        	HAL_UART_Transmit(&huart2,
	 	        	                  (uint8_t *)msg,
	 	        	                  (uint16_t)strlen(msg),
	 	        	                  1000);
	 	        }

	 	      }
	 	    }
	 	}
	 	else
	 	{
	 	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	 	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	 	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	 	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	 	}

	 	/* 每约1ms运行一次单路PI：PA1反馈，PA6输出 */
	 	Update_Single_Channel_PI();

	 	// 0x200发送代码
	 	if ((now - last_200_tick) >= 250U)
	 	{
	 		last_200_tick = now;

	 		uint32_t vbus_sum = 0;
	 		uint32_t ibus_sum = 0;
	 		uint32_t vbus_raw;
	 		uint32_t ibus_raw;
	 		uint32_t vbus10;
	 		uint32_t ibus10;
	 		int32_t vbus_code;
	 		int32_t ibus_code;
	 		uint32_t sample_count = 0;
	 		uint32_t i;

	 		for (i = 0; i < ADC_AVG_SAMPLES; i++)
	 		{
	 		  if (HAL_ADC_Start(&hadc1) != HAL_OK)
	 		  {
	 		    continue;
	 		  }

	 		  /* Rank 1：PA0，模拟母线电压 */
	 		  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	 		  {
	 		    vbus_sum += HAL_ADC_GetValue(&hadc1);
	 		  }
	 		  else
	 		  {
	 		    HAL_ADC_Stop(&hadc1);
	 		    continue;
	 		  }

	 		  /* Rank 2：PA1，模拟电流 */
	 		  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	 		  {
	 		    ibus_sum += HAL_ADC_GetValue(&hadc1);
	 		    sample_count++;
	 		  }

	 		  HAL_ADC_Stop(&hadc1);
	 		}

	 		if (sample_count == 0U)
	 		{
	 		  sample_count = 1U;
	 		}

	 		vbus_raw = vbus_sum / sample_count;
	 		ibus_raw = ibus_sum / sample_count;

	 		/* PA0：0~3.3V模拟0~33.0V */
	 		vbus10 = (vbus_raw * 330U + 2047U) / 4095U;
	 		vbus_code = (int32_t)vbus10 - 150;

	 		if (vbus_code < 0)
	 		{
	 		  vbus_code = 0;
	 		}

	 		if (vbus_code > 255)
	 		{
	 		  vbus_code = 255;
	 		}

	 		/*
	 		 * PA1：暂时模拟0~10.0A
	 		 * ibus10单位为0.1A
	 		 */
	 		ibus10 = (ibus_raw * 100U + 2047U) / 4095U;

	 		/*
	 		 * 0x200 Ibus_raw = round(Ibus * 5) + 127
	 		 */
	 		ibus_code = 127 + (int32_t)((ibus10 + 1U) / 2U);

	 		if (ibus_code < 0)
	 		{
	 		  ibus_code = 0;
	 		}

	 		if (ibus_code > 255)
	 		{
	 		  ibus_code = 255;
	 		}

	 	  feedbackHeader.StdId = 0x200;
	 	  feedbackHeader.ExtId = 0;
	 	  feedbackHeader.IDE = CAN_ID_STD;
	 	  feedbackHeader.RTR = CAN_RTR_DATA;
	 	  feedbackHeader.DLC = 8;
	 	  feedbackHeader.TransmitGlobalTime = DISABLE;

	 	  feedbackData[0] = 0x55;
	 	  feedbackData[1] = (uint8_t)vbus_code;
	 	  feedbackData[2] = (uint8_t)ibus_code;
	 	  feedbackData[3] = 0;
	 	  feedbackData[4] = 0;
	 	  feedbackData[5] = 50;

	 	  if (fault_code != 0U)
	 	  {
	 	    feedbackData[6] = 0x0A;
	 	  }
	 	  else
	 	  {
	 	    feedbackData[6] = 0x00;
	 	  }

	 	  feedbackData[7] = fault_code;

	 	  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
	 	  {
	 	    HAL_CAN_AddTxMessage(&hcan1,
	 	                         &feedbackHeader,
	 	                         feedbackData,
	 	                         &feedbackMailbox);
	 	  }
	 	}
	 	  // 原来的心跳超时检测
	 	 if ((HAL_GetTick() - last_heartbeat_tick > 1000U) &&
	 	     (heartbeat_timeout_reported == 0))
	 	 {
	 	   heartbeat_timeout_reported = 1;
	 	   safe_mode = 1;
	 	   fault_code = 5;

	 	   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	 	   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	 	   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	 	   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

	 	   strcpy(msg, "HEARTBEAT TIMEOUT\r\n");

	 	   HAL_UART_Transmit(&huart2,
	 	                     (uint8_t *)msg,
	 	                     (uint16_t)strlen(msg),
	 	                     1000);
	 	 }

	 	    HAL_Delay(1);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
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
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
