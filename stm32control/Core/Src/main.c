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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 舵机 PWM 参数 */
#define SERVO_PULSE_MIN    500    /* 0°   → 0.5ms */
#define SERVO_PULSE_MAX    2500   /* 180° → 2.5ms */
#define STEP_DELAY_MS      50     /* 主循环周期 (ms) */
#define SERVO_RANGE_MIN    0      /* 舵机最小角度（扩展至 0°，原 30°） */
#define SERVO_RANGE_MAX    180    /* 舵机最大角度（扩展至 180°，原 150°） */

#define SPEED_MAX          8      /* 最大速度（度/步），实际使用 STEP_DEGREE */

/* 防抖参数 — 提速版：Python端已做平滑，STM32端适当放宽 */
#define FILTER_ALPHA       45     /* 输入滤波系数 0~100 (越大响应越快) */
#define DEAD_ZONE_PAN      2      /* 水平死区（度） */
#define DEAD_ZONE_TILT     2      /* 俯仰死区（度） */
#define STEP_DEGREE        4      /* 每步移动度数 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t rx_bytes[2] = {0x5A, 0x5A}; /* [0]=水平, [1]=俯仰 */
volatile uint8_t rx_byte   = 0;              /* 中断接收临时缓冲 */
volatile uint8_t rx_toggle = 0;              /* 0=收水平, 1=收俯仰 */
volatile uint8_t rx_count  = 0;              /* 收到字节计数器（用于调试） */
uint8_t  pan_angle  = 90;        /* 当前水平舵机角度 */
uint16_t pan_filter_x10  = 900;  /* 水平滤波目标×10 */
uint8_t  tilt_angle = 90;        /* 当前俯仰舵机角度 */
uint16_t tilt_filter_x10 = 900;  /* 俯仰滤波目标×10 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Servo_SetPan(int angle);
static void Servo_SetTilt(int angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  设置水平舵机角度 (TIM2_CH1)
 */
static void Servo_SetPan(int angle)
{
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    uint32_t pulse = SERVO_PULSE_MIN + (uint32_t)((float)angle / 180.0f * (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

/**
 * @brief  设置俯仰舵机角度 (TIM2_CH2)
 */
static void Servo_SetTilt(int angle)
{
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    uint32_t pulse = SERVO_PULSE_MIN + (uint32_t)((float)angle / 180.0f * (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* 立即启动串口中断接收 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);
  /* USER CODE BEGIN 2 */
  /* LED 闪烁 2 次 — 指示芯片已启动 */
  for (int i = 0; i < 2; i++) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); /* 亮 */
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   /* 灭 */
    HAL_Delay(200);
  }

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* ── 舵机上电自检（逐个测，避开极端位置防供电不足）── */
  Servo_SetTilt(90);
  Servo_SetPan(30);  HAL_Delay(500);
  Servo_SetPan(150); HAL_Delay(500);
  Servo_SetPan(90);  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* LED: 收到数据就常亮 */
    if (rx_count > 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }

    /* ═══ 水平：滤波 + 死区 + 加减速 ═══ */
    uint8_t raw_pan = (uint8_t)(180 - ((uint32_t)rx_bytes[0] * 180 / 0xB4));
    raw_pan = SERVO_RANGE_MIN + (uint8_t)((uint32_t)raw_pan * (SERVO_RANGE_MAX - SERVO_RANGE_MIN) / 180);
    pan_filter_x10 = (uint16_t)(((uint32_t)raw_pan * FILTER_ALPHA * 10
                     + pan_filter_x10 * (100 - FILTER_ALPHA)) / 100);
    int diff_pan = (int)(pan_filter_x10 / 10) - (int)pan_angle;
    if      (diff_pan >  DEAD_ZONE_PAN) pan_angle += STEP_DEGREE;
    else if (diff_pan < -DEAD_ZONE_PAN) pan_angle -= STEP_DEGREE;
    Servo_SetPan(pan_angle);

    /* ═══ 俯仰：滤波 + 死区 + 加减速 ═══ */
    uint8_t raw_tilt = (uint8_t)((uint32_t)rx_bytes[1] * 180 / 0xB4);
    raw_tilt = SERVO_RANGE_MIN + (uint8_t)((uint32_t)raw_tilt * (SERVO_RANGE_MAX - SERVO_RANGE_MIN) / 180);
    tilt_filter_x10 = (uint16_t)(((uint32_t)raw_tilt * FILTER_ALPHA * 10
                      + tilt_filter_x10 * (100 - FILTER_ALPHA)) / 100);
    int diff_tilt = (int)(tilt_filter_x10 / 10) - (int)tilt_angle;
    if      (diff_tilt >  DEAD_ZONE_TILT) tilt_angle += STEP_DEGREE;
    else if (diff_tilt < -DEAD_ZONE_TILT) tilt_angle -= STEP_DEGREE;
    Servo_SetTilt(tilt_angle);

    HAL_Delay(50);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  /* PC13 板载 LED — 芯片运行指示灯 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* 初始灭 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  UART 接收完成回调
 *         每收到 1 字节，交替存入 rx_bytes[0]（水平）和 rx_bytes[1]（俯仰）
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_toggle == 0) {
            rx_bytes[0] = rx_byte;  /* 第 1 字节 → 水平 */
            rx_toggle = 1;
        } else {
            rx_bytes[1] = rx_byte;  /* 第 2 字节 → 俯仰 */
            rx_toggle = 0;
        }
        rx_count++;
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);
    }
}

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
