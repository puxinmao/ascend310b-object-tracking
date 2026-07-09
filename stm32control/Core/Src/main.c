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
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 舵机 PWM 参数 */
#define SERVO_PULSE_MIN    500    /* 0°   → 0.5ms */
#define SERVO_PULSE_MAX    2500   /* 180° → 2.5ms */
#define SERVO_RANGE_MIN    0      /* 舵机机械最小角度 */
#define SERVO_RANGE_MAX    180    /* 舵机机械最大角度 */

/* 主循环节奏 — 非阻塞时间片 (ms) */
#define LOOP_PERIOD_MS     20

/* 串口协议帧: [0xAA] [0x55] [pan] [tilt] [track_id] [checksum], checksum=(pan+tilt+id)&0xFF */
#define FRAME_HEAD1        0xAA
#define FRAME_HEAD2        0x55

/* 舵机控制参数 — Python端已做 EMA+增益平滑，STM32端只保留死区+限速 */
#define DEAD_ZONE_PAN      2      /* 水平死区（度） */
#define DEAD_ZONE_TILT     2      /* 俯仰死区（度） */
#define MAX_STEP_PAN       10     /* 水平每周期最大步长（度）：小误差一步到位，大误差限速 */
#define MAX_STEP_TILT      6      /* 俯仰每周期最大步长（度） */
#define PAN_INVERT         1      /* 水平舵机机械方向反向补偿 (1=反向, 0=同向) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* ── 串口接收状态机 ── 帧头同步：丢一字节也能在下一帧自动重新对齐 */
typedef enum {
    RX_WAIT_HEAD1 = 0,
    RX_WAIT_HEAD2,
    RX_WAIT_PAN,
    RX_WAIT_TILT,
    RX_WAIT_ID,
    RX_WAIT_CHK,
} rx_state_t;

volatile rx_state_t rx_state   = RX_WAIT_HEAD1;
volatile uint8_t    rx_byte    = 0;              /* 中断接收临时缓冲 */
volatile uint8_t    rx_buf[3]  = {90, 90, 0};    /* 当前帧暂存 [pan, tilt, track_id] */
volatile uint8_t    rx_bytes[3] = {90, 90, 0};   /* 最近一帧有效数据 [水平, 俯仰, 跟踪ID] */
volatile uint32_t   rx_count   = 0;              /* 已收到的有效帧计数 */
volatile uint32_t   rx_bytes_seen = 0;           /* 已收到的字节计数（LED 诊断用） */
volatile uint32_t   rx_drops   = 0;              /* 校验失败/丢帧计数（调试用） */

int pan_angle  = 90;        /* 当前水平舵机角度 */
int tilt_angle = 90;        /* 当前俯仰舵机角度 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* 启动串口中断接收 — 必须放在 USER CODE 区,否则 CubeMX 重新生成会清掉 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);
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

  /* ── OLED 初始化 + 启动画面 ── */
  OLED_Init();
  OLED_DrawString(0, 0, "PAN:---");
  OLED_DrawString(0, 1, "TLT:---");
  OLED_DrawString(0, 2, "ID:---");
  OLED_DrawString(0, 4, "SERVO READY");
  OLED_Refresh();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_loop_tick = HAL_GetTick();
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* LED 三态故障诊断：
       常亮     = 收到有效帧，系统正常 (rx_count>0)
       慢闪 1Hz = 收到字节但无有效帧（协议不匹配 / 校验失败）
       快闪 5Hz = 完全没收到字节（接线 / 串口号 / 波特率 / 中断问题） */
    static uint32_t led_tick = 0;
    static int led_state = 0;
    if (rx_count > 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);   /* 常亮 */
    } else {
        uint32_t blink_period = (rx_bytes_seen > 0) ? 500 : 100; /* 慢闪 / 快闪 */
        if (now - led_tick >= blink_period) {
            led_tick = now;
            led_state = !led_state;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,
                              led_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
    }

    /* 非阻塞时间片：每 LOOP_PERIOD_MS ms 更新一次舵机，不阻塞中断 */
    if (now - last_loop_tick >= LOOP_PERIOD_MS) {
        last_loop_tick = now;

        /* ═══ 水平：机械方向补偿 → 死区 + 可变步长（EMA 已在 Python 端完成） ═══ */
        int target_pan = PAN_INVERT ? (180 - (int)rx_bytes[0]) : (int)rx_bytes[0];
        int diff_pan = target_pan - pan_angle;
        if (diff_pan > DEAD_ZONE_PAN) {
            pan_angle += (diff_pan > MAX_STEP_PAN ? MAX_STEP_PAN : diff_pan);
        } else if (diff_pan < -DEAD_ZONE_PAN) {
            pan_angle -= (-diff_pan > MAX_STEP_PAN ? MAX_STEP_PAN : -diff_pan);
        }
        if (pan_angle > SERVO_RANGE_MAX) pan_angle = SERVO_RANGE_MAX;
        if (pan_angle < SERVO_RANGE_MIN) pan_angle = SERVO_RANGE_MIN;
        Servo_SetPan(pan_angle);

        /* ═══ 俯仰：死区 + 可变步长 ═══ */
        int target_tilt = (int)rx_bytes[1];
        int diff_tilt = target_tilt - tilt_angle;
        if (diff_tilt > DEAD_ZONE_TILT) {
            tilt_angle += (diff_tilt > MAX_STEP_TILT ? MAX_STEP_TILT : diff_tilt);
        } else if (diff_tilt < -DEAD_ZONE_TILT) {
            tilt_angle -= (-diff_tilt > MAX_STEP_TILT ? MAX_STEP_TILT : -diff_tilt);
        }
        if (tilt_angle > SERVO_RANGE_MAX) tilt_angle = SERVO_RANGE_MAX;
        if (tilt_angle < SERVO_RANGE_MIN) tilt_angle = SERVO_RANGE_MIN;
        Servo_SetTilt(tilt_angle);
    }

    /* OLED 显示:每 ~200ms 刷一次 (10Hz,够人眼看;I2C 阻塞约 24ms 不影响舵机) */
    static uint32_t last_oled_tick = 0;
    if (now - last_oled_tick >= 200) {
        last_oled_tick = now;
        OLED_Clear();
        OLED_DrawString(0, 0, "PAN:");
        OLED_DrawInt(30, 0, pan_angle, 3);
        OLED_DrawString(0, 1, "TLT:");
        OLED_DrawInt(30, 1, tilt_angle, 3);
        int tid = (int)rx_bytes[2];
        if (tid > 0) {
            OLED_DrawString(0, 2, "ID:");
            OLED_DrawInt(24, 2, tid, 3);
        } else {
            OLED_DrawString(0, 2, "ID:---");
        }
        OLED_Refresh();
    }
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
 * @brief  UART 接收完成回调 — 逐字节推进帧解析状态机
 *         帧: [0xAA][0x55][pan][tilt][checksum=(pan+tilt)&0xFF]
 *         任何字节丢失后，状态机在下一帧 0xAA→0x55 处自动重新同步。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t b = rx_byte;
        rx_bytes_seen++;                       /* 任意字节都计数（LED 诊断用） */
        switch (rx_state)
        {
            case RX_WAIT_HEAD1:
                if (b == FRAME_HEAD1) rx_state = RX_WAIT_HEAD2;
                break;
            case RX_WAIT_HEAD2:
                if (b == FRAME_HEAD2) {
                    rx_state = RX_WAIT_PAN;
                } else if (b == FRAME_HEAD1) {
                    rx_state = RX_WAIT_HEAD2;   /* 连续 0xAA，继续等 0x55 */
                } else {
                    rx_state = RX_WAIT_HEAD1;
                }
                break;
            case RX_WAIT_PAN:
                rx_buf[0] = b;
                rx_state = RX_WAIT_TILT;
                break;
            case RX_WAIT_TILT:
                rx_buf[1] = b;
                rx_state = RX_WAIT_ID;
                break;
            case RX_WAIT_ID:
                rx_buf[2] = b;
                rx_state = RX_WAIT_CHK;
                break;
            case RX_WAIT_CHK:
                if (b == (uint8_t)((rx_buf[0] + rx_buf[1] + rx_buf[2]) & 0xFF)) {
                    rx_bytes[0] = rx_buf[0];    /* 校验通过 → 提交有效数据 */
                    rx_bytes[1] = rx_buf[1];
                    rx_bytes[2] = rx_buf[2];
                    rx_count++;
                } else {
                    rx_drops++;                  /* 校验失败，丢弃，等下一帧 */
                }
                rx_state = RX_WAIT_HEAD1;
                break;
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);
    }
}

/**
 * @brief  UART 错误回调 — 出现 ORE/FE/NE 等错误时清标志并重启接收
 *         否则 HAL 在错误后会停止接收，导致永久收不到数据。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;  /* 强制复位，否则 Receive_IT 返回 HAL_BUSY 永久卡死 */
        rx_state = RX_WAIT_HEAD1;               /* 复位状态机，避免半帧残留 */
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
