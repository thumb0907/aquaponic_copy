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
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP_PUL_PORT			GPIOA
#define STEP_PUL_PIN  		 	GPIO_PIN_0   // 예시: PUL- 연결 핀
#define STEP_DIR_PORT			GPIOC
#define STEP_DIR_PIN			GPIO_PIN_11

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile uint32_t home_z = 0;     //원점(z)
volatile uint32_t step_cnt = 0;
volatile uint32_t step_target = 4000; // 목표 카운트
volatile uint8_t done = 0;
int32_t tol_cnt    = 2;         // 허용 오차(±2카운트 등)
volatile uint32_t last_home_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ===================== User Tunables =====================
#define CTRL_DT_MS          2
#define PPS_MAX             3000
#define PPS_MIN             150
#define ACC_PPS_PER_TICK    80
#define TOL_CNT             2          // 허용오차
#define HOME_PPS            300        // 원점 탐색 속도
#define CALIB_PPS           300        // 방향 보정 속도
#define CALIB_MS            200        // 방향 보정 시간

// P 제어 게인 (pps = abs(err) * KP_NUM / KP_DEN)
#define KP_NUM              1
#define KP_DEN              3
// =========================================================

// ====== Encoder (wrap 확장) ======
static int32_t  enc_pos  = 0;
static uint16_t enc_prev = 0;

static void Encoder_Reset(void)
{
    enc_prev = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    enc_pos  = 0;
}

static int32_t Encoder_GetPos(void)
{
    uint16_t now  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t  diff = (int16_t)(now - enc_prev);  // wrap 자동 처리
    enc_prev = now;
    enc_pos += diff;
    return enc_pos;
}

// ====== STEP PWM ======
static inline void Step_PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

static inline void Step_PWM_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

// pps=0이면 stop (중요!)
static void Step_SetPPS(uint32_t pps)
{
	if (pps == 0)
	{
	        Step_PWM_Stop();          // ✅ 진짜 정지
	        return;
    }

    if (pps < 1) pps = 1;

    // TIM2 tick = 1MHz(PSC=83 가정)
    uint32_t arr = (1000000UL / pps) - 1;
    uint32_t ccr = (arr + 1) / 2;

    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_ENABLE(&htim2);

    Step_PWM_Start();             //  혹시 Stop 돼있어도 자동 재시작
}

// ====== DIR ======
static uint8_t dir_invert = 0;

static void Step_SetDir(uint8_t dir)
{
    if (dir_invert) dir ^= 1;
    HAL_GPIO_WritePin(STEP_DIR_PORT, STEP_DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ====== Control helpers ======
static uint32_t RampTo(uint32_t now, uint32_t target)
{
    if (now < target) {
        uint32_t up = now + ACC_PPS_PER_TICK;
        return (up > target) ? target : up;
    } else if (now > target) {
        uint32_t down = (now > ACC_PPS_PER_TICK) ? (now - ACC_PPS_PER_TICK) : 0;
        return (down < target) ? target : down;
    }
    return now;
}

static uint32_t PpsFromErr(int32_t aerr)
{
    uint32_t pps = (uint32_t)((aerr * KP_NUM) / KP_DEN);
    if (pps > PPS_MAX) pps = PPS_MAX;
    if (pps < PPS_MIN) pps = PPS_MIN;
    return pps;
}

// ====== Dir invert auto calib ======
static void Calib_DirInvert(void)
{
    dir_invert = 0;
    Step_SetDir(1);

    Encoder_Reset();
    Step_PWM_Start();
    Step_SetPPS(CALIB_PPS);
    HAL_Delay(CALIB_MS);
    Step_SetPPS(0);
    Step_PWM_Stop();

    int32_t moved = Encoder_GetPos(); // 누적값
    if (moved < 0) dir_invert = 1;
}

// ====== Home routine (home_z는 EXTI에서 1로) ======
static void Home_Find(void)
{
    home_z = 0;
    Step_SetDir(0);

    Step_PWM_Start();              // 여기서 시작
    Step_SetPPS(HOME_PPS);

    while (!home_z) { /* wait */ }

    Step_SetPPS(0);

    Step_PWM_Stop();               // 여기서 정지
    HAL_Delay(100);

    Encoder_Reset(); // 원점=0 확정
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
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // 완료 LED OFF

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  Encoder_Reset();

  // 1) 원점 찾기
  Home_Find();

  // 2) DIR 증가방향 자동 보정 (원점 근처에서 짧게 테스트)
  Calib_DirInvert();
  Encoder_Reset();  // 보정 끝났으면 다시 0

  // 3) 목표 이동 (B 방식: 램프+P제어)
  uint32_t pps_now = 0;
  uint32_t next_tick = HAL_GetTick() + CTRL_DT_MS;
  Step_PWM_Start();
  while (1)
  {
      // 고정 주기(2ms)
      if ((int32_t)(HAL_GetTick() - next_tick) < 0) continue;
      next_tick += CTRL_DT_MS;

      int32_t cnt = Encoder_GetPos();
      int32_t err = (int32_t)step_target - cnt;
      int32_t aerr = (err >= 0) ? err : -err;

      // 목표 도착?
      if (aerr <= TOL_CNT)
      {
          pps_now = RampTo(pps_now, 0);
          Step_SetPPS(pps_now);

          if (pps_now == 0)
          {
        	  	  Step_PWM_Stop();
        	  	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
                  break;
          }
          continue;
      }

      // 방향
      Step_SetDir(err > 0 ? 1 : 0);

      // 속도 명령 + 램프
      uint32_t pps_cmd = PpsFromErr(aerr);
      pps_now = RampTo(pps_now, pps_cmd);
      Step_SetPPS(pps_now);
  }
  /* USER CODE END 2 */

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
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000-1;
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
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_6)
    {
        uint32_t now = HAL_GetTick();
        if (now - last_home_tick < 50) return; // 50ms 디바운스
        last_home_tick = now;

        if(home_z == 0)
        {
            home_z = 1;
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
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
