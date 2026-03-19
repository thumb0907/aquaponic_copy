	// first_convey.c
	#include "stm32f4xx_hal.h"
	#include "first_convey.h"
	#include "board_pin.h"
	#include "sensor.h"
	#include "main.h"
	#include <stdio.h>

	extern TIM_HandleTypeDef  htim3; // htim3, huart2는 main.c에 실제로 생성(정의) 되어 있고, 여기서는 “그걸 가져다 쓰겠다”는 선언만 한 것(extern).

	// 파종관련파라미터
	static volatile int32_t convey_remain = 0;
	static volatile bool    convey_moving = false;
	// 컨베이어 steps/mm (실측 후 조정)
	// 1600pulse / 20mm(한바퀴 거리) = 80
	static float g_con_steps_per_mm = 80.0f;
	static uint32_t g_con_run_hz = 4500;

	typedef enum { CONVEYOR_RUN = 0, CONVEYOR_STOP } ConveyorState;
	static ConveyorState state = CONVEYOR_RUN; 	// 현재 상태: 컨베이어 구동중
	static bool motor_running = false; 			// 이미 모터를 켰는지 기억하는 플래그

	// TIM3는 APB1 타이머. prescaler가 1이 아니면 TIM clk = PCLK1*2
	static uint32_t TIM3_GetClockHz(void) 			// 타이머 클럭 계산
	{
	  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
	  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) return pclk1 * 2;
	  return pclk1;
	}
	static void StepPWM_SetHz(uint32_t hz)      	// PWM 주파수 설정
	{
	  if (hz < 1) hz = 1;

	  uint32_t tim_clk = TIM3_GetClockHz();
	  uint32_t arr = (tim_clk / hz) - 1;
	  if (arr > 0xFFFF) arr = 0xFFFF;

	  __HAL_TIM_SET_PRESCALER(&htim3, 0);
	  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr / 2); // 50%
	  __HAL_TIM_SET_COUNTER(&htim3, 0);
	}

	//////////////////////////////////////////////// 컨베이어동작
	static void Motor_SetDir(bool dir) // dir
	{
	  HAL_GPIO_WritePin(CON_DIR_PORT, CON_DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
	static void Motor_Enable(bool en) // en
	{
	  HAL_GPIO_WritePin(CON_EN_PORT, CON_EN_PIN, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
	static void Motor_Start(uint32_t step_hz) // motor start
	{
	  StepPWM_SetHz(step_hz);
	  ////////// 컨베이어 파라미터 찾는용도
	  // Update(ARR overflow) 인터럽트로 펄스 카운트하기 위해 Base IT도 시작
	    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
	    HAL_TIM_Base_Start_IT(&htim3);
	  ///////////////////////////////////
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	}
	static void Motor_Stop(void) // motor stop
	{
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_Base_Stop_IT(&htim3); // 컨베이어 파라미터 찾는용도
	}
	void FirstConvey_SendPulses(uint32_t pulses) // 컨베이어 파라미터 찾는용도
	{
	    if (pulses == 0) return;

	    Motor_SetDir(true);
	    Motor_Enable(true);

	    convey_remain = (int32_t)pulses;
	    convey_moving = true;

	    Motor_Start(g_con_run_hz);
	}
	static void Conveyor_Run(void) // 컨베이어 구동
	{
	  if (!motor_running)
	  {
		Motor_SetDir(true);
		Motor_Enable(true);
		Motor_Start(4500);
		motor_running = true;
	  }

	  if (Sensor_IR_Detected()) // 컨베이어 정지
	  {
		Motor_Stop();
		Motor_Enable(false);
		motor_running = false;
		state = CONVEYOR_STOP;
	  }
	}
	void FirstConvey_Reset(void)
	{
		state = CONVEYOR_RUN;
		motor_running = false;
		Motor_Stop();
		Motor_Enable(false);

		convey_moving = false;
		convey_remain = 0;
	}
	static void Conveyor_Stop(void) // 컨베이어 정지
	{
	}

	void FirstConvey_Task(void) // 컨베이어 구동/정지 함수 (main에서 호출)
	{
	  if (state == CONVEYOR_RUN)  Conveyor_Run();
	  else                        Conveyor_Stop();
	}

	bool FirstConvey_IsStopped(void)
	{
	  return (state == CONVEYOR_STOP);
	}

//////////////// 파종할때 컨베이어 움직임
	void FirstConvey_MoveDistance(float mm)
	{
	    if (mm <= 0.0f) return;
	    int32_t steps = (int32_t)(mm * g_con_steps_per_mm);
	    Motor_SetDir(true);
	    Motor_Enable(true);
	    convey_remain = steps;
	    convey_moving = true;
	    Motor_Start(g_con_run_hz);
	}

	bool FirstConvey_IsMoveDone(void)
	{
	    return !convey_moving;
	}

	void FirstConvey_ForceStop(void)
	{
	    Motor_Stop();
	    Motor_Enable(false);
	    convey_moving = false;
	    convey_remain = 0;
	    motor_running = false;
	    state = CONVEYOR_STOP;
	}

	void FirstConvey_OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
	{
	    if (htim->Instance == TIM3 && convey_moving) {
	        if (convey_remain > 0) {
	            convey_remain--;
	            if (convey_remain == 0) {
	                Motor_Stop();
	                Motor_Enable(false);
	                convey_moving = false;
	            }
	        }
	    }
	}
