	// sequence.c — STM2 메인 시퀀스
	//
	// 동작 흐름:
	//   SYS_IDLE        : FF=1 수신 대기 (라파2로부터)
	//   SYS_RUN_CYCLE   : 컨베이어 구동 → IR 감지 → Z 호밍 → Z 고정
	//   SYS_HARVESTING  : HF=1 수신 대기 (라파2가 스카라+매니퓰 R1 취합 후 전송)
	//   SYS_RUN_CYCLE   : Z 복귀 → 트레이 배출
	//   SYS_ERROR       : 긴급정지 또는 타임아웃 — RESET 대기

	#include "sequence.h"
	#include "test.h"
	#include "sec_convey.h"
	#include "stepper.h"
	#include "sensor.h"
	#include "comm.h"
	#include "stm32f4xx_hal.h"

	/* ── 동작 파라미터 (실측 후 수정) ────────────── */
	#define CONVEY_HZ        3000U   // 컨베이어 속도 (step/s)
	#define STOP_SETTLE_MS    200U   // IR 감지 후 정착 딜레이 (ms)
	#define Z_HOMING_HZ      1500U   // Z 호밍 속도 (step/s)
	#define Z_FIX_STEPS      4000    // 리밋 후 하강 스텝 (실측 필요)
	#define Z_FIX_HZ         1500U   // Z 고정 속도 (step/s)
	#define Z_RETURN_STEPS   4000    // 수확 후 상승 스텝
	#define Z_RETURN_HZ      1500U   // Z 복귀 속도 (step/s)
	#define EXIT_MIN_RUN_MS   500U   // 배출 컨베이어 최소 구동 시간 (ms)

	/* ── 타임아웃 ────────────────────────────────── */
	#define TIMEOUT_CONVEY_MS   30000U   // 컨베이어 IR 감지 대기
	#define TIMEOUT_HOMING_MS   15000U   // Z 호밍 리밋 감지 대기
	#define TIMEOUT_MOVE_MS      5000U   // Z 이동 완료 대기
	#define TIMEOUT_HARVEST_MS 120000U   // 수확 완료 신호 대기
	#define TIMEOUT_EXIT_MS     15000U   // 배출 완료 대기

	/* ── SKIP_COMM 시뮬레이션 용 ─────────────────── */
	#define HARVEST_SIM_MS  3000U

	/* ── 시스템 상태 ─────────────────────────────── */
	typedef enum {
		SYS_IDLE,
		SYS_RUN_CYCLE,
		SYS_HARVESTING,
		SYS_ERROR,
	} SystemState;

	/* ── 내부 시퀀스 상태 (SYS_RUN_CYCLE 내부) ───── */
	typedef enum {
		SEQ_CONVEY_RUN,
		SEQ_CONVEY_STOP,
		SEQ_Z_HOMING,
		SEQ_WAIT_Z_FIX,
		SEQ_SEND_HARVEST,
		SEQ_WAIT_Z_RETURN,
		SEQ_EXIT_CONVEY,
		SEQ_CYCLE_DONE,
	} SeqState;

	static SystemState sys_state  = SYS_IDLE;
	static SeqState    seq_state  = SEQ_CONVEY_RUN;
	static uint32_t    timer_start  = 0;
	static uint32_t    settle_start = 0;
	static volatile bool z_limit_hit = false;

	#define TIMER_START()   (timer_start = HAL_GetTick())
	#define TIMED_OUT(ms)   ((HAL_GetTick() - timer_start) >= (ms))

	/* ── 타임아웃 처리 ───────────────────────────── */
	static void Handle_Timeout(uint8_t err_code)
	{
		Conveyor_Stop();
		Conveyor_Enable(false);
		Z_Stop();
		Comm_SendError(err_code);
		Comm_SendState(STATE_ESTOP);
		sys_state = SYS_ERROR;
	}

	/* ── 긴급정지 처리 ───────────────────────────── */
	static void Do_EStop(void)
	{
		Conveyor_Stop();
		Conveyor_Enable(false);
		Z_Stop();
		Comm_SendState(STATE_ESTOP);
		sys_state = SYS_ERROR;
	}

	/* ── 리셋 처리 ───────────────────────────────── */
	static void Do_Reset(void)
	{
		Conveyor_Stop();
		Conveyor_Enable(false);
		Z_Stop();
		Comm_ClearAllFlags();
		z_limit_hit = false;
		seq_state   = SEQ_CONVEY_RUN;
		Comm_SendState(STATE_IDLE);
		sys_state = SYS_IDLE;
	}

	/* ── EXTI 콜백: 리밋스위치 (main.c에서 호출) ─── */
	void Sequence_OnLimitHit(void)
	{
		if (!Z_LimitHit()) return;  // 실제 눌림 확인
		if (sys_state == SYS_RUN_CYCLE && seq_state == SEQ_Z_HOMING) {
			z_limit_hit = true;
			Z_Stop();
		}
	}

	/* ── 초기화 ─────────────────────────────────── */
	void Sequence_Init(void)
	{
	#if TEST_MODE
		Test_Init();
	#else
		Comm_Init();
		Comm_SendState(STATE_IDLE);
	#endif
		sys_state   = SYS_IDLE;
		seq_state   = SEQ_CONVEY_RUN;
		z_limit_hit = false;
	}

	/* ── 메인 루프 (while(1)에서 매 루프 호출) ────── */
	void App_Run(void)
	{
	#if TEST_MODE
		Test_Run();
		return;
	#endif

		/* 긴급정지 최우선 처리 */
		if (Comm_IsEstop()) {
			Comm_ClearEstop();
			Do_EStop();
			return;
		}

		/* 리셋 처리 */
		if (Comm_IsReset()) {
			Comm_ClearReset();
			Do_Reset();
			return;
		}

		switch (sys_state)
		{
			/* ────────────────────────────────────────
			 * SYS_IDLE: FF=1 수신 대기
			 * ──────────────────────────────────────── */
			case SYS_IDLE:
				if (Comm_GetFf() == 1) {
					Comm_SetFf(0);                        // 플래그 소비
					seq_state = SEQ_CONVEY_RUN;
					TIMER_START();
					Conveyor_Enable(true);
					Conveyor_SetDir(true);
					Conveyor_StartHz(CONVEY_HZ);
					Comm_SendState(STATE_CONVEY_RUN);
					sys_state = SYS_RUN_CYCLE;
				}
				break;

			/* ────────────────────────────────────────
			 * SYS_RUN_CYCLE: 컨베이어 → Z호밍 → Z고정 → (수확) → Z복귀 → 배출
			 * ──────────────────────────────────────── */
			case SYS_RUN_CYCLE:
				switch (seq_state)
				{
					/* 컨베이어 구동 → IR 감지 대기 */
					case SEQ_CONVEY_RUN:
						if (IR_Detected()) {
							Conveyor_Stop();
							Conveyor_Enable(false);
							settle_start = HAL_GetTick();
							Comm_SendState(STATE_IR_DETECTED);
							seq_state = SEQ_CONVEY_STOP;
						} else if (TIMED_OUT(TIMEOUT_CONVEY_MS)) {
							Handle_Timeout(ERR_CONVEY_TIMEOUT);
						}
						break;

					/* 정착 딜레이 후 Z 호밍 시작 */
					case SEQ_CONVEY_STOP:
						if ((HAL_GetTick() - settle_start) >= STOP_SETTLE_MS) {
							z_limit_hit = false;
							Z_Enable(true);
							Z_MoveSteps(+1000000, Z_HOMING_HZ);  // CW 리밋 방향
							TIMER_START();
							seq_state = SEQ_Z_HOMING;
						}
						break;

					/* 리밋스위치 감지 대기 (EXTI에서 z_limit_hit=true + Z_Stop) */
					case SEQ_Z_HOMING:
						if (z_limit_hit && !Z_IsBusy()) {
							Z_Enable(true);
							HAL_Delay(10);                        // 리밋 충격 안정화
							Z_MoveSteps(-Z_FIX_STEPS, Z_FIX_HZ); // CCW 고정 위치
							TIMER_START();
							seq_state = SEQ_WAIT_Z_FIX;
						} else if (TIMED_OUT(TIMEOUT_HOMING_MS)) {
							Handle_Timeout(ERR_HOMING_TIMEOUT);
						}
						break;

					/* Z 고정 완료 대기 */
					case SEQ_WAIT_Z_FIX:
						if (!Z_IsBusy()) {
							Comm_SendState(STATE_Z_FIX);
							seq_state = SEQ_SEND_HARVEST;
						} else if (TIMED_OUT(TIMEOUT_MOVE_MS)) {
							Handle_Timeout(ERR_Z_FIX_TIMEOUT);
						}
						break;

					/* STATE_HARVESTING 보고 → 라파2가 스카라+매니퓰에 H1 중계 */
					case SEQ_SEND_HARVEST:
						Comm_SendState(STATE_HARVESTING);
						TIMER_START();
						sys_state = SYS_HARVESTING;   // HF=1 대기 상태로 전환
						break;

					/* Z 복귀 완료 대기 (SYS_HARVESTING → SYS_RUN_CYCLE 복귀 후 처리) */
					case SEQ_WAIT_Z_RETURN:
						if (!Z_IsBusy()) {
							Z_Enable(false);
							Conveyor_Enable(true);
							Conveyor_SetDir(true);
							Conveyor_StartHz(CONVEY_HZ);
							Comm_SendState(STATE_EJECTING);
							TIMER_START();
							seq_state = SEQ_EXIT_CONVEY;
						} else if (TIMED_OUT(TIMEOUT_MOVE_MS)) {
							Handle_Timeout(ERR_Z_RETURN_TIMEOUT);
						}
						break;

					/* IR 해제까지 배출 구동 */
					case SEQ_EXIT_CONVEY:
						if (TIMED_OUT(EXIT_MIN_RUN_MS) && !IR_Detected()) {
							Conveyor_Stop();
							Conveyor_Enable(false);
							Comm_SendState(STATE_EJECT_DONE);
							seq_state = SEQ_CYCLE_DONE;
						} else if (TIMED_OUT(TIMEOUT_EXIT_MS)) {
							Handle_Timeout(ERR_EXIT_TIMEOUT);
						}
						break;

					/* 사이클 완료 → 라파2에 DONE_CYCLE2 송신 → IDLE로 복귀 */
					case SEQ_CYCLE_DONE:
					    /* 사이클 도중 들어온 중복 FF 제거 */
					    Comm_SetFf(0);

					    Comm_SendDone(DONE_CYCLE2);
					    z_limit_hit = false;
					    seq_state   = SEQ_CONVEY_RUN;
					    sys_state   = SYS_IDLE;
					    break;

					default:
						break;
				}
				break;

			/* ────────────────────────────────────────
			 * SYS_HARVESTING: HF=1 수신 대기
			 * 라파2가 스카라+매니퓰레이터 R1을 취합해서 HF=1로 전송
			 * ──────────────────────────────────────── */
			case SYS_HARVESTING:
			#if SKIP_COMM
				/* 테스트 모드: HARVEST_SIM_MS 딜레이로 수확 시뮬레이션 */
				if (TIMED_OUT(HARVEST_SIM_MS)) {
					Z_Enable(true);
					Z_MoveSteps(+Z_RETURN_STEPS, Z_RETURN_HZ);
					TIMER_START();
					seq_state = SEQ_WAIT_Z_RETURN;
					sys_state = SYS_RUN_CYCLE;
				}
			#else
				/* 정상 모드: 라파2로부터 HF=1 수신 대기 */
				if (Comm_IsHfSet()) {
					Comm_ClearHfFlag();
					Z_Enable(true);
					Z_MoveSteps(+Z_RETURN_STEPS, Z_RETURN_HZ);
					TIMER_START();
					seq_state = SEQ_WAIT_Z_RETURN;
					sys_state = SYS_RUN_CYCLE;
				} else if (TIMED_OUT(TIMEOUT_HARVEST_MS)) {
					Handle_Timeout(ERR_HARVEST_TIMEOUT);
				}
			#endif
				break;

			/* ────────────────────────────────────────
			 * SYS_ERROR: RESET 수신 대기 (공통 처리에서 처리됨)
			 * ──────────────────────────────────────── */
			case SYS_ERROR:
				break;

			default:
				sys_state = SYS_IDLE;
				break;
		}
	}
