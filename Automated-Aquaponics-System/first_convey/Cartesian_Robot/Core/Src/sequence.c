/*
 * sequence.c  ─  STM32 #1 메인 시퀀스
 *
 * Comm_SendState / Comm_SendDone / Comm_SendError 가
 * 이제 uint8_t 값(바이너리)을 인자로 받음.
 * comm.h의 STATE_xxx / DONE_xxx / ERR_xxx 상수 사용.
 */

#include "sequence.h"
#include "comm.h"
#include "cartesian.h"
#include "first_convey.h"
#include "test1.h"

static SystemState    sys_state       = SYS_IDLE;
static CartesianStage prev_cart_stage = CART_STAGE_UNKNOWN;
static bool scara_crf_set = false;

static void Sequence_EnterState(SystemState next) { sys_state = next; }

/* ══════════════════════════════════════════════
 * 초기화
 * ══════════════════════════════════════════════ */
void Sequence_Init(void)
{
    sys_state       = SYS_IDLE;
    prev_cart_stage = CART_STAGE_UNKNOWN;

    Cartesian_ResetSequence();
    FirstConvey_Reset();
    Comm_ClearAllFlags();

    Comm_SendState(STATE_IDLE);
}

SystemState Sequence_GetState(void) { return sys_state; }
bool        Sequence_IsError(void)  { return (sys_state == SYS_ERROR); }

/* ══════════════════════════════════════════════
 * 긴급정지 / 리셋
 * ══════════════════════════════════════════════ */
void Sequence_EStop(void)
{
	scara_crf_set = false;
    FirstConvey_ForceStop();
    Cartesian_TestStopX();
    Cartesian_TestStopZ();
    Cartesian_ResetSequence();

    prev_cart_stage = CART_STAGE_UNKNOWN;
    Comm_SendState(STATE_ESTOP);
    Sequence_EnterState(SYS_ERROR);
}

void Sequence_ResetError(void)
{
	scara_crf_set = false;
    Cartesian_ResetSequence();
    FirstConvey_Reset();
    Comm_ClearAllFlags();

    prev_cart_stage = CART_STAGE_UNKNOWN;
    Comm_SendState(STATE_IDLE);
    Sequence_EnterState(SYS_IDLE);
}

/* ══════════════════════════════════════════════
 * 직교로봇 단계 변화 → 라파로 STATE 전송
 * ══════════════════════════════════════════════ */
static void Sequence_ReportCartesianStage(void)
{
    CartesianStage now = Cartesian_GetStage();
    if (now == prev_cart_stage) return;
    prev_cart_stage = now;

    switch (now) {
        case CART_STAGE_RUN_CONVEY:
            Comm_SendState(STATE_RUN_CONVEYOR1);
            break;
        case CART_STAGE_SEEDING:
            Comm_SendState(STATE_SEEDING);
            break;
        case CART_STAGE_EJECTING:
            Comm_SendState(STATE_EJECTING);
            break;
        case CART_STAGE_PICKING:
            Comm_SendState(STATE_PICKING);
            break;
        default:
            break;
    }
}

/* ══════════════════════════════════════════════
 * 메인 태스크
 * ══════════════════════════════════════════════ */
void Sequence_Task(void)
{
    /* 긴급정지 / 리셋 최우선 */
    if (Comm_IsEstop()) {
        Comm_ClearEstop();
        Sequence_EStop();
        return;
    }
    if (Comm_IsReset()) {
        Comm_ClearReset();
        Sequence_ResetError();
        return;
    }

    switch (sys_state)
    {
        case SYS_IDLE:
        	if (Comm_GetC1f() == 1) {   // cam1detected → c1f
        	    Comm_SetC1f(0);          // 클리어

                Cartesian_ResetSequence();
                FirstConvey_Reset();
                prev_cart_stage = CART_STAGE_UNKNOWN;

                Comm_SendState(STATE_HOMING);
                Cartesian_StartHoming();
                Sequence_EnterState(SYS_HOMING);
            }
            break;

        case SYS_HOMING:
            Cartesian_HomingTask();

            if (Cartesian_IsHomingError()) {
                Comm_SendError(ERR_HOMING_TIMEOUT);
                Sequence_EnterState(SYS_ERROR);
            }
            else if (Cartesian_IsHomingDone()) {
                Comm_SendDone(DONE_HOMING);

                Cartesian_PrepareCycleAfterHoming();
                FirstConvey_Reset();
                prev_cart_stage = CART_STAGE_UNKNOWN;

                Sequence_EnterState(SYS_RUN_CYCLE);
            }
            break;


        case SYS_RUN_CYCLE:
            Cartesian_Task();
            Sequence_ReportCartesianStage();

            if (Cartesian_IsCycleDone()) {
#if FULL_SEQ_TEST
                /* 테스트: 스카라 없이 바로 완료 */
                Comm_SendDone(DONE_CYCLE1);
                Comm_SendState(STATE_IDLE);

                Cartesian_ResetSequence();
                FirstConvey_Reset();
                Comm_ClearAllFlags();

                Sequence_EnterState(SYS_IDLE);
#else
                //파종 완료
                Comm_SendDone(DONE_CYCLE1);
                Comm_SendState(STATE_WAIT_SCARA_PICK);
                Sequence_EnterState(SYS_WAIT_SCARA_PICK);
#endif
            }
            break;

        /* SYS_WAIT_SCARA_PICK: crf=1 받은 후 crf=0 수신 시 초기화 */
        case SYS_WAIT_SCARA_PICK:
            if (Comm_GetCrf() == 1 && !scara_crf_set) {
                // crf=1 수신 확인 — 이제 crf=0 기다릴 준비
                scara_crf_set = true;
            }
            if (scara_crf_set && Comm_GetCrf() == 0) {
                scara_crf_set = false;
                Comm_SetC1f(0);
                Cartesian_ResetSequence();
                FirstConvey_Reset();
                Comm_SendState(STATE_IDLE);
                Sequence_EnterState(SYS_IDLE);
            }
            break;





        case SYS_ERROR:
        default:
            break;
    }
}
