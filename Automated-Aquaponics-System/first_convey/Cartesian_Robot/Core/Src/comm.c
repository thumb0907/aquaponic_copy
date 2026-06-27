/*
 * comm.c  ─  STM32 #1 통신 레이어
 */

#include "comm.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart2;  // 라즈베리파이1

/* ── 수신 상태머신 ──────────────────────────── */
typedef enum {
    RX_WAIT_SOF = 0,
    RX_WAIT_ID,
    RX_WAIT_LEN,
    RX_WAIT_DATA,
    RX_WAIT_CHK
} RxState;

/* 라파1 수신 */
static uint8_t  g_rasp_rx_byte = 0;
static RxState  g_rasp_state   = RX_WAIT_SOF;
static uint8_t  g_rasp_id      = 0;
static uint8_t  g_rasp_len     = 0;
static uint8_t  g_rasp_idx     = 0;
static uint8_t  g_rasp_data[COMM_MAX_DATA_LEN];

/* ── 공유 플래그 ────────────────────────────── */
static uint8_t  g_ssf  = 0;
static uint8_t  g_smf  = 0;
static uint8_t  g_crf  = 0;
static uint16_t g_uv   = 0;
static uint8_t  g_ulf  = 0;
static uint8_t  g_urf  = 0;
static uint16_t  g_wcnt = 0;
static uint8_t  g_wlf  = 0;
static uint8_t  g_wrf  = 0;
static uint8_t  g_ff   = 0;
static uint8_t  g_uef  = 0;
static uint8_t  g_wef  = 0;
static uint8_t  g_hf   = 0;
static uint8_t  g_c1f  = 0;
static uint8_t  g_c2f  = 0;

/* 이벤트 플래그 */
static volatile bool g_cam1_detected = false;
static volatile bool g_estop         = false;
static volatile bool g_reset         = false;

/* ── 체크섬 ─────────────────────────────────── */
static uint8_t calc_chk(uint8_t id, uint8_t len, const uint8_t *data)
{
    uint16_t s = id + len;
    for (uint8_t i = 0; i < len; i++) s += data[i];
    return (uint8_t)(s & 0xFF);
}

/* ── 프레임 송신 ────────────────────────────── */
static void send_frame(UART_HandleTypeDef *huart,
                       uint8_t id, const uint8_t *data, uint8_t len)
{
    uint8_t pkt[3 + COMM_MAX_DATA_LEN + 1];
    uint8_t p = 0;
    pkt[p++] = COMM_SOF;
    pkt[p++] = id;
    pkt[p++] = len;
    for (uint8_t i = 0; i < len; i++) pkt[p++] = data[i];
    pkt[p++] = calc_chk(id, len, data);
    HAL_UART_Transmit(huart, pkt, p, 100);
}

/* ── 수신 프레임 처리 ───────────────────────── */
static void handle_frame(uint8_t id, const uint8_t *data, uint8_t len)
{
    switch (id) {
        /* 트레이 감지 이벤트 (cam1 감지) */
        case PID_C1F:
            if (len == 1) {
                g_c1f = data[0];
                /* c1f=1 이면 cam1 감지와 동일하게 처리 */
                if (g_c1f == 1) g_cam1_detected = true;
            }
            break;


        case PID_SMF:   if (len == 1) g_smf  = data[0]; break;
        case PID_CRF:   if (len == 1) g_crf  = data[0]; break;
        case PID_UV:
            if (len == 2) g_uv = ((uint16_t)data[0] << 8) | data[1];
            break;
        case PID_ULF:   if (len == 1) g_ulf  = data[0]; break;
        case PID_URF:   if (len == 1) g_urf  = data[0]; break;
        case PID_WCNT:
            if (len == 2) g_wcnt = ((uint16_t)data[0] << 8) | data[1];
            break;
        case PID_WLF:   if (len == 1) g_wlf  = data[0]; break;
        case PID_WRF:   if (len == 1) g_wrf  = data[0]; break;
        case PID_FF:    if (len == 1) g_ff   = data[0]; break;
        case PID_UEF:   if (len == 1) g_uef  = data[0]; break;
        case PID_WEF:   if (len == 1) g_wef  = data[0]; break;
        case PID_HF:    if (len == 1) g_hf   = data[0]; break;
        //case PID_C1F:   if (len == 1) g_c1f  = data[0]; break;
        case PID_C2F:   if (len == 1) g_c2f  = data[0]; break;
        case PID_ESTOP:
            if (len == 0) g_estop = true;
            break;

        case PID_RESET:
            if (len == 0) g_reset = true;
            break;
        default: break;
    }
}

/* ── 수신 상태머신 공통 처리 ────────────────── */
static void process_byte(uint8_t b, RxState *state,
                          uint8_t *id, uint8_t *len,
                          uint8_t *idx, uint8_t *data)
{
    switch (*state) {
        case RX_WAIT_SOF:
            if (b == COMM_SOF) *state = RX_WAIT_ID;
            break;
        case RX_WAIT_ID:
            *id    = b;
            *state = RX_WAIT_LEN;
            break;
        case RX_WAIT_LEN:
            *len = b;
            if (*len > COMM_MAX_DATA_LEN) {
                *state = RX_WAIT_SOF;
            } else if (*len == 0) {
                *state = RX_WAIT_CHK;
            } else {
                *idx   = 0;
                *state = RX_WAIT_DATA;
            }
            break;
        case RX_WAIT_DATA:
            data[(*idx)++] = b;
            if (*idx >= *len) *state = RX_WAIT_CHK;
            break;
        case RX_WAIT_CHK: {
            uint8_t calc = calc_chk(*id, *len, data);
            if (calc == b) handle_frame(*id, data, *len);
            *state = RX_WAIT_SOF;
            *idx   = 0;
            break;
        }
    }
}

/* ── 공개 API ───────────────────────────────── */
void Comm_Init(void)
{
    g_rasp_state = RX_WAIT_SOF;
    Comm_ClearAllFlags();
    HAL_UART_Receive_IT(&huart2, &g_rasp_rx_byte, 1);
}

void Comm_Rasp_RxCallback(void)
{
    process_byte(g_rasp_rx_byte,
                 &g_rasp_state, &g_rasp_id, &g_rasp_len,
                 &g_rasp_idx,    g_rasp_data);
    HAL_UART_Receive_IT(&huart2, &g_rasp_rx_byte, 1);
}

/* ── getter / setter ────────────────────────── */
uint8_t  Comm_GetSsf(void)          { return g_ssf;  }
void     Comm_SetSsf(uint8_t v)     { g_ssf  = v;    }
uint8_t  Comm_GetSmf(void)          { return g_smf;  }
void     Comm_SetSmf(uint8_t v)     { g_smf  = v;    }
uint8_t  Comm_GetCrf(void)          { return g_crf;  }
void     Comm_SetCrf(uint8_t v)     { g_crf  = v;    }
uint16_t Comm_GetUv(void)           { return g_uv;   }
void     Comm_SetUv(uint16_t v)     { g_uv   = v;    }
uint8_t  Comm_GetUlf(void)          { return g_ulf;  }
void     Comm_SetUlf(uint8_t v)     { g_ulf  = v;    }
uint8_t  Comm_GetUrf(void)          { return g_urf;  }
void     Comm_SetUrf(uint8_t v)     { g_urf  = v;    }
uint16_t  Comm_GetWcnt(void)        { return g_wcnt; }
void     Comm_SetWcnt(uint16_t v)   { g_wcnt = v;    }
uint8_t  Comm_GetWlf(void)          { return g_wlf;  }
void     Comm_SetWlf(uint8_t v)     { g_wlf  = v;    }
uint8_t  Comm_GetWrf(void)          { return g_wrf;  }
void     Comm_SetWrf(uint8_t v)     { g_wrf  = v;    }
uint8_t  Comm_GetFf(void)           { return g_ff;   }
void     Comm_SetFf(uint8_t v)      { g_ff   = v;    }
uint8_t  Comm_GetUef(void)          { return g_uef;  }
void     Comm_SetUef(uint8_t v)     { g_uef  = v;    }
uint8_t  Comm_GetWef(void)          { return g_wef;  }
void     Comm_SetWef(uint8_t v)     { g_wef  = v;    }
uint8_t  Comm_GetHf(void)           { return g_hf;   }
void     Comm_SetHf(uint8_t v)      { g_hf   = v;    }
uint8_t  Comm_GetC1f(void)          { return g_c1f;  }
void     Comm_SetC1f(uint8_t v)     { g_c1f  = v;    }
uint8_t  Comm_GetC2f(void)          { return g_c2f;  }
void     Comm_SetC2f(uint8_t v)     { g_c2f  = v;    }

bool Comm_IsEstop(void)             { return g_estop;        }
void Comm_ClearEstop(void)          { g_estop = false;       }
bool Comm_IsReset(void)             { return g_reset;        }
void Comm_ClearReset(void)          { g_reset = false;       }
bool Comm_IsCam1Detected(void)      { return g_cam1_detected;}
void Comm_ClearCam1Detected(void)   { g_cam1_detected = false; g_ssf = 0; }

void Comm_ClearAllFlags(void)
{
	g_ssf  = 0;
	g_smf  = 0;
	g_crf  = 0;
	g_uv   = 0;
	g_ulf  = 0;
	g_urf  = 0;
	g_wcnt = 0;
	g_wlf  = 0;
	g_wrf  = 0;
	g_ff   = 0;
	g_uef  = 0;
	g_wef  = 0;
	g_hf   = 0;
	g_c1f  = 0;
	g_c2f  = 0;

    g_cam1_detected = false;
    g_estop         = false;
    g_reset         = false;
}

/* ── 송신 ───────────────────────────────────── */
void Comm_SendState(uint8_t state_val)
{
    uint8_t d[1] = { state_val };
    send_frame(&huart2, PID_STATE, d, 1);
}

void Comm_SendDone(uint8_t done_val)
{
    uint8_t d[1] = { done_val };
    send_frame(&huart2, PID_DONE, d, 1);
}

void Comm_SendError(uint8_t err_val)
{
    uint8_t d[1] = { err_val };
    send_frame(&huart2, PID_ERR, d, 1);
}

/* 1바이트 플래그 → 라파1로 전송 */
void Comm_SendFlag(uint8_t pid, uint8_t val)
{
    uint8_t d[1] = { val };
    send_frame(&huart2, pid, d, 1);
}

/* 2바이트 값(UV, WCNT) → 라파1로 전송 */
void Comm_SendFlag16(uint8_t pid, uint16_t val)
{
    uint8_t d[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    send_frame(&huart2, pid, d, 2);
}

/* 테스트용 */
void Comm_SetStartFlag(void) { g_cam1_detected = true; g_ssf = 1; }
