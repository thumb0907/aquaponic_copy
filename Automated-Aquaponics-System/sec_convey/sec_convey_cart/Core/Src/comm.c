// comm.c — STM2 통신 레이어
// 라파2(USART2)와만 통신. 스카라/매니퓰레이터는 라파2가 중계함.

#include "comm.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart2;  // 라즈베리파이2 (유일한 통신 대상)

/* ── 수신 상태머신 ──────────────────────────── */
typedef enum {
    RX_WAIT_SOF = 0,
    RX_WAIT_ID,
    RX_WAIT_LEN,
    RX_WAIT_DATA,
    RX_WAIT_CHK,
} RxState;

static uint8_t g_rx_byte              = 0;
static RxState g_rx_state             = RX_WAIT_SOF;
static uint8_t g_rx_id                = 0;
static uint8_t g_rx_len               = 0;
static uint8_t g_rx_idx               = 0;
static uint8_t g_rx_data[COMM_MAX_DATA_LEN];

/* ── 플래그 ─────────────────────────────────── */
static volatile uint8_t g_ff    = 0;      // FF 값 (0 또는 1)
static volatile bool    g_hf    = false;  // 수확 완료
static volatile bool    g_estop = false;  // 긴급정지
static volatile bool    g_reset = false;  // 리셋

/* ── 체크섬 계산 ──────────────────────────── */
static uint8_t calc_chk(uint8_t id, uint8_t len, const uint8_t *data)
{
    uint16_t s = id + len;
    for (uint8_t i = 0; i < len; i++) s += data[i];
    return (uint8_t)(s & 0xFF);
}

/* ── 프레임 송신 ──────────────────────────── */
static void send_frame(uint8_t id, uint8_t len, const uint8_t *data)
{
    uint8_t buf[4 + COMM_MAX_DATA_LEN];
    buf[0] = COMM_SOF;
    buf[1] = id;
    buf[2] = len;
    memcpy(&buf[3], data, len);
    buf[3 + len] = calc_chk(id, len, data);
    HAL_UART_Transmit(&huart2, buf, 4 + len, 100);
}

/* ── 수신 프레임 처리 ─────────────────────── */
static void process_frame(uint8_t id, uint8_t len, uint8_t *data)
{
    if (len == 0) return;

    switch (id) {
        case PID_FF:
            g_ff = data[0];          // 0 또는 1 그대로 저장
            break;
        case PID_HF:
            if (data[0] == 1) g_hf = true;
            break;
        case PID_ESTOP:
            g_estop = true;
            break;
        case PID_RESET:
            g_reset = true;          // Do_Reset()에서 클리어
            break;
        default:
            break;
    }
}

/* ── 초기화 ───────────────────────────────── */
void Comm_Init(void)
{
    HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
}

/* ── USART2 수신 콜백 (main.c에서 호출) ───── */
void Comm_Rasp_RxCallback(void)
{
    uint8_t b = g_rx_byte;

    switch (g_rx_state) {
        case RX_WAIT_SOF:
            if (b == COMM_SOF) g_rx_state = RX_WAIT_ID;
            break;

        case RX_WAIT_ID:
            g_rx_id    = b;
            g_rx_state = RX_WAIT_LEN;
            break;

        case RX_WAIT_LEN:
            g_rx_len   = b;
            g_rx_idx   = 0;
            g_rx_state = (g_rx_len == 0) ? RX_WAIT_CHK : RX_WAIT_DATA;
            break;

        case RX_WAIT_DATA:
            if (g_rx_idx < COMM_MAX_DATA_LEN)
                g_rx_data[g_rx_idx++] = b;
            if (g_rx_idx >= g_rx_len) g_rx_state = RX_WAIT_CHK;
            break;

        case RX_WAIT_CHK: {
            uint8_t expected = calc_chk(g_rx_id, g_rx_len, g_rx_data);
            if (b == expected)
                process_frame(g_rx_id, g_rx_len, g_rx_data);
            // 체크섬 불일치 → 프레임 버림
            g_rx_state = RX_WAIT_SOF;
            break;
        }

        default:
            g_rx_state = RX_WAIT_SOF;
            break;
    }

    HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
}

/* ── 플래그 getter / setter ──────────────────── */
uint8_t Comm_GetFf(void)          { return g_ff; }
void    Comm_SetFf(uint8_t v)     { g_ff = v; }

bool    Comm_IsHfSet(void)        { return g_hf; }
void    Comm_ClearHfFlag(void)    { g_hf = false; }

bool    Comm_IsEstop(void)        { return g_estop; }
void    Comm_ClearEstop(void)     { g_estop = false; }

bool    Comm_IsReset(void)        { return g_reset; }
void    Comm_ClearReset(void)     { g_reset = false; }

void    Comm_ClearAllFlags(void)  {
    g_ff = 0; g_hf = false;
    g_estop = false; g_reset = false;
}

/* ── 송신 함수 ────────────────────────────── */
void Comm_SendState(uint8_t state)
{
    send_frame(PID_STATE, 1, &state);
}

void Comm_SendDone(uint8_t done_code)
{
    send_frame(PID_DONE, 1, &done_code);
}

void Comm_SendError(uint8_t err_code)
{
    send_frame(PID_ERR, 1, &err_code);
}
