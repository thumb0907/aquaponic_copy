// comm.c
#include "comm.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart1;  // 아두이노
extern UART_HandleTypeDef huart2;  // 라즈베리파이

// 수신 버퍼
#define RX_BUF_SIZE 8

// 라즈베리파이
static uint8_t rasp_rx_byte = 0;
static uint8_t rasp_buf[RX_BUF_SIZE] = {0};
static uint8_t rasp_buf_idx = 0;
static volatile bool start_flag = false;

// 아두이노
static uint8_t ardu_rx_byte = 0;
static uint8_t ardu_buf[RX_BUF_SIZE] = {0};
static uint8_t ardu_buf_idx = 0;
static volatile bool reset_flag = false;

// 파싱 (\n 종단)
static void Parse_Rasp(uint8_t *buf) // cam에서 트레이를 인식하면 S1을 stm32한테 보냄
{
    // 형식: "S1\n", "S0\n" ...
    if (buf[0] < 'A' || buf[0] > 'Z') return;
    if (buf[1] < '0' || buf[1] > '9') return;

    uint8_t cmd = buf[0];
    uint8_t val = buf[1] - '0';

    if (cmd == 'S' && val == 1) start_flag = true;
    // 확장: else if (cmd == 'E') emergency = true; 등
}

static void Parse_Arduino(uint8_t *buf) //
{
    // 형식: "R1\n" ...
    if (buf[0] < 'A' || buf[0] > 'Z') return;
    if (buf[1] < '0' || buf[1] > '9') return;

    uint8_t cmd = buf[0];
    uint8_t val = buf[1] - '0';

    if (cmd == 'R' && val == 1) reset_flag = true;
}

// 초기화
void Comm_Init(void)
{
    HAL_UART_Receive_IT(&huart2, &rasp_rx_byte, 1);  // 라즈베리파이
    HAL_UART_Receive_IT(&huart1, &ardu_rx_byte, 1);  // 아두이노
}

// 콜백
void Comm_Rasp_RxCallback(void)
{
    if (rasp_rx_byte == '\n') {
        rasp_buf[rasp_buf_idx] = '\0';
        Parse_Rasp(rasp_buf);
        rasp_buf_idx = 0;
    } else if (rasp_buf_idx < RX_BUF_SIZE - 1) {
        rasp_buf[rasp_buf_idx++] = rasp_rx_byte;
    } else {
        rasp_buf_idx = 0;  // 버퍼 오버플로 → 리셋
    }
    HAL_UART_Receive_IT(&huart2, &rasp_rx_byte, 1);
}

void Comm_Arduino_RxCallback(void)
{
    if (ardu_rx_byte == '\n') {
        ardu_buf[ardu_buf_idx] = '\0';
        Parse_Arduino(ardu_buf);
        ardu_buf_idx = 0;
    } else if (ardu_buf_idx < RX_BUF_SIZE - 1) {
        ardu_buf[ardu_buf_idx++] = ardu_rx_byte;
    } else {
        ardu_buf_idx = 0;
    }
    HAL_UART_Receive_IT(&huart1, &ardu_rx_byte, 1);
}

// 플래그
bool Comm_IsStartFlagSet(void) { return start_flag; }
void Comm_ClearStartFlag(void) { start_flag = false; }

bool Comm_IsResetFlagSet(void) { return reset_flag; }
void Comm_ClearResetFlag(void) { reset_flag = false; }

// 송신
void Comm_SendDone(void)   // 직교로봇 파종 끝나면 라즈베리파이로 전송
{
    uint8_t msg[] = "D1\n";
    HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 100);
}

void Comm_SendScara(void)  // 직교로봇 파종 끝나면 아두이노로 전송
{
    uint8_t msg[] = "D1\n";
    HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, 100);
}

// 테스트할때 사용
void Comm_SetStartFlag(void) { start_flag = true; }
