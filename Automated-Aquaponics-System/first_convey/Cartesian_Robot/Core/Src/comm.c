#include "comm.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

static uint8_t rx_byte = 0;
static volatile bool start_flag = false;

void Comm_Init(void)
{
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

void Comm_RxCallback(void)
{
    if (rx_byte == 'S')
    {
        start_flag = true;
    }
    // 여기에 나중에 다른 커맨드 추가 가능
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1); // 재등록
}

bool Comm_IsStartFlagSet(void)
{
    return start_flag;
}

void Comm_ClearStartFlag(void)
{
    start_flag = false;
}

void Comm_SendDone(void)
{
    uint8_t msg[] = "DONE\r\n";
    HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 100);
}
