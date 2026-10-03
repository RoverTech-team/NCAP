// Route newlib stdio (printf) to the ST-Link UART so micro-ROS/uxr logs are visible.

#include "main.h"

extern UART_HandleTypeDef huart2;

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
    return ch;
}

int __io_getchar(void)
{
    uint8_t c = 0;
    HAL_UART_Receive(&huart2, &c, 1, 1);
    return c;
}
