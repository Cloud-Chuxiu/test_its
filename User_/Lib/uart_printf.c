/**
 * @file uart_printf.c
 * @author TITH (1023515576@qq.com)
 * @brief printf()串口重定向
 * @version 1.0
 * @date 2022-01-12
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __UART_PRINTF_C__
#define __UART_PRINTF_C__
#include "usart.h"
// #include"Chassis_Start.h"

#include <stdio.h>

// 串口号配置
#define UART_Printf_Config_huart huart6

/* 重定向printf */

// 对于 GCC:

__attribute__((weak)) int _write(int file, uint8_t *ptr, int len)
{
    if (HAL_UART_Transmit(&UART_Printf_Config_huart, ptr, len, 100) != HAL_OK) {
        Error_Handler();
    }
    return len;
}

// 对于 MDK:

// int fputc(int ch, FILE *stream)
// {
//     while (HAL_UART_Transmit(&UART_Printf_Config_huart, (uint8_t *)&ch, 1, 0xffff) == HAL_BUSY)
//         ;
//     return ch;
// }

// 非阻塞浮点数打印（双缓冲，避免发送冲突）
void float_print(UART_HandleTypeDef *huart, float value)
{
    static char buf[2][20];
    static uint8_t idx = 0;
    idx = (idx + 1) % 2;
    int len = snprintf(buf[idx], 20, "%.1f\r\n", value);
    HAL_UART_Transmit_IT(huart, (uint8_t *)buf[idx], len);
}

#endif