#ifndef _THREAD_USART_H__
#define _THREAD_USART_H__

#include "Head.h"
#include "STP_Decode.h"

extern uint8_t Rxbuffer_1[];
extern uint8_t usart1_rx[];
extern uint16_t UartFlag[];
extern LidarPointTypedef Lidar1;
extern LidarPointTypedef Lidar2;
extern uint8_t Rxbuffer_3[];
extern uint8_t usart3_rx[];
extern uint8_t usart6_rx[];
extern volatile uint32_t usart1_rx_bytes;
extern volatile uint32_t usart1_frame_cnt;
extern volatile uint32_t usart3_frame_cnt;

extern volatile uint8_t pi_digit_ready;
extern volatile uint8_t pi_bean_ready;
extern char pi_digit_str[16];
extern char pi_bean_code;

void Usart_Start();
void Usart_Function(void *argument);
void Usart_start();
void Pi_SendString(const char *str);

#endif
