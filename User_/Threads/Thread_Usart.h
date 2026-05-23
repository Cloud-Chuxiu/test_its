#ifndef _THREAD_USART_H__
#define _THREAD_USART_H__

#include "Head.h"
#include "STP_Decode.h"

extern uint8_t Rxbuffer_1[];
extern uint8_t usart1_rx[];
extern uint16_t UartFlag[];
extern LidarPointTypedef Lidar1;

void Usart_Start();
void Usart_Function(void *argument);
void Usart_start();

#endif
