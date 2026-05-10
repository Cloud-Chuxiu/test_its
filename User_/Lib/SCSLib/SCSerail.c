/*
 * SCServo.c
 * 飞特舵机硬件接口层程序
 * 日期: 2024.12.2
 * 作者: ChenXuanyu
 */
#include <stdint.h>
#include "SCServo.h"
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"

uint8_t wBuf[128];
uint8_t wLen = 0;



//UART 接收数据接口
int readSCS(unsigned char *nDat, int nLen)
{
	return ftUart_Read(nDat, nLen);
}

//UART 发送数据接口
int writeSCS(unsigned char *nDat, int nLen)
{
	while(nLen--){
		if(wLen<sizeof(wBuf)){
			wBuf[wLen] = *nDat;
			wLen++;
			nDat++;
		}
	}
	return wLen;
}

//接收缓冲区刷新
void rFlushSCS()
{
	ftBus_Delay();
}

//发送缓冲区刷新
void wFlushSCS()
{
	if(wLen){
		ftUart_Send(wBuf, wLen);
		wLen = 0;
	}
}
//串口发送函数
void ftUart_Send(uint8_t *nDat , int nLen)
{
	HAL_UART_Transmit(&huart2, nDat, nLen, 100);
}
//串口读取函数
int ftUart_Read(uint8_t *nDat, int nLen)
{
	if(HAL_OK!=HAL_UART_Receive(&huart2, nDat, nLen, 100)){
		return 0;
	}else{
		return nLen;
	}
}

//自定义延时函数，大于10us
void ftBus_Delay(void)
{
	osDelay(1);
}