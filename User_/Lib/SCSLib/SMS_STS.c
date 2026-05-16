/*
 * SMS_STS.c
 * 飞特SMS/STS系列串行舵机应用层程序
 * 日期: 2025.3.3
 * 作者: 
 */

#include <string.h>
#include "INST.h"
#include "SCS.h"
#include "SMS_STS.h"
#include "SCServo.h"
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


FT_STS_t h_FT_STS[2];
uint8_t flag = 0;

int 


WritePosEx(FT_STS_t *FT_STS, int16_t Position, uint16_t Speed, uint8_t ACC) //增加舵机结构体
{
	uint8_t bBuf[7];
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);
	}

	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, Position);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, Speed);
	
	return genWrite(FT_STS->FT_ID, SMS_STS_ACC, bBuf, 7);
}

int RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
	uint8_t bBuf[7];
	if(Position<0){
		Position = -Position;
		Position |= (1<<15);
	}

	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, Position);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, Speed);
	
	return regWrite(ID, SMS_STS_ACC, bBuf, 7);
}

void SyncWritePosEx(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[])
{
	uint8_t offbuf[32*7];
	uint8_t i;
	uint16_t V;
  for(i = 0; i<IDN; i++){
		if(Position[i]<0){
			Position[i] = -Position[i];
			Position[i] |= (1<<15);
		}

		if(Speed){
			V = Speed[i];
		}else{
			V = 0;
		}
		if(ACC){
			offbuf[i*7] = ACC[i];
		}else{
			offbuf[i*7] = 0;
		}
		Host2SCS(offbuf+i*7+1, offbuf+i*7+2, Position[i]);
    Host2SCS(offbuf+i*7+3, offbuf+i*7+4, 0);
    Host2SCS(offbuf+i*7+5, offbuf+i*7+6, V);
	}
  syncWrite(ID, IDN, SMS_STS_ACC, offbuf, 7);
}

int WheelMode(uint8_t ID)
{
	return writeByte(ID, SMS_STS_MODE, 1);		
}

int WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC)
{
	uint8_t bBuf[2];
	if(Speed<0){
		Speed = -Speed;
		Speed |= (1<<15);
	}
	bBuf[0] = ACC;
	genWrite(ID, SMS_STS_ACC, bBuf, 1);
	
	Host2SCS(bBuf+0, bBuf+1, Speed);

	genWrite(ID, SMS_STS_GOAL_SPEED_L, bBuf, 2);
	return 1;
}

int CalibrationOfs(uint8_t ID)
{
	return writeByte(ID, SMS_STS_TORQUE_ENABLE, 128);
}

int unLockEpromEx(uint8_t ID)
{
	return writeByte(ID, SMS_STS_LOCK, 0);
}

int LockEpromEx(uint8_t ID)
{
	return writeByte(ID, SMS_STS_LOCK, 1);
}

void FT_Init()
{
	for(int i = 0; i<2 ; i++)
	{
		h_FT_STS[i].FT_ID = i+1;
		h_FT_STS[i].FT_Type = i;
		h_FT_STS[i].current = 0;
		h_FT_STS[i].read_flag = 0;
		h_FT_STS[i].load = 0;
		h_FT_STS[i].move = 0;
		h_FT_STS[i].position = 0;
		h_FT_STS[i].speed = 0;
		h_FT_STS[i].temper = 0;
		h_FT_STS[i].voltage = 0;
		h_FT_STS[i].overflow_number = 0;
		h_FT_STS[i].last_count = 0;
		h_FT_STS[i].total_count = 0;
		h_FT_STS[i].detect_count = 0;
		h_FT_STS[i].cnt = 0;
		h_FT_STS[i].Rotate_OK = 1;
		h_FT_STS[i].Step = 1;
		h_FT_STS[i].LastStep = 0;
		h_FT_STS[i].present_time_ms = 0;
		h_FT_STS[i].last_time_ms = 0;
		h_FT_STS[i].Flag = 1;
		WritePosEx(&h_FT_STS[i],2300,50,50);
	}
	setEnd(0);//SMS_STS舵机为大端存储结构

}

void FT_Update(FT_STS_t *STS_t)
{
	if(STS_t->read_flag == 1)
	{
		STS_t->cnt = ReadPos(STS_t->FT_ID);
		STS_t->total_count = STS_t->cnt + STS_t->overflow_number*STS_RELOADVALUE;
		if(STS_t->last_count - STS_t->total_count > 3000) // 根据编码器原理刷新圈数
		{
			  STS_t->overflow_number++;
			 STS_t->total_count = STS_t->cnt + STS_t->overflow_number*STS_RELOADVALUE;
		}
		else if(STS_t->total_count - STS_t->last_count > 3000) // 
		{
			STS_t->overflow_number--;
			STS_t->total_count = STS_t->cnt + STS_t->overflow_number*STS_RELOADVALUE;
		}
		STS_t->position = STS_t->total_count;
		STS_t->detect_count = STS_t->last_count;
		STS_t->last_count = STS_t->total_count;	
	}
	STS_t->read_flag = 0;
}

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim)//定时器中断回调函数
// {
//   if(htim->Instance == TIM2)   //判断是哪个定时器触发的中�??
//   {
//     flag = 1;
//   }
// }