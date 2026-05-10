#include "main.h"
#include "SCServo.h"
#include "math.h"
#include "Thread_Start.h"
#include "cmsis_os2.h"

void FT_Write_Position(FT_STS_t *STS_t, int16_t position, uint16_t speed, uint8_t ACC)
{
  if((fabs(STS_t->position-position)>8)&&(STS_t->Rotate_OK == 1))//只有当舵机既没有转到指定位置，并且允许旋转时，舵机才可以转动
  {
    STS_t->Rotate_OK = 0; //锁住舵机状态，不让后续指令介入，建立在一条指令就可以控制舵机的原则上。
    WritePosEx(STS_t, position, speed, ACC);
  }
  if(fabs(STS_t->position-position)<=8)
  {
    osDelay(500);
    STS_t->Rotate_OK = 1;// 表示可以后续操作。
  }
  // 舵机(ID1)以最高速度V=60*0.732=43.92rpm，加速度A=50*8.7deg/s^2，运行至P1=4095位置
   
  // HAL_Delay((4095-0)*1000/(60*50) + (60*50)*10/(50) + 50);//[(P1-P0)/(V*50)]*1000+[(V*50)/(A*100)]*1000 + 50(误差)
}
