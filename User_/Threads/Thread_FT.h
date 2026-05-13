#ifndef _THREAD_FT_H__
#define _THREAD_FT_H__

#include "User_SMS_STS.h"
#include "Head.h"

extern int16_t FT_phy;
extern int16_t* pFT_phy;

void FT_Start();      //定义任务开启函数，在默认线程的实现中调用它来创建这个线程
void FT_Function(void *argument);
extern FT_STS_t *Claw_0;


#endif