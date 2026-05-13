#ifndef _THREAD_PTZ_H__
#define _THREAD_PTZ_H__

#include "Head.h"
#include "PTZ.h"


void PTZ_Function(void *argument);
void PTZ_Start();      //定义任务开启函数，在默认线程的实现中调用它来创建这个线程


#endif