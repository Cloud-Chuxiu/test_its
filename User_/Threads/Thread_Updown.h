#ifndef _THREAD_UPDOWN_H__
#define _THREAD_UPDOWN_H__

#include "updown.h"
#include "Head.h"


void Updown_Start();      //定义任务开启函数，在默认线程的实现中调用它来创建这个线程
void Updown_Function(void *argument);


#endif