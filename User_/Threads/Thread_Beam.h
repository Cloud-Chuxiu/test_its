#ifndef _THREAD_BEAM_H__
#define _THREAD_BEAM_H__

#endif

extern float Beam_distance;
extern float *pBeam_distance;

#include "Head.h"
#include "beam.h"

void Beam_Start();      //定义任务开启函数，在默认线程的实现中调用它来创建这个线程
