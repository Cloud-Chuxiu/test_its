#ifndef _THREAD_DJI_H__
#define _THREAD_DJI_H__

#include "Head.h"
#include "chassis.h"
#include "beam.h"
#include "updown.h"

extern volatile float Chassis_distance;
extern volatile float Beam_distance;
extern volatile float Updown_distance;
extern float *pChassis_distance;
extern float *pBeam_distance;
extern float *pUpdown_distance;

void DJI_Start();

#endif
