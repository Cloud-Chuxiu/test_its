#ifndef _CACULATE_H__
#define _CACULATE_H__

#include "DJI.h"

void positionServo(float ref, DJI_t * motor);

void positionServo_chassis(float ref, DJI_t *motor, float lidar_distance);

void speedServo(float ref, DJI_t * motor);

#endif