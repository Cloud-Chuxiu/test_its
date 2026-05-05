#ifndef _CHASSIS_H__
#define _CHASSIS_H__

#endif

#include "DJI.h"
#include "Caculate.h"
#include "wtr_can.h"
#include "State.h"
#include "usart.h"
#include "stdio.h"

void chassis_init();
void chassis_readspeed();
void chassis_move(float distance);
void chassis_readpos();
void chassis_ctrl(float distance);


