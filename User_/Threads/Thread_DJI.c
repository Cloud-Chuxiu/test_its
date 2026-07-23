#include "Thread_DJI.h"

volatile float Chassis_distance = 0;
volatile float Beam_distance    = 0;
volatile float Updown_distance  = 0;
float *pChassis_distance = (float *)&Chassis_distance;
float *pBeam_distance    = (float *)&Beam_distance;
float *pUpdown_distance  = (float *)&Updown_distance;


void DJI_Function(void *argument)
{
  
  osDelay(500);
  for (;;) {
    // ----- 位置伺服计算（不发送CAN）-----
    if(Chassis_distance != 0)
    {
      positionServo_chassis(Chassis_distance, &hDJI[0]);
    }
    if(Beam_distance != 0)
    {
      positionServo_Beam(Beam_distance, &hDJI[2]);
    }
    //positionServo(Beam_distance, &hDJI[2]);
    positionServo(Updown_distance, &hDJI[3]);

    // ----- 统一CAN发送（避免竞态）-----
    CanTransmit_DJI_1234(&hcan1,
        hDJI[0].speedPID.output,   // m0: 底盘
        -hDJI[0].speedPID.output,  // m1: 底盘反向跟随
        hDJI[2].speedPID.output,   // m2: 横梁
        hDJI[3].speedPID.output);  // m3

    // CanTransmit_DJI_5678(&hcan2,
    //     0,   // m4
    //     0,   // m5: 升降
    //     0,   // m6
    //     0);  // m7

    osDelay(1);
  }
}

void DJI_Start()
{
  osThreadId_t DJI_Handle;
  const osThreadAttr_t DJI_attributes = {
    .name       = "DJI",
    .stack_size = 128 * 10,
    .priority   = (osPriority_t)osPriorityHigh,
  };
  osThreadNew(DJI_Function, NULL, &DJI_attributes);
}
