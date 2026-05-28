#include "Thread_DJI.h"

float Chassis_distance = 0;
float Beam_distance    = 0;
float Updown_distance  = 0;
float *pChassis_distance = &Chassis_distance;
float *pBeam_distance    = &Beam_distance;
float *pUpdown_distance  = &Updown_distance;


void DJI_Function(void *argument)
{
  osDelay(1000);

  for (;;) {
    // ----- 位置伺服计算（不发送CAN）-----
    positionServo(Chassis_distance, &hDJI[0]);
    positionServo(Beam_distance, &hDJI[2]);
    positionServo(Updown_distance, &hDJI[5]);

    // ----- 统一CAN发送（避免竞态）-----
    CanTransmit_DJI_1234(&hcan1,
        -hDJI[0].speedPID.output,   // m0: 底盘
        hDJI[0].speedPID.output,  // m1: 底盘反向跟随
        hDJI[2].speedPID.output,   // m2: 横梁
        hDJI[3].speedPID.output);  // m3

    CanTransmit_DJI_5678(&hcan2,
        hDJI[4].speedPID.output,   // m4
        hDJI[5].speedPID.output,   // m5: 升降
        hDJI[6].speedPID.output,   // m6
        hDJI[7].speedPID.output);  // m7

    osDelay(1);
  }
}

void DJI_Start()
{
  osThreadId_t DJI_Handle;
  const osThreadAttr_t DJI_attributes = {
    .name       = "DJI",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityLow,
  };
  osThreadNew(DJI_Function, NULL, &DJI_attributes);
}
