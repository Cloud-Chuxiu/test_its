#include "Thread_Chassis.h"

float Chassis_distance = 0;
float* pChassis_distance = &Chassis_distance;



void Chassis_Function(void *argument)
{
  /* USER CODE BEGIN ITS_Function */
  
  
  osDelay(1000);
  /* Infinite loop */
  for(;;)
  {
    //底盘伺服线程 移动指定距离，并且返回坐标
    chassis_move(Chassis_distance);
    osDelay(1);
    //printf("finished\n");
    // chassis_readpos();
    //osDelay(2000);
    //DJI_Init();
    ///chassis_ctrl(360);
  }
  /* USER CODE END ITS_Function */
}

void Chassis_Start()      //定义任务开启函数，在默认线程的实现中调用它来创建这个线程
{

osThreadId_t Chassis_Handle;
const osThreadAttr_t Chassis_attributes = {       //封装好了创建线程需要的属性
  .name = "Chassis",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadNew(Chassis_Function, NULL, &Chassis_attributes);       //调用了CMSIS-RTOS2的线程创建函数来创建线程

}
