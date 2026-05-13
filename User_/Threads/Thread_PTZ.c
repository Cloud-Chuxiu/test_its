#include "Thread_PTZ.h"



float PTZ_distance;
float* pPTZ_distance = &PTZ_distance;



void PTZ_Function(void *argument)
{
  /* USER CODE BEGIN ITS_Function */
  
  osDelay(3000);
  /* Infinite loop */
  for(;;)
  {
   // PTZ_move(720);
    osDelay(1);
  }
  /* USER CODE END ITS_Function */
}

void PTZ_Start()      //定义任务开启函数，在默认线程的实现中调用它来创建这个线程
{
osThreadId_t PTZ_Handle;
const osThreadAttr_t PTZ_attributes = {       //封装好了创建线程需要的属性
  .name = "PTZ",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
  osThreadNew(PTZ_Function, NULL, &PTZ_attributes);       //调用了CMSIS-RTOS2的线程创建函数来创建线程
}
