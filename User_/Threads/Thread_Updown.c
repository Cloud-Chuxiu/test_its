#include "Thread_Updown.h"

float Updown_distance = 0;
float* pUpdown_distance = &Updown_distance;





void Updown_Function(void *argument)
{
  /* USER CODE BEGIN ITS_Function */
  
  
  //Updown_Init();
  /* Infinite loop */
  for(;;)
  {
    updown_move(480);
    osDelay(1);
  }
  /* USER CODE END ITS_Function */
}

void Updown_Start()      //定义任务开启函数，在默认线程的实现中调用它来创建这个线程
{
osThreadId_t Updown_Handle;
const osThreadAttr_t Updown_attributes = {       //封装好了创建线程需要的属性
  .name = "Updown",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
  osThreadNew(Updown_Function, NULL, &Updown_attributes);       //调用了CMSIS-RTOS2的线程创建函数来创建线程
}
