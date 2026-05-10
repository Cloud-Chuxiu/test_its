#include "Thread_FT.h"

FT_STS_t *Claw_0;


void FT_Function(void *argument)
{
  /* USER CODE BEGIN ITS_Function */
  FT_Write_Speed_Setup();
  FT_Write_Position(Claw_0,1024,50,50);
  /* Infinite loop */
  for(;;)
  {
    
    osDelay(1);
  }
  /* USER CODE END ITS_Function */
}

void FT_Start()      //定义任务开启函数，在默认线程的实现中调用它来创建这个线程
{
osThreadId_t FT_Handle;
const osThreadAttr_t FT_attributes = {       //封装好了创建线程需要的属性
  .name = "FT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
  osThreadNew(FT_Function, NULL, &FT_attributes);       //调用了CMSIS-RTOS2的线程创建函数来创建线程
}
