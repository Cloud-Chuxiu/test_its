#include "Thread_FT.h"

int16_t FT_phy = 0;
int16_t* pFT_phy = &FT_phy;


void FT_Function(void *argument)
{
  /* USER CODE BEGIN ITS_Function */
  FT_Init();
  FT_phy = FT_INIT;


  /* Infinite loop */
  for(;;)
  {
    WritePosEx(&h_FT_STS[0],FT_phy,20,50);
    osDelay(50);
  }
  /* USER CODE END ITS_Function */
}

void FT_Start()      //定义任务开启函数，在默认线程的实现中调用它来创建这个线程
{
osThreadId_t FT_Handle;
const osThreadAttr_t FT_attributes = {       //封装好了创建线程需要的属性
  .name = "FT",
  .stack_size = 128 * 10,
  .priority = (osPriority_t)osPriorityAboveNormal,
};
  osThreadNew(FT_Function, NULL, &FT_attributes);       //调用了CMSIS-RTOS2的线程创建函数来创建线程
}
