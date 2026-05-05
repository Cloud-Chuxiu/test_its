#include "Thread_Master.h"

float Master_distance;
float* pMaster_distance = &Master_distance;

osMessageQueueId_t Master_Queue01Handle;     //定义消息队列的句柄 （全局）
const osMessageQueueAttr_t Master_Queue01_attributes = {
  .name = "Master_Queue01"
};

xSemaphoreHandle xMutex_test03;  //定义一个互斥量 （全局）

double protected_variable02;  //定义一个受保护的变量，假设它是一个double类型的变量

void Master_Function(void *argument)
{
  /* USER CODE BEGIN ITS_Function */
  
  Master_Queue01Handle = osMessageQueueNew (16, sizeof(float), &Master_Queue01_attributes);//初始化队列
  xMutex_test03 = xSemaphoreCreateRecursiveMutex(); //初始化递归互斥量
  /* Infinite loop */
  for(;;)
  {
    
    osDelay(1);
  }
  /* USER CODE END ITS_Function */
}

void Master_Start()      //定义任务开启函数，在默认线程的实现中调用它来创建这个线程
{
osThreadId_t Master_Handle;
const osThreadAttr_t Master_attributes = {       //封装好了创建线程需要的属性
  .name = "Master",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
  osThreadNew(Master_Function, NULL, &Master_attributes);       //调用了CMSIS-RTOS2的线程创建函数来创建线程
}
