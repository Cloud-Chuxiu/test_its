#include "Thread_Beam.h"

float Beam_distance;
float* pBeam_distance = &Beam_distance;

osMessageQueueId_t Beam_Queue01Handle;     //定义消息队列的句柄 （全局）
const osMessageQueueAttr_t Beam_Queue01_attributes = {
  .name = "Beam_Queue01"
};

xSemaphoreHandle xMutex_test02;  //定义一个互斥量 （全局）

double protected_variable01;  //定义一个受保护的变量，假设它是一个double类型的变量

void Beam_Function(void *argument)
{
  /* USER CODE BEGIN ITS_Function */
  
  Beam_Queue01Handle = osMessageQueueNew (16, sizeof(float), &Beam_Queue01_attributes);//初始化队列
  xMutex_test02 = xSemaphoreCreateRecursiveMutex(); //初始化递归互斥量  
  beam_Init();
  /* Infinite loop */
  for(;;)
  {
    beam_ctrl(Beam_distance);
    osDelay(1);
  }
  /* USER CODE END ITS_Function */
}

void Beam_Start()      //定义任务开启函数，在默认线程的实现中调用它来创建这个线程
{
osThreadId_t Beam_Handle;
const osThreadAttr_t Beam_attributes = {       //封装好了创建线程需要的属性
  .name = "Beam",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
  osThreadNew(Beam_Function, NULL, &Beam_attributes);       //调用了CMSIS-RTOS2的线程创建函数来创建线程
}
