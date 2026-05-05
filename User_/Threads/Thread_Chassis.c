#include "Thread_Chassis.h"

float Chassis_distance;
float* pChassis_distance = &Chassis_distance;

osMessageQueueId_t Chassis_Queue01Handle;     //定义消息队列的句柄 （全局）
const osMessageQueueAttr_t Chassis_Queue01_attributes = {
  .name = "Chassis_Queue01"
};

xSemaphoreHandle xMutex_test01;  //定义一个互斥量 （全局）

double protected_variable;  //定义一个受保护的变量，假设它是一个double类型的变量

void Chassis_Function(void *argument)
{
  /* USER CODE BEGIN ITS_Function */
  
  Chassis_Queue01Handle = osMessageQueueNew (16, sizeof(float), &Chassis_Queue01_attributes);//初始化队列
  xMutex_test01 = xSemaphoreCreateRecursiveMutex(); //初始化递归互斥量
  
  chassis_init();
  
  /* Infinite loop */
  for(;;)
  {
    //底盘伺服线程 移动指定距离，并且返回坐标
    chassis_ctrl(360);
    // chassis_readpos();
    osDelay(1);
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
