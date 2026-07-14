#include "Thread_Usart.h"


void Usart_Function(void *argument)
{
  /* USER CODE BEGIN Usart_Function */
  osDelay(2000);
  printf("u6ready\r\n");
  uint32_t tick = 0;
  HAL_UART_Receive_IT(&huart1, usart1_rx, 1);
  /* Infinite loop */
  for (;;) {
    if (UartFlag[0]) {
      STP_23L_Decode(Rxbuffer_1, &Lidar1);
      UartFlag[0] = 0;
      hDJI[0].AxisData.lidar_distance = Lidar1.distance_aver;
      //printf("%.2f\n",hDJI[0].AxisData.lidar_distance);
      //printf("%.2f\n",hDJI[2].AxisData.AxisAngle_inDegree);
      //printf("%.2f\n",hDJI[5].AxisData.AxisAngle_inDegree);
      //printf("%f\n",hDJI[5].speedPID.output);
    }
      osDelay(1);
  }
  /* USER CODE END Usart_Function */
}

void Usart_Start()
{
  osThreadId_t Usart_Handle;
  const osThreadAttr_t Usart_attributes = {
    .name       = "Usart",
    .stack_size = 128 * 10,
    .priority   = (osPriority_t)osPriorityAboveNormal,
  };
  osThreadNew(Usart_Function, NULL, &Usart_attributes);
}

void Usart_start()
{
  HAL_UART_Receive_IT(&huart1, usart1_rx, 1);
}


