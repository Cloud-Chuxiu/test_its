#include "camera.h"

void camera_read(uint8_t* camera_rx)
{
    
}


void camera_start(uint8_t came)
{
    HAL_UART_Transmit_IT(&huart3,"camera start\n",13);
    HAL_UART_Transmit_IT(&huart3,&came,1);
}
