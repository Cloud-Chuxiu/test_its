#include "State.h"
#include "State.h"
coordinate crane_xyz;
coordinate target_xyz;


//坐标系初始化
void crane_Init()
{
    crane_xyz.state_x = 0;
    crane_xyz.state_y = 0;
    crane_xyz.state_z = 0;
    crane_xyz.state_phy = 0;
    crane_xyz.state_theta = 0;
    target_xyz.state_x = 0;
    target_xyz.state_y = 0;
    target_xyz.state_z = 0;
    target_xyz.state_phy = 0;
    target_xyz.state_theta = 0;
}

void read_pos(uint8_t part_)
{
    float value = 0;
    char str_buffer[10] = {0};      
    switch(part_)
    {
        case 1:{
            value = target_xyz.state_x;
            sprintf(str_buffer, "%.2f\r\n", value);
            HAL_UART_Transmit_IT(&huart6,(uint8_t *)str_buffer,10);
        }
        case 2:{
            value = target_xyz.state_y;
            sprintf(str_buffer, "%.2f\r\n", value);
            HAL_UART_Transmit_IT(&huart6,(uint8_t *)str_buffer,10);

        }
        case 3:{
            value = target_xyz.state_z;
            sprintf(str_buffer, "%.2f\r\n", value);
            HAL_UART_Transmit_IT(&huart6,(uint8_t *)str_buffer,10);
        }
        case 4:{
            value = target_xyz.state_theta;
            sprintf(str_buffer, "%.2f\r\n", value);
            HAL_UART_Transmit_IT(&huart6,(uint8_t *)str_buffer,10);
        }
        default: return;
    }
}







