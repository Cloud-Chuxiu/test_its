/*
 * @Author: ZYT
 * @Date: 2024-05-20 16:08:20
 * @LastEditors: ZYT
 * @LastEditTime: 2024-05-26 23:23:52
 * @FilePath: \Gantry_Trial\UserCode\Lib\STP_23L_Decode\STP_Decode.c
 * @Brief: 
 * 
 * Copyright (c) 2024 by zyt, All Rights Reserved. 
 */


#include "STP_Decode.h"
#include <stdio.h>

void STP_23L_Decode(uint8_t *buffer, LidarPointTypedef*lidardata) // num:指明是第几个雷达，本代码框架中范围为0-3
{
    if((buffer[0]==buffer[1])&&(buffer[1]==buffer[2])&&(buffer[2]==buffer[3])&&(buffer[3]==0xAA))     //检测帧头
    {
        if (buffer[5] == PACK_GET_DISTANCE)                                                                       //检测命令码
         {   uint32_t CS_sum = 0;
            for (uint16_t i = 4; i < 194; i++) CS_sum += buffer[i];
            if (buffer[194] == CS_sum%256)                                                                        //检测校验码
            {
                float dis_sum = 0;
                for (uint16_t i = 0; i < 12; i++) {
                    lidardata->distance = buffer[10 + 15 * i] + (buffer[11 + 15 * i] << 8);
                    dis_sum += lidardata->distance;
                }
                lidardata->distance_aver = dis_sum / 12;
                //printf("avg:%.1f mm\r\n", lidardata->distance_aver);
            }
         }
    }
}