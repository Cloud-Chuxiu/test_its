#include "updown.h"



//齿条初始化
void updown_Init()
{
    hDJI[5].motorType = M3508;
    DJI_Init();
}


//升降
void updown_move(float distance)
{
    positionServo(distance,&hDJI[5]);
    CanTransmit_DJI_5678(&hcan2,hDJI[4].speedPID.output,
    hDJI[5].speedPID.output,
    hDJI[6].speedPID.output,
    hDJI[7].speedPID.output);
}


//控制升降机构移动到相应高度

void updown_ctrl(float pos) //坐标
{
    hDJI[5].flag = 1;
    while(1)
    {
        if(hDJI[5].flag == 1)
        {
            updown_move(pos);
           
            hDJI[5].flag = 0;
        }
        if(hDJI[5].speedPID.output == 0)
        {
            
            printf("return\n");
            return;
        }
    
    }
}

void updown_posget()
{
    crane_xyz.state_z = hDJI[5].AxisData.AxisVelocity;
    printf("%.2f\n",crane_xyz.state_z);
}


