#include "updown.h"



//齿条初始化
void updown_Init()
{
    hDJI[3].motorType = M3508;
    DJI_Init();
}


//升降
void updown_move(float distance)
{
    positionServo(distance,&hDJI[3]);
    CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,
    hDJI[1].speedPID.output,
    hDJI[2].speedPID.output,
    hDJI[3].speedPID.output);
}


//控制升降机构移动到相应高度

void updown_ctrl(float pos) //坐标
{
   while(1)
    {
        if(hDJI[3].flag == 1)
        {
            updown_move(pos);
            //updown_posget();
            hDJI[3].flag = 0;
        }
        if(hDJI[3].speedPID.output == 0)
        {
            return;
        }
    }
}

void updown_posget()
{
    crane_xyz.state_z = hDJI[3].AxisData.AxisVelocity;
    printf("%f\n",crane_xyz.state_z);
}


