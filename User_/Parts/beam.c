#include "beam.h"


//横梁初始化
void beam_Init()
{
    hDJI[2].motorType = M3508;
    DJI_Init();
}


//
void beam_move(float distance)
{
    positionServo(distance,&hDJI[2]);
    CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,
    hDJI[1].speedPID.output,
    hDJI[2].speedPID.output,
    hDJI[3].speedPID.output);
}


//控制横梁机构移动到指定坐标对应的位置

void beam_ctrl(float pos) //坐标
{
   while(1)
    {
        if(hDJI[2].flag == 1)
        {
            beam_move(pos);
            //beam_posget();
            hDJI[2].flag = 0;
        }
        if(hDJI[2].speedPID.output == 0)
        {
            return;
        }
    }
}

void beam_posget()
{
    printf("%f\n",hDJI[2].AxisData.AxisAngle_inDegree);
    crane_xyz.state_y = hDJI[2].AxisData.AxisVelocity;
    printf("%f\n",crane_xyz.state_y);
}


