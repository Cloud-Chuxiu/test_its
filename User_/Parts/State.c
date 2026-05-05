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

//







