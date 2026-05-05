#ifndef _STATE_H__
#define _STATE_H__

#include "DJI.h"

typedef struct{

    float state_x; //
    float state_y;
    float state_z;
    float state_phy;
    float state_theta; 

}coordinate;

extern coordinate crane_xyz;
extern coordinate target_xyz;

#endif
