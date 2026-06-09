#ifndef _ACTION_H__
#define _ACTION_H__

#include <stdint.h>

#define SM_ROUNDS 3

/* 单次赛程所有参数 */
typedef struct {
    float   pick_x[SM_ROUNDS];     // 各轮取货区底盘位置 (mm)
    float   drop_x[SM_ROUNDS];     // 各轮卸货区底盘位置 (mm)
    float   beam_pick[SM_ROUNDS];  // 各轮取货横梁角度
    float   beam_drop[SM_ROUNDS];  // 各轮卸货横梁角度
    float   up_pick[SM_ROUNDS];    // 各轮取货下降高度
    float   up_drop[SM_ROUNDS];    // 各轮卸货下降高度
    float   up_lift;               // 升起高度
    int16_t claw_grab;             // 夹取角度
    int16_t claw_release;          // 张开角度
} Mission_t;

extern Mission_t mission;

void StateMachine_Init(void);
void SM_StartMission(const Mission_t *m);

#endif
