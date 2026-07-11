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
    float   via_gap1[SM_ROUNDS];    // 各轮避障中继点底盘位置
    float   via_gap2[SM_ROUNDS];
    float   beam_start[SM_ROUNDS]; //起步横梁位
    float   beam_gap[SM_ROUNDS];   //横梁避障目的
    float   up_lift;               // 升起高度
    int16_t claw_grab;             // 夹取角度
    int16_t claw_release;          // 张开角度
} Mission_t;

typedef struct {
    float box_x[5];
    float box_y[5];
} Box_t;


extern Mission_t mission;

void StateMachine_Init(void);
void SM_StartMission(const Mission_t *m);

#endif
