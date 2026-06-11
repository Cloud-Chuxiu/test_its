#ifndef _THREAD_STATEMACHINE_H__
#define _THREAD_STATEMACHINE_H__

#include "Head.h"
#include "DJI.h"
#include "chassis.h"
#include "beam.h"
#include "updown.h"
#include "Thread_FT.h"
#include "STP_Decode.h"
#include "Thread_DJI.h"
#include "Action.h"
#include <math.h>

/* 状态枚举 */
typedef enum {
    SM_IDLE = 0,
    // 旧版单步调试
    SM_CHASSIS_X, SM_BEAM_Y, SM_UPDOWN_Z, SM_CLAW,
    // 完整赛程
    SM_UPDOWN_LIFT, SM_BEAM_PICK, SM_CHASSIS_PICK, SM_UPDOWN_PICK,
    SM_CLAW_GRAB,
    SM_CHASSIS_DROP, SM_BEAM_DROP, SM_UPDOWN_DROP, SM_CLAW_RELEASE,
    SM_DONE, SM_ERROR
} SM_State;

typedef struct {
    SM_State  current_state;
    SM_State  next_state;
    // 每轮不同的坐标
    float     pick_x[SM_ROUNDS];
    float     drop_x[SM_ROUNDS];
    float     beam_pick[SM_ROUNDS];
    float     beam_drop[SM_ROUNDS];
    float     up_pick[SM_ROUNDS];
    float     up_drop[SM_ROUNDS];
    float     via_gap[SM_ROUNDS];    // 避障中继点底盘位置
    // 每轮相同的坐标
    float     up_lift;
    int16_t   claw_grab;
    int16_t   claw_release;
    // 轮次
    uint8_t   round;
    uint8_t   lift_stage;
    // 旧版目标
    float     target_x, target_y, target_z;
    int16_t   target_claw;
    // 状态管理
    uint8_t   state_entered;
    uint32_t  state_entry_tick;
    uint32_t  timeout_ms;
} StateMachine_t;

extern StateMachine_t sm;

void SM_Start();
void SM_SetTarget(float x, float y, float z, int16_t claw);
void SM_EnterState(SM_State s, uint32_t timeout);
void SM_Stop();
SM_State SM_GetState();
const char* SM_StateName(SM_State s);

#endif
