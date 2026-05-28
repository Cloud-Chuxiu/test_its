#ifndef _THREAD_STATEMACHINE_H__
#define _THREAD_STATEMACHINE_H__

#include "Head.h"
#include "DJI.h"
#include "chassis.h"
#include "beam.h"
#include "updown.h"
#include "Thread_FT.h"
#include "STP_Decode.h"
#include <math.h>
/* 状态枚举 */
typedef enum {
    SM_IDLE = 0,
    SM_CHASSIS_X,       // 底盘X轴移动
    SM_BEAM_Y,          // 横梁Y轴移动
    SM_UPDOWN_Z,        // 升降Z轴移动
    SM_CLAW,            // 夹爪动作
    SM_DONE,            // 任务完成
    SM_ERROR            // 超时/异常
} SM_State;

/* 状态机控制结构体 */
typedef struct {
    SM_State  current_state;
    SM_State  next_state;
    float     target_x;          // 底盘目标位置 (mm)
    float     target_y;          // 横梁目标位置 (degree)
    float     target_z;          // 升降目标位置 (degree)
    int16_t   target_claw;       // 夹爪目标角度
    uint8_t   state_entered;     // 进入新状态标志
    uint32_t  state_entry_tick;  // 进入状态时的系统tick
    uint32_t  timeout_ms;        // 当前状态超时时间
} StateMachine_t;

extern StateMachine_t sm;

/* 外部接口 */
void SM_Start();                     // 启动状态机线程
void SM_SetTarget(float x, float y, float z, int16_t claw);  // 设置目标并启动任务
void SM_Stop();                      // 急停
SM_State SM_GetState();              // 获取当前状态
const char* SM_StateName(SM_State s);// 状态名转字符串

#endif
