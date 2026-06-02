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
#include <math.h>

/* 状态枚举 */
typedef enum {
    SM_IDLE = 0,
    // 旧版单步调试状态
    SM_CHASSIS_X,       // 底盘X轴移动
    SM_BEAM_Y,          // 横梁Y轴移动
    SM_UPDOWN_Z,        // 升降Z轴移动
    SM_CLAW,            // 夹爪动作
    // 完整赛程状态
    SM_CHASSIS_PICK,    // 底盘→取货区
    SM_BEAM_PICK,       // 横梁到位
    SM_UPDOWN_PICK,     // 升降下降取货
    SM_CLAW_GRAB,       // 夹爪夹取
    SM_UPDOWN_LIFT,     // 升降升起（带货）
    SM_CHASSIS_DROP,    // 底盘→卸货区
    SM_BEAM_DROP,       // 横梁→卸货侧
    SM_UPDOWN_DROP,     // 升降下降卸货
    SM_CLAW_RELEASE,    // 夹爪张开
    SM_DONE,            // 任务完成
    SM_ERROR            // 超时/异常
} SM_State;

/* 状态机控制结构体 */
typedef struct {
    SM_State  current_state;
    SM_State  next_state;          // SM_SetTarget 用
    // 赛程目标坐标
    float     pick_x;              // 取货区底盘位置 (mm)
    float     drop_x;              // 卸货区底盘位置 (mm)
    float     beam_pick;           // 取货横梁角度
    float     beam_drop;           // 卸货横梁角度
    float     up_lift;             // 升起高度
    float     up_pick;             // 取货下降高度
    float     up_drop;             // 卸货下降高度
    int16_t   claw_grab;           // 夹取角度
    int16_t   claw_release;        // 张开角度
    // 旧版目标
    float     target_x;
    float     target_y;
    float     target_z;
    int16_t   target_claw;
    // 状态管理
    uint8_t   state_entered;
    uint32_t  state_entry_tick;
    uint32_t  timeout_ms;
} StateMachine_t;

extern StateMachine_t sm;

/* 外部接口 */
void SM_Start();                     // 启动状态机线程
void SM_SetTarget(float x, float y, float z, int16_t claw);  // 旧版单步调试
void SM_StartMission(float pick_x, float drop_x,
                     float beam_pick, float beam_drop,
                     float up_lift, float up_pick, float up_drop,
                     int16_t claw_grab, int16_t claw_release); // 完整赛程
void SM_Stop();                      // 急停
SM_State SM_GetState();              // 获取当前状态
const char* SM_StateName(SM_State s);// 状态名转字符串

#endif
