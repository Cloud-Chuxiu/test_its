#include "Thread_StateMachine.h"

StateMachine_t sm = {
    .current_state   = SM_IDLE,
    .next_state      = SM_IDLE,
    .pick_x          = 0,
    .drop_x          = 0,
    .beam_pick       = 0,
    .beam_drop       = 0,
    .up_lift         = 0,
    .up_pick         = 0,
    .up_drop         = 0,
    .claw_grab       = 0,
    .claw_release    = 0,
    .target_x        = 0,
    .target_y        = 0,
    .target_z        = 0,
    .target_claw     = 0,
    .lift_stage      = 0,
    .state_entered   = 0,
    .timeout_ms      = 0,
};

/* ---------- 状态完成检测 ---------- */

static uint8_t Chassis_Done(void)
{
    return (hDJI[0].speedPID.output == 0)
        && (fabs(hDJI[0].AxisData.lidar_distance - sm.target_x) < 5.0f);
}

static uint8_t Beam_Done(void)
{
    return (hDJI[2].speedPID.output == 0)
        && (fabs(hDJI[2].AxisData.AxisAngle_inDegree - sm.target_y) < 3.0f);
}

static uint8_t Updown_Done(void)
{
    return (hDJI[5].speedPID.output == 0)
        && (fabs(hDJI[5].AxisData.AxisAngle_inDegree - sm.target_z) < 3.0f);
}

static uint8_t Claw_Done(void)
{
    return (HAL_GetTick() - sm.state_entry_tick) > 2000;
}

/* ---------- 状态跳转 ---------- */

static void SM_EnterState(SM_State new_state, uint32_t timeout)
{
    sm.current_state    = new_state;
    sm.state_entered    = 1;
    sm.state_entry_tick = HAL_GetTick();
    sm.timeout_ms       = timeout;
}

static void SM_CheckTimeout(void)
{
    if ((HAL_GetTick() - sm.state_entry_tick) > sm.timeout_ms) {
        printf("[SM] timeout in %s\r\n", SM_StateName(sm.current_state));
        sm.current_state = SM_ERROR;
    }
}

/* ---------- 状态机主循环 ---------- */

void StateMachine_Function(void *argument)
{
    printf("[SM] ready\r\n");
    osDelay(1000);

    for (;;) {
        switch (sm.current_state) {

            /* ===== 空闲 ===== */
            case SM_IDLE:
                break;
            /* ============ 完整赛程 ============ */
            /* 1. 升降升起 */
            case SM_UPDOWN_LIFT:
                if (sm.state_entered) {
                    printf("[SM] updown -> lift %.1f\r\n", sm.up_lift);
                    sm.state_entered = 0;
                }
                sm.target_z = sm.up_lift;
                *pUpdown_distance = sm.up_lift;
                if (Updown_Done()) {
                    if (sm.lift_stage) {
                        SM_EnterState(SM_CHASSIS_DROP, 20000);  // 夹取后升起→卸货
                    } else {
                        SM_EnterState(SM_BEAM_PICK, 20000);     // 初始升起→取货
                    }
                }
                SM_CheckTimeout();
                break;

            /* 2. 横梁→取货侧 */
            case SM_BEAM_PICK:
                if (sm.state_entered) {
                    printf("[SM] beam -> pick %.1f\r\n", sm.beam_pick);
                    sm.state_entered = 0;
                }
                sm.target_y = sm.beam_pick;
                *pBeam_distance = sm.beam_pick;
                if (Beam_Done()) SM_EnterState(SM_CHASSIS_PICK, 20000);
                SM_CheckTimeout();
                break;

            /* 3. 底盘→取货区 */
            case SM_CHASSIS_PICK:
                if (sm.state_entered) {
                    printf("[SM] chassis -> pick %.1f mm\r\n", sm.pick_x);
                    sm.state_entered = 0;
                }
                sm.target_x = sm.pick_x;
                *pChassis_distance = sm.pick_x;
                if (Chassis_Done()) SM_EnterState(SM_UPDOWN_PICK, 30000);
                SM_CheckTimeout();
                break;

            /* 4. 升降下降取货 */
            case SM_UPDOWN_PICK:
                if (sm.state_entered) {
                    printf("[SM] updown -> pick %.1f\r\n", sm.up_pick);
                    sm.state_entered = 0;
                }
                sm.target_z = sm.up_pick;
                *pUpdown_distance = sm.up_pick;
                if (Updown_Done()) SM_EnterState(SM_CLAW_GRAB, 20000);
                SM_CheckTimeout();
                break;

            /* 5. 夹爪夹取 */
            case SM_CLAW_GRAB:
                if (sm.state_entered) {
                    printf("[SM] claw -> grab %d\r\n", sm.claw_grab);
                    sm.state_entered = 0;
                }
                *pFT_phy = sm.claw_grab;
                if (Claw_Done()) {
                    sm.lift_stage = 1;  // 标记：下次升起后去卸货
                    SM_EnterState(SM_UPDOWN_LIFT, 10000);
                }
                SM_CheckTimeout();
                break;

            /* 6. 升降升起后 → 底盘→卸货区 */
            case SM_CHASSIS_DROP:
                if (sm.state_entered) {
                    printf("[SM] chassis -> drop %.1f mm\r\n", sm.drop_x);
                    sm.state_entered = 0;
                }
                sm.target_x = sm.drop_x;
                *pChassis_distance = sm.drop_x;
                if (Chassis_Done()) SM_EnterState(SM_BEAM_DROP, 30000);
                SM_CheckTimeout();
                break;

            /* 7. 横梁→卸货侧 */
            case SM_BEAM_DROP:
                if (sm.state_entered) {
                    printf("[SM] beam -> drop %.1f\r\n", sm.beam_drop);
                    sm.state_entered = 0;
                }
                sm.target_y = sm.beam_drop;
                *pBeam_distance = sm.beam_drop;
                if (Beam_Done()) SM_EnterState(SM_UPDOWN_DROP, 20000);
                SM_CheckTimeout();
                break;

            /* 8. 升降下降卸货 */
            case SM_UPDOWN_DROP:
                if (sm.state_entered) {
                    printf("[SM] updown -> drop %.1f\r\n", sm.up_drop);
                    sm.state_entered = 0;
                }
                sm.target_z = sm.up_drop;
                *pUpdown_distance = sm.up_drop;
                if (Updown_Done()) SM_EnterState(SM_CLAW_RELEASE, 20000);
                SM_CheckTimeout();
                break;

            /* 9. 夹爪张开 */
            case SM_CLAW_RELEASE:
                if (sm.state_entered) {
                    printf("[SM] claw -> release %d\r\n", sm.claw_release);
                    sm.state_entered = 0;
                }
                *pFT_phy = sm.claw_release;
                if (Claw_Done()) SM_EnterState(SM_DONE, 10000);
                SM_CheckTimeout();
                break;

            /* ===== 任务完成 ===== */
            case SM_DONE:
                if (sm.state_entered) 
                {
                    printf("[SM] mission done!\r\n");
                    sm.state_entered = 0;
                    *pUpdown_distance = 750;
                    *pBeam_distance = 0;
                    osDelay(500);
                    *pChassis_distance = 2000;

                }
                sm.current_state = SM_IDLE;
                break;

            /* ===== 异常 ===== */
            case SM_ERROR:
                if (sm.state_entered) {
                    printf("[SM] error!\r\n");
                    sm.state_entered = 0;
                }
                break;

            default:
                break;
        }
        osDelay(100);
    }
}

/* ---------- 外部接口 ---------- */

void SM_Start()
{
    osThreadId_t SM_Handle;
    const osThreadAttr_t SM_attributes = {
        .name       = "StateMachine",
        .stack_size = 128 * 10,
        .priority   = (osPriority_t)osPriorityNormal,
    };
    osThreadNew(StateMachine_Function, NULL, &SM_attributes);
}

// 旧版：单次取货
void SM_SetTarget(float x, float y, float z, int16_t claw)
{
    if (sm.current_state != SM_IDLE) return;
    sm.target_x    = x;
    sm.target_y    = y;
    sm.target_z    = z;
    sm.target_claw = claw;
    sm.next_state  = SM_BEAM_Y;
    SM_EnterState(SM_CHASSIS_X, 30000);
}

// 完整赛程
void SM_StartMission(float pick_x, float drop_x,
                     float beam_pick, float beam_drop,
                     float up_lift, float up_pick, float up_drop,
                     int16_t claw_grab, int16_t claw_release)
{
    if (sm.current_state != SM_IDLE) return;
    sm.pick_x       = pick_x;
    sm.drop_x       = drop_x;
    sm.beam_pick    = beam_pick;
    sm.beam_drop    = beam_drop;
    sm.up_lift      = up_lift;
    sm.up_pick      = up_pick;
    sm.up_drop      = up_drop;
    sm.claw_grab    = claw_grab;
    sm.claw_release = claw_release;
    sm.lift_stage   = 0;
    SM_EnterState(SM_UPDOWN_LIFT, 20000);
}

void SM_Stop()
{
    sm.current_state = SM_IDLE;
    sm.state_entered = 1;
}

SM_State SM_GetState()
{
    return sm.current_state;
}

const char* SM_StateName(SM_State s)
{
    switch (s) {
        case SM_IDLE:          return "IDLE";
        case SM_CHASSIS_X:     return "CHASSIS_X";
        case SM_BEAM_Y:        return "BEAM_Y";
        case SM_UPDOWN_Z:      return "UPDOWN_Z";
        case SM_CLAW:          return "CLAW";
        case SM_UPDOWN_LIFT:   return "UPDOWN_LIFT";
        case SM_BEAM_PICK:     return "BEAM_PICK";
        case SM_CHASSIS_PICK:  return "CHASSIS_PICK";
        case SM_UPDOWN_PICK:   return "UPDOWN_PICK";
        case SM_CLAW_GRAB:     return "CLAW_GRAB";
        case SM_CHASSIS_DROP:  return "CHASSIS_DROP";
        case SM_BEAM_DROP:     return "BEAM_DROP";
        case SM_UPDOWN_DROP:   return "UPDOWN_DROP";
        case SM_CLAW_RELEASE:  return "CLAW_RELEASE";
        case SM_DONE:          return "DONE";
        case SM_ERROR:         return "ERROR";
        default:               return "?";
    }
}
