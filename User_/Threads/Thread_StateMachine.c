#include "Thread_StateMachine.h"

StateMachine_t sm = {
    .current_state = SM_IDLE,
    .next_state    = SM_IDLE,
    .target_x      = 0,
    .target_y      = 0,
    .target_z      = 0,
    .target_claw   = 0,
    .state_entered = 0,
    .timeout_ms    = 0,
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
    // 夹爪通过时间判断（伺服电机以设定速度转动，根据角度差估算时间）
    return (HAL_GetTick() - sm.state_entry_tick) > 3000;
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
    osDelay(500);

    for (;;) {
        switch (sm.current_state) {

            /* ===== 空闲 ===== */
            case SM_IDLE:
                break;

            /* ===== 底盘X轴 ===== */
            case SM_CHASSIS_X:
                if (sm.state_entered) {
                    printf("[SM] chassis -> %.1f mm\r\n", sm.target_x);
                    sm.state_entered = 0;
                }
                chassis_move(sm.target_x);
                if (Chassis_Done()) {
                    SM_EnterState(sm.next_state, 30000);
                }
                SM_CheckTimeout();
                break;

            /* ===== 横梁Y轴 ===== */
            case SM_BEAM_Y:
                if (sm.state_entered) {
                    printf("[SM] beam -> %.1f\r\n", sm.target_y);
                    sm.state_entered = 0;
                }
                beam_move(sm.target_y);
                if (Beam_Done()) {
                    SM_EnterState(sm.next_state, 20000);
                }
                SM_CheckTimeout();
                break;

            /* ===== 升降Z轴 ===== */
            case SM_UPDOWN_Z:
                if (sm.state_entered) {
                    printf("[SM] updown -> %.1f\r\n", sm.target_z);
                    sm.state_entered = 0;
                }
                updown_move(sm.target_z);
                if (Updown_Done()) {
                    SM_EnterState(sm.next_state, 20000);
                }
                SM_CheckTimeout();
                break;

            /* ===== 夹爪动作 ===== */
            case SM_CLAW:
                if (sm.state_entered) {
                    printf("[SM] claw -> %d\r\n", sm.target_claw);
                    sm.state_entered = 0;
                }
                WritePosEx(&h_FT_STS[0], sm.target_claw, 20, 50);
                if (Claw_Done()) {
                    SM_EnterState(sm.next_state, 10000);
                }
                SM_CheckTimeout();
                break;

            /* ===== 任务完成 ===== */
            case SM_DONE:
                if (sm.state_entered) {
                    printf("[SM] done\r\n");
                    sm.state_entered = 0;
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
        osDelay(10); // 100Hz 轮询，对 10Hz LIDAR 绰绰有余
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

// 设置目标并启动一次任务流程
void SM_SetTarget(float x, float y, float z, int16_t claw)
{
    if (sm.current_state != SM_IDLE) return;

    sm.target_x    = x;
    sm.target_y    = y;
    sm.target_z    = z;
    sm.target_claw = claw;

    // 串联状态链: X → Y → Z → CLAW → DONE
    sm.next_state = SM_BEAM_Y;
    SM_EnterState(SM_CHASSIS_X, 30000);  // 底盘超时30秒
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
        case SM_IDLE:        return "IDLE";
        case SM_CHASSIS_X:   return "CHASSIS_X";
        case SM_BEAM_Y:      return "BEAM_Y";
        case SM_UPDOWN_Z:    return "UPDOWN_Z";
        case SM_CLAW:         return "CLAW";
        case SM_DONE:        return "DONE";
        case SM_ERROR:       return "ERROR";
        default:             return "?";
    }
}
