#include "Thread_StateMachine.h"

StateMachine_t sm = {
    .current_state   = SM_IDLE,
    .next_state      = SM_IDLE,
    .pick_x          = {0},
    .drop_x          = {0},
    .beam_pick       = {0},
    .beam_drop       = {0},
    .up_pick         = {0},
    .up_drop         = {0},
    .via_gap1        = {0},
    .via_gap2        = {0},
    .beam_start      = {0},
    .beam_gap        = {0},
    .up_lift         = 0,
    .claw_grab       = 0,
    .claw_release    = 0,
    .round           = 0,
    .lift_stage      = 0,
    .target_x        = 0,
    .target_y        = 0,
    .target_z        = 0,
    .target_claw     = 0,
    .state_entered   = 0,
    .timeout_ms      = 0,
};

/* ---------- 完成检测 ---------- */

static uint8_t Chassis_Done(void) {
    return (hDJI[0].speedPID.output == 0)
        && (fabs(hDJI[0].AxisData.lidar_distance - sm.target_x) < 5.0f);
}
static uint8_t Beam_Done(void) {
    return (hDJI[2].speedPID.output == 0)
        && (fabs(hDJI[2].AxisData.AxisAngle_inDegree - sm.target_y) < 3.0f);
}
static uint8_t Updown_Done(void) {
    return (hDJI[5].speedPID.output == 0)
        && (fabs(hDJI[5].AxisData.AxisAngle_inDegree - sm.target_z) < 3.0f);
}
static uint8_t Claw_Done(void) {
    return (HAL_GetTick() - sm.state_entry_tick) > 2000;
}

/* ---------- 状态跳转 ---------- */

void SM_EnterState(SM_State s, uint32_t timeout) {
    sm.current_state    = s;
    sm.state_entered    = 1;
    sm.state_entry_tick = HAL_GetTick();
    sm.timeout_ms       = timeout;
}
static void SM_CheckTimeout(void) {
    if ((HAL_GetTick() - sm.state_entry_tick) > sm.timeout_ms) {
        printf("[SM] timeout %s\r\n", SM_StateName(sm.current_state));
        sm.current_state = SM_ERROR;
    }
}

/* ---------- 主循环 ---------- */

void StateMachine_Function(void *argument)
{
    printf("[SM] ready\r\n");
    osDelay(1000);

    for (;;) {
        uint8_t r = sm.round;
        switch (sm.current_state) {

        case SM_IDLE:
            break;

        /* ===== 旧版调试 ===== */
        // case SM_CHASSIS_X:
        //     if (sm.state_entered) {
        //        // printf("[SM] chassis -> %.1f mm\r\n", sm.target_x);
        //         sm.state_entered = 0;
        //     }
        //     *pChassis_distance = sm.target_x;
        //     if (Chassis_Done()) SM_EnterState(sm.next_state, 30000);
        //     SM_CheckTimeout(); break;

        // case SM_BEAM_Y:
        //     if (sm.state_entered) {
        //         //printf("[SM] beam -> %.1f\r\n", sm.target_y);
        //         sm.state_entered = 0;
        //     }
        //     *pBeam_distance = sm.target_y;
        //     if (Beam_Done()) SM_EnterState(sm.next_state, 20000);
        //     SM_CheckTimeout(); break;

        // case SM_UPDOWN_Z:
        //     if (sm.state_entered) {
        //         //printf("[SM] updown -> %.1f\r\n", sm.target_z);
        //         sm.state_entered = 0;
        //     }
        //     *pUpdown_distance = sm.target_z;
        //     if (Updown_Done()) SM_EnterState(sm.next_state, 20000);
        //     SM_CheckTimeout(); break;

        // case SM_CLAW:
        //     if (sm.state_entered) {
        //         //printf("[SM] claw -> %d\r\n", sm.target_claw);
        //         sm.state_entered = 0;
        //     }
        //     WritePosEx(&h_FT_STS[0], sm.target_claw, 20, 50);
        //     if (Claw_Done()) SM_EnterState(sm.next_state, 10000);
        //     SM_CheckTimeout(); break;

        /* ===== 完整赛程 ===== */

        /* 1. 升降升起 */
        case SM_UPDOWN_LIFT:
            if (sm.state_entered) {
              //  printf("[SM] R%d updown lift %.1f\r\n", r+1, sm.up_lift);
                sm.state_entered = 0;
            }
            sm.target_z = sm.up_lift;
            *pUpdown_distance = sm.up_lift;
            *pBeam_distance = sm.beam_start[r];


            if (Updown_Done()) {
                if (sm.lift_stage)
                    SM_EnterState(SM_CHASSIS_DROP, 20000);
                else
                    SM_EnterState(SM_CHASSIS_PICK, 20000);
            }
            SM_CheckTimeout(); break;

        /* 2. 底盘→取货区 */
        case SM_CHASSIS_PICK:
            if (sm.state_entered) {
             //  printf("[SM] R%d chassis pick %.1f mm\r\n", r+1, sm.pick_x[r]);
                sm.state_entered = 0;
            }
            sm.target_x = sm.pick_x[r];
            *pChassis_distance = sm.pick_x[r];
            if (Chassis_Done()) SM_EnterState(SM_BEAM_PICK, 30000);
            SM_CheckTimeout(); break;
        
            /* 3. 横梁→取货侧 */
        case SM_BEAM_PICK:
            if (sm.state_entered) {
              //  printf("[SM] R%d beam pick %.1f\r\n", r+1, sm.beam_pick[r]);
                sm.state_entered = 0;
            }
            sm.target_y = sm.beam_pick[r];
            *pBeam_distance = sm.beam_pick[r];
            if (Beam_Done()) SM_EnterState(SM_UPDOWN_PICK, 20000);
            SM_CheckTimeout(); break;

       
        /* 4. 升降下降取货 */
        case SM_UPDOWN_PICK:
            if (sm.state_entered) {
              //  printf("[SM] R%d updown pick %.1f\r\n", r+1, sm.up_pick[r]);
                sm.state_entered = 0;
            }
            sm.target_z = sm.up_pick[r];
            *pUpdown_distance = sm.up_pick[r];
            if (Updown_Done()) SM_EnterState(SM_CLAW_GRAB, 20000);
            SM_CheckTimeout(); break;

        /* 5. 夹爪夹取 */
        case SM_CLAW_GRAB:
            if (sm.state_entered) {
              //  printf("[SM] R%d claw grab %d\r\n", r+1, sm.claw_grab);
                sm.state_entered = 0;
            }
            *pFT_phy = sm.claw_grab;
            if (Claw_Done()) {
                sm.lift_stage = 1;
                SM_EnterState(SM_UPDOWN_LIFT, 10000);
            }
            SM_CheckTimeout(); break;

        /* 6. 底盘→卸货区（经过避障中继点自动触发横梁摆动） */
        case SM_CHASSIS_DROP:
            if (sm.state_entered) {
                printf("[SM] R%d chassis drop %.1f, via %.1f\r\n", r+1, sm.drop_x[r], sm.via_gap1[r]);
                sm.state_entered = 0;
            }
            sm.target_x = sm.drop_x[r];
            *pChassis_distance = sm.drop_x[r];
            // 底盘到达避障中继点 → 触发横梁摆动（底盘不停）
            if (fabs(hDJI[0].AxisData.lidar_distance - sm.via_gap1[r]) < 100) {
                *pBeam_distance = sm.beam_drop[r];
            }
            if (Chassis_Done()) SM_EnterState(SM_BEAM_DROP, 30000);
            SM_CheckTimeout(); break;

        /* 7. 横梁→卸货侧 */
        case SM_BEAM_DROP:
            if (sm.state_entered) {
              //  printf("[SM] R%d beam drop %.1f\r\n", r+1, sm.beam_drop[r]);
                sm.state_entered = 0;
            }
            sm.target_y = sm.beam_drop[r];
            *pBeam_distance = sm.beam_drop[r];
            if (Beam_Done()) SM_EnterState(SM_UPDOWN_DROP, 20000);
            SM_CheckTimeout(); break;

        /* 8. 升降下降卸货 */
        case SM_UPDOWN_DROP:
            if (sm.state_entered) {
                printf("[SM] R%d updown drop %.1f\r\n", r+1, sm.up_drop[r]);
                sm.state_entered = 0;
            }
            sm.target_z = sm.up_drop[r];
            *pUpdown_distance = sm.up_drop[r];
            if (Updown_Done()) SM_EnterState(SM_CLAW_RELEASE, 20000);
            SM_CheckTimeout(); break;

        /* 9. 夹爪张开 */
        case SM_CLAW_RELEASE:
            if (sm.state_entered) {
              //  printf("[SM] R%d claw release %d\r\n", r+1, sm.claw_release);
                sm.state_entered = 0;
            }
            *pFT_phy = sm.claw_release;
            if (Claw_Done()) SM_EnterState(SM_DONE, 10000);
            SM_CheckTimeout(); break;

        /* 任务完成 → 下一轮或结束 */
        case SM_DONE:
            if (sm.state_entered) {
              //  printf("[SM] R%d done\r\n", r+1);
                sm.state_entered = 0;
            }
            sm.round++;
            if (sm.round < SM_ROUNDS) {
                sm.lift_stage = 0;
                SM_EnterState(SM_UPDOWN_LIFT, 20000);
            } else {
             //   printf("[SM] all 3 rounds complete!\r\n");
                sm.round = 0;
                sm.current_state = SM_IDLE;
            }
            break;

        case SM_ERROR:
            if (sm.state_entered) {
             //   printf("[SM] error!\r\n");
                sm.state_entered = 0;
            }
            break;

        default: break;
        }

        osDelay(20);
    }
}

/* ---------- 外部接口 ---------- */

void SM_Start() {
    osThreadId_t h;
    const osThreadAttr_t a = {
        .name = "StateMachine", .stack_size = 128 * 10,
        .priority = (osPriority_t)osPriorityAboveNormal,
    };
    osThreadNew(StateMachine_Function, NULL, &a);
}

void SM_SetTarget(float x, float y, float z, int16_t claw) {
    if (sm.current_state != SM_IDLE) return;
    sm.target_x = x; sm.target_y = y; sm.target_z = z; sm.target_claw = claw;
    sm.next_state = SM_BEAM_Y;
    SM_EnterState(SM_CHASSIS_X, 30000);
}

void SM_Stop() { sm.current_state = SM_IDLE; sm.state_entered = 1; }
SM_State SM_GetState() { return sm.current_state; }

const char* SM_StateName(SM_State s) {
    switch (s) {
        case SM_IDLE: return "IDLE";
        case SM_CHASSIS_X: return "CHASSIS_X";
        case SM_BEAM_Y: return "BEAM_Y";
        case SM_UPDOWN_Z: return "UPDOWN_Z";
        case SM_CLAW: return "CLAW";
        case SM_UPDOWN_LIFT: return "UPDOWN_LIFT";
        case SM_BEAM_PICK: return "BEAM_PICK";
        case SM_CHASSIS_PICK: return "CHASSIS_PICK";
        case SM_UPDOWN_PICK: return "UPDOWN_PICK";
        case SM_CLAW_GRAB: return "CLAW_GRAB";
        case SM_CHASSIS_DROP: return "CHASSIS_DROP";
        case SM_BEAM_DROP: return "BEAM_DROP";
        case SM_UPDOWN_DROP: return "UPDOWN_DROP";
        case SM_CLAW_RELEASE: return "CLAW_RELEASE";
        case SM_DONE: return "DONE";
        case SM_ERROR: return "ERROR";
        default: return "?";
    }
}
