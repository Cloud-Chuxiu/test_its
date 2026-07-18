#include "Action.h"
#include "Thread_StateMachine.h"

/* ===== 3 个货箱取货坐标 ===== */
const BoxPos_t boxes[NUM_BOXES] = {
    // {chassis_x, beam_y, updown_z, claw_grab}
       {48,        1422,    260,      1650},   // 箱1: 最左侧
       {285,       911,     500,      1650},   // 箱2: 中间
       {48,        507,     380,      1650},   // 箱3: 最右侧
};

/* ===== 5 个卸货区坐标 ===== */
const DropPos_t drops[NUM_DROPS] = {
    // {chassis_x, beam_y, updown_z}
       {3534,      50,      302},   // B1
       {3755,      526,     302},   // B2
       {3755,      911,     302},   // B3
       {3755,      1331,    302},   // B4 
       {3534,      1790,    302},   // B5 
};

Mission_t mission;

void StateMachine_Init(int d1, int d2, int d3)
{
    int drop_idx[3] = {d1, d2, d3};

    for (int i = 0; i < SM_ROUNDS; i++) {
        int d = drop_idx[i];

        // 取货参数来自货箱
        mission.pick_x[i]    = boxes[i].chassis_x;
        mission.beam_pick[i] = boxes[i].beam_y;
        mission.up_pick[i]   = boxes[i].updown_z;

        // 卸货参数来自卸货区
        mission.drop_x[i]    = drops[d].chassis_x;
        mission.beam_drop[i] = drops[d].beam_y;
        mission.up_drop[i]   = drops[d].updown_z;
    }

    // ===== 避障中继点 =====
    mission.via_gap1[0] = 700;
    mission.via_gap1[1] = 700;
    mission.via_gap1[2] = 700;

    mission.via_gap2[0] = 2700;
    mission.via_gap2[1] = 2700;
    mission.via_gap2[2] = 2700;

    // ===== 横梁起步位 =====
    mission.beam_start[0] = 1422;
    mission.beam_start[1] = 507;
    mission.beam_start[2] = 507;

    // ===== 横梁避障目的 =====
    mission.beam_gap[0] = 526;
    mission.beam_gap[1] = 1331;
    mission.beam_gap[2] = 1331;

    // ===== 卸货在障碍区之外 → 避障目的地设为卸货目的地 =====
    for (int i = 0; i < SM_ROUNDS; i++) {
        if (mission.beam_drop[i] < 526.0f && mission.beam_gap[i] == 526)
            mission.beam_gap[i] = mission.beam_drop[i];   // 左侧外: 直接到50
        else if (mission.beam_drop[i] > 1331.0f && mission.beam_gap[i] == 1331)
            mission.beam_gap[i] = mission.beam_drop[i];   // 右侧外: 直接到1790
        // 526~1331 之间: 保持 mission.beam_gap 原值
    }

    // ===== 通用参数 =====
    mission.up_lift      = 800;
    mission.claw_release = 2000;
}

void SM_StartMission(const Mission_t *m)
{
    if (sm.current_state != SM_IDLE) return;

    for (int i = 0; i < SM_ROUNDS; i++) {
        sm.pick_x[i]    = m->pick_x[i];
        sm.drop_x[i]    = m->drop_x[i];
        sm.beam_pick[i] = m->beam_pick[i];
        sm.beam_drop[i] = m->beam_drop[i];
        sm.up_pick[i]   = m->up_pick[i];
        sm.up_drop[i]   = m->up_drop[i];
        sm.via_gap1[i]  = m->via_gap1[i];
        sm.via_gap2[i]  = m->via_gap2[i];
        sm.beam_start[i]= m->beam_start[i];
        sm.beam_gap[i]  = m->beam_gap[i];
        sm.claw_grab[i] = boxes[i].claw_grab;
    }
    sm.up_lift      = m->up_lift;
    sm.claw_release = m->claw_release;
    sm.round        = 0;
    sm.lift_stage   = 0;

    SM_EnterState(SM_UPDOWN_LIFT, 20000);
}
