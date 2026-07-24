#include "Action.h"
#include "Thread_StateMachine.h"

/* ===== 3 个货箱取货坐标 ===== */
const BoxPos_t boxes[NUM_BOXES] = {
    // {chassis_x, beam_y, updown_z, claw_grab}
       {70,        1422,    265,      1650},   // 箱1: 最左侧
       {344,       924,     500,      1650},   // 箱2: 中间
       {70,        430,     387,      1650},   // 箱3: 最右侧
};

/* ===== 5 个卸货区坐标 ===== */
const DropPos_t drops[NUM_DROPS] = {
    // {chassis_x, beam_y, updown_z}
       {3530,      50,      710},   // B1
       {3770,      526,     330},   // B2
       {3770,      911,     330},   // B3
       {3770,      1331,    330},   // B4 
       {3530,      1790,    710},   // B5 
};

Mission_t mission;

void StateMachine_Init(void)
{
    for (int i = 0; i < SM_ROUNDS; i++) {
        // 取货参数来自货箱（固定）
        mission.pick_x[i]    = boxes[i].chassis_x;
        mission.beam_pick[i] = boxes[i].beam_y;
        mission.up_pick[i]   = boxes[i].updown_z;

        // 卸货目的地由视觉动态填入，初始置0
        mission.drop_x[i]    = 0;
        mission.beam_drop[i] = 0;
        mission.up_drop[i]   = 0;
    }

    // ===== 避障中继点 =====
    mission.via_gap1[0] = 700;
    mission.via_gap1[1] = 700;
    mission.via_gap1[2] = 700;

    mission.via_gap2[0] = 2700;
    mission.via_gap2[1] = 2700;
    mission.via_gap2[2] = 2700;

    // ===== 横梁起步位（先用默认值，Action_SetDropDest 会修正）=====
    mission.beam_start_pick[0] = 1422;
    mission.beam_start_pick[1] = 1422;
    mission.beam_start_pick[2] = 1422;


    mission.beam_start_drop[0] = 1422;
    mission.beam_start_drop[1] = 1422;
    mission.beam_start_drop[2] = 430;

    // ===== 横梁避障目的 =====
    mission.beam_gap[0] = 526;
    mission.beam_gap[1] = 526;
    mission.beam_gap[2] = 1331;

    // ===== 通用参数 =====
    mission.up_lift      = 800;
    mission.claw_release = 2200;
}

/* 根据豆子码动态设置本轮卸货目的地，并重算关联的避障参数 */
void Action_SetDropDest(int round, char bean_code)
{
    if (round >= SM_ROUNDS) return;

    // 在 box_order 中查找 bean_code（箱子编号）的位置 → B区索引
    int drop_idx = -1;
    for (int i = 0; i < 5; i++) {
        if (sm.box_order[i] == bean_code) {
            drop_idx = i;
            break;
        }
    }
    if (drop_idx < 0 || drop_idx >= NUM_DROPS) return;  // 未找到或无效

    // 写入 sm 的卸货目的地
    sm.drop_x[round]    = drops[drop_idx].chassis_x;
    sm.beam_drop[round] = drops[drop_idx].beam_y;
    sm.up_drop[round]   = drops[drop_idx].updown_z;

    // ---- 重算受影响的 beam_start_pick（下一轮取货起步位依赖本轮 beam_drop）----
    if (round + 1 < SM_ROUNDS) {
        if (sm.beam_drop[round] < 900.0f)
            sm.beam_start_pick[round + 1] = 430;
        else
            sm.beam_start_pick[round + 1] = 1422;
    }

    // ---- 重算受影响的 beam_start_drop ----
    if (round == 0) {
        sm.beam_start_drop[0] = 1422;
    }
    if (round == 1) {
        if (sm.beam_drop[1] < 900.0f)
            sm.beam_start_drop[1] = 1422;
        else
            sm.beam_start_drop[1] = 430;
    }
    // round==2: beam_start_drop[2] 恒为 430，不需重算

    // ---- 重算受影响的 beam_gap ----
    if (round == 1) {
        // beam_gap[1] 依赖 beam_start_drop[1]
        if (sm.beam_start_drop[1] == 430)
            sm.beam_gap[1] = 1331;
        else
            sm.beam_gap[1] = 526;
    }

    // ---- 卸货在障碍区之外 → 避障目的地设为卸货目的地 ----
    if (sm.beam_drop[round] < 526.0f && sm.beam_gap[round] == 526)
        sm.beam_gap[round] = sm.beam_drop[round];
    else if (sm.beam_drop[round] > 1331.0f && sm.beam_gap[round] == 1331)
        sm.beam_gap[round] = sm.beam_drop[round];
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
        sm.beam_start_pick[i] = m->beam_start_pick[i];
        sm.beam_start_drop[i] = m->beam_start_drop[i];
        sm.beam_gap[i]        = m->beam_gap[i];
        sm.claw_grab[i] = boxes[i].claw_grab;
    }
    sm.up_lift      = m->up_lift;
    sm.claw_release = m->claw_release;
    sm.round        = 0;
    sm.lift_stage   = 0;

    SM_EnterState(SM_CAMERA_BOX_ORDER, 50000);
}
