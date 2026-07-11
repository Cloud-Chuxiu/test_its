#include "Action.h"
#include "Thread_StateMachine.h"

Mission_t mission;

Box_t box_pos;

void Box_Init()
{
    box_pos.box_x[0] = 0;
    box_pos.box_y[0] = 0;
    box_pos.box_x[1] = 0;
    box_pos.box_y[1] = 0;
    box_pos.box_x[2] = 0;
    box_pos.box_y[2] = 0;
    box_pos.box_x[3] = 0;
    box_pos.box_y[3] = 0;
    box_pos.box_x[4] = 0;
    box_pos.box_y[4] = 0;
}





void StateMachine_Init(void)
{
    // ===== R0：第一趟 =====
    mission.pick_x[0]     = 48;
    mission.drop_x[0]     = 3755;
    mission.beam_pick[0]  = 750;
    mission.beam_drop[0]  = -710;
    mission.up_pick[0]    = 260;
    mission.up_drop[0]    = 302;

    // ===== R1：第二趟 =====
    mission.pick_x[1]     = 285;    
    mission.drop_x[1]     = 3755;
    mission.beam_pick[1]  = -20;
    mission.beam_drop[1]  = 550;
    mission.up_pick[1]    = 500;
    mission.up_drop[1]    = 302;

    // ===== R2：第三趟 =====
    mission.pick_x[2]     = 48;    
    mission.drop_x[2]     = 3755;
    mission.beam_pick[2]  = -840;
    mission.beam_drop[2]  = 0;
    mission.up_pick[2]    = 380;
    mission.up_drop[2]    = 302;

    // ===== 避障中继点（底盘LIDAR位置，经过此处时横梁摆动）=====
    mission.via_gap1[0]     = 700;
    mission.via_gap1[1]     = 800;
    mission.via_gap1[2]     = 800;

    // ===== 回头中继点（底盘LIDAR位置，经过此处时横梁摆动）=====
    mission.via_gap2[0]     = 3000;
    mission.via_gap2[1]     = 3000;    
    mission.via_gap2[2]     = 3000;    


    //====== 横梁起步位===========
    mission.beam_start[0]   = 750;
    mission.beam_start[1]   = -750; 
    mission.beam_start[2]   = -750; 

    //======横梁避障目的 ========
    mission.beam_gap[0]     = -750;
    mission.beam_gap[1]     = 750;
    mission.beam_gap[2]     = 750;



    // ===== 通用参数 =====
    mission.up_lift         = 800;
    mission.claw_grab       = 1650;
    mission.claw_release    = 2000;
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
    }
    sm.up_lift      = m->up_lift;
    sm.claw_grab    = m->claw_grab;
    sm.claw_release = m->claw_release;
    sm.round        = 0;
    sm.lift_stage   = 0;

    SM_EnterState(SM_UPDOWN_LIFT, 20000);
}
