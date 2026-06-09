#include "Action.h"
#include "Thread_StateMachine.h"

Mission_t mission;

void StateMachine_Init(void)
{
    // ===== R0：第一趟 =====
    mission.pick_x[0]     = 48;
    mission.drop_x[0]     = 3755;
    mission.beam_pick[0]  = 750;
    mission.beam_drop[0]  = -750;
    mission.up_pick[0]    = 302;
    mission.up_drop[0]    = 302;

    // ===== R1：第二趟 =====
    mission.pick_x[1]     = 302.2;    
    mission.drop_x[1]     = 3755;
    mission.beam_pick[1]  = 750;
    mission.beam_drop[1]  = -750;
    mission.up_pick[1]    = 500;
    mission.up_drop[1]    = 302;

    // ===== R2：第三趟 =====
    mission.pick_x[2]     = 48;    
    mission.drop_x[2]     = 3755;
    mission.beam_pick[2]  = 750;
    mission.beam_drop[2]  = -750;
    mission.up_pick[2]    = 500;
    mission.up_drop[2]    = 302;

    // ===== 通用参数 =====
    mission.up_lift         = 750;
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
    }
    sm.up_lift      = m->up_lift;
    sm.claw_grab    = m->claw_grab;
    sm.claw_release = m->claw_release;
    sm.round        = 0;
    sm.lift_stage   = 0;

    SM_EnterState(SM_UPDOWN_LIFT, 20000);
}
