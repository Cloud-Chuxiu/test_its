#ifndef _ACTION_H__
#define _ACTION_H__

#include <stdint.h>

#define SM_ROUNDS 3
#define NUM_BOXES 3
#define NUM_DROPS 5

/* 卸货区 0-4 */
#define B1 0
#define B2 1
#define B3 2
#define B4 3
#define B5 4

/* 货箱取货坐标 */
typedef struct {
    float   chassis_x;
    float   beam_y;
    float   updown_z;
    int16_t claw_grab;
} BoxPos_t;

/* 卸货区坐标 */
typedef struct {
    float   chassis_x;
    float   beam_y;
    float   updown_z;
} DropPos_t;

/* 赛程参数（由 StateMachine_Init 自动填充） */
typedef struct {
    float   pick_x[SM_ROUNDS];
    float   drop_x[SM_ROUNDS];
    float   beam_pick[SM_ROUNDS];
    float   beam_drop[SM_ROUNDS];
    float   up_pick[SM_ROUNDS];
    float   up_drop[SM_ROUNDS];
    float   via_gap1[SM_ROUNDS];
    float   via_gap2[SM_ROUNDS];
    float   beam_start_pick[SM_ROUNDS];
    float   beam_start_drop[SM_ROUNDS];
    float   beam_gap[SM_ROUNDS];
    float   up_lift;
    int16_t claw_release;
} Mission_t;

extern const BoxPos_t  boxes[NUM_BOXES];
extern const DropPos_t drops[NUM_DROPS];
extern Mission_t mission;

void StateMachine_Init(void);
void SM_StartMission(const Mission_t *m);
void Action_SetDropDest(int round, char bean_code);

#endif
