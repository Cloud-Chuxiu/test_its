/*pid算法和速度位置伺服*/

#include "Caculate.h"
#include "math.h"

//增量式PID算法（适用于高频反馈如编码器速度环）
void PID_Calc(PID_t *pid){
	pid->cur_error = pid->ref - pid->fdb;
	pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
	pid->error[0] = pid->error[1];
	pid->error[1] = pid->ref - pid->fdb;
	/*设定输出上限*/
	if(pid->output > pid->outputMax) pid->output = pid->outputMax;
	if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;

}

//位置式PID算法（适用于低频反馈如LIDAR位置环）
void PID_Calc_P(PID_t *pid){
	pid->cur_error = pid->ref - pid->fdb;
	pid->integral += pid->cur_error;
	pid->output = pid->KP * pid->cur_error + pid->KI * pid->integral + pid->KD * (pid->cur_error - pid->error[1]);
	pid->error[0] = pid->error[1];
	pid->error[1] = pid->ref - pid->fdb;
	/*设定输出上限*/
	if(pid->output > pid->outputMax) pid->output = pid->outputMax;
	if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;
}

//比例算法
void P_Calc(PID_t *pid){
	pid->cur_error = pid->ref - pid->fdb;
	pid->output = pid->KP * pid->cur_error;
	/*设定输出上限*/
	if(pid->output > pid->outputMax) pid->output = pid->outputMax;
	if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;
	
	if(fabs(pid->output)<pid->outputMin)
		pid->output=0;

}

//位置伺服函数
void positionServo(float ref, DJI_t * motor){
	
	motor->posPID.ref = ref;
	motor->posPID.fdb = motor->AxisData.AxisAngle_inDegree;
	PID_Calc_P(&motor->posPID);
	
	motor->speedPID.ref = motor->posPID.output;
	motor->speedPID.fdb = motor->FdbData.rpm;
	PID_Calc(&motor->speedPID);
	
	//死区
	if(fabs(motor->posPID.fdb - ref) < 3)
	{
		motor->speedPID.output = 0;
	}
	
}


//横梁位置伺服（LIDAR2+编码器融合反馈，位置式PID）
extern volatile uint32_t usart3_frame_cnt;

void positionServo_Beam(float ref, DJI_t *motor)
{
    static float fused_pos    = 0;
    static float enc_latch    = 0;
    static uint32_t last_fr   = 0;

    if (usart3_frame_cnt == 0) return;  // 等待首帧LIDAR

    /* ---- 编码器插值融合 ---- */
    if (usart3_frame_cnt != last_fr) {
        last_fr   = usart3_frame_cnt;
        fused_pos = motor->AxisData.lidar_distance;
        enc_latch = motor->AxisData.AxisAngle_inDegree;
    } else {
        float enc_delta = motor->AxisData.AxisAngle_inDegree - enc_latch;
        fused_pos += enc_delta * BEAM_MM_PER_DEG;
        enc_latch  = motor->AxisData.AxisAngle_inDegree;
    }

    /* ---- 目标变化 → 积分清零 ---- */
    static float last_ref = 0;
    static float start_enc_b = 0;
    if (fabs(ref - last_ref) > 0.5f) {
        last_ref = ref;
        motor->posPID.integral = 0;
        start_enc_b = motor->AxisData.AxisAngle_inDegree;
    }

    float error = fabs(fused_pos - ref);

    /* ---- 加速/减速斜坡（用常量基准，不用变量防锁死）---- */
    float dist_start = fabs(motor->AxisData.AxisAngle_inDegree - start_enc_b) * BEAM_MM_PER_DEG;

    float accel_limit = BEAM_MAX;
    if (dist_start < 200.0f) {
        float t = dist_start / 200.0f;
        float s = t * t * (3.0f - 2.0f * t);
        accel_limit = 2000.0f + (BEAM_MAX - 2000.0f) * s;
    }

    float decel_limit = BEAM_MAX;
    if (error < 500.0f) {
        float t = error / 500.0f;
        float s = t * t * (3.0f - 2.0f * t);   // smoothstep 减速
        decel_limit = 1000.0f + (BEAM_MAX - 1000.0f) * s;
    }

    motor->posPID.outputMax = (accel_limit < decel_limit) ? accel_limit : decel_limit;

    /* ---- 位置式PID ---- */
    motor->posPID.ref = ref;
    motor->posPID.fdb = fused_pos;
    PID_Calc_P(&motor->posPID);

    motor->speedPID.ref = motor->posPID.output;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);
     if (error < 8.0f) {
        motor->posPID.output = 0;
    }
}

extern volatile uint32_t usart1_frame_cnt;
//底盘位置伺服函数（LIDAR+编码器融合反馈，位置式PID）
void positionServo_chassis(float ref, DJI_t *motor)
{
    static float fused_pos    = 0;
    static float enc_latch    = 0;
    static uint32_t last_fr   = 0;

    if (usart1_frame_cnt == 0) return;  // 等待首帧LIDAR

    /* ---- 编码器插值融合 ---- */
    if (usart1_frame_cnt != last_fr) {
        last_fr   = usart1_frame_cnt;
        fused_pos = motor->AxisData.lidar_distance;
        enc_latch = motor->AxisData.AxisAngle_inDegree;
    } else {
        float enc_delta = motor->AxisData.AxisAngle_inDegree - enc_latch;
        fused_pos += enc_delta * CHASSIS_MM_PER_DEG;
        enc_latch  = motor->AxisData.AxisAngle_inDegree;
    }

    /* ---- 目标变化 → 积分清零 ---- */
    static float start_enc          = 0;
    static float last_chassis_ref   = 0;
    if (fabs(ref - last_chassis_ref) > 0.5f) {
        last_chassis_ref = ref;
        start_enc = motor->AxisData.AxisAngle_inDegree;
        motor->posPID.integral = 0;
    }
    float dist_start = fabs(motor->AxisData.AxisAngle_inDegree - start_enc) * CHASSIS_MM_PER_DEG;
    float error = fabs(fused_pos - ref);

    /* ---- 加速斜坡 ---- */
    float accel_limit = CHASSIS_MAX;
    if (dist_start < 300.0f) {
        float t = dist_start / 300.0f;
        float s = t * t * (3.0f - 2.0f * t);  // smoothstep S曲线
        accel_limit = 2500.0f + (CHASSIS_MAX - 2500.0f) * s;
    }

    /* ---- 减速斜坡 ---- */
    float decel_limit = CHASSIS_MAX;
    if (error < 750.0f) {
        decel_limit = CHASSIS_MAX * (error / 750.0f);
        if (decel_limit < 500.0f) decel_limit = 500.0f;
    }

    motor->posPID.outputMax = (accel_limit < decel_limit) ? accel_limit : decel_limit;

    /* ---- 位置环=位置式PID, 速度环=增量式PID ---- */
    motor->posPID.ref = ref;
    motor->posPID.fdb = fused_pos;
    float saved_KI = motor->posPID.KI;
    if (error > 100.0f) motor->posPID.KI = 0;
    PID_Calc_P(&motor->posPID);
    motor->posPID.KI = saved_KI;

    motor->speedPID.ref = motor->posPID.output;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);
     if (error < 8.0f) {
        motor->posPID.output = 0;
    }
}

//速度伺服函数
void speedServo(float ref, DJI_t * motor){
	motor->speedPID.ref = ref;
	motor->speedPID.fdb = motor->FdbData.rpm;
	PID_Calc(&motor->speedPID);
}