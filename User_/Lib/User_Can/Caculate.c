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


//横梁位置伺服（编码器反馈 + 位置误差线性减速）
void positionServo_Beam(float ref, DJI_t *motor)
{
    static float last_ref = 0;
    if (fabs(ref - last_ref) > 0.5f) {       // 目标变化 → 积分清零，防windup
        last_ref = ref;
        motor->posPID.integral = 0;
    }

    motor->posPID.ref = ref;
    motor->posPID.fdb = motor->AxisData.AxisAngle_inDegree;
    PID_Calc_P(&motor->posPID);

    motor->speedPID.ref = motor->posPID.output;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);

    // 单向速度限制：只限朝目标冲的方向，不限反向刹车
    float s_error   = ref - motor->AxisData.AxisAngle_inDegree;
    float s_abs_err = fabs(s_error);
    if (s_abs_err < 300.0f) {
        float limit = motor->speedPID.outputMax * (s_abs_err / 300.0f);
        if (limit < 100.0f) limit = 100.0f;
        if (s_error > 0 && motor->speedPID.output >  limit)      // 朝目标正向冲→限速
            motor->speedPID.output =  limit;
        else if (s_error < 0 && motor->speedPID.output < -limit)  // 朝目标反向冲→限速
            motor->speedPID.output = -limit;
        // output与error方向相反（刹车/回退）→ 不限速，自由制动
    }

  // if (s_abs_err < 3.0f) motor->speedPID.output = 0;
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

    /* ---- 起步加速斜坡：距起点越近，输出限制越低 ---- */
    static float start_pos          = 0;
    static float last_chassis_ref   = 0;
    if (fabs(ref - last_chassis_ref) > 0.5f) {
        last_chassis_ref = ref;
        start_pos = fused_pos;
        motor->posPID.integral = 0;  // 目标变化，积分清零
    }
    float dist_start = fabs(fused_pos - start_pos);

    /* ---- 接近目标时线性减速 ---- */
    float error = fabs(fused_pos - ref);
    float decel_limit = CHASSIS_MAX;    // 减速上限（默认全输出）
    // if (error < 600.0f) {
    //     decel_limit = CHASSIS_MAX * (error / 600.0f);
    //     if (decel_limit < 1500.0f) decel_limit = 1500.0f;
    // }

    float accel_limit = CHASSIS_MAX;    // 加速上限（默认全输出）
    if (dist_start < 200.0f) {
        accel_limit = CHASSIS_MAX * (dist_start / 200.0f);
        if (accel_limit < 800.0f) accel_limit = 1500.0f;
    }

    // 取加速和减速率中更严格的那个
    motor->posPID.outputMax = (accel_limit < decel_limit) ? accel_limit : decel_limit;

    /* ---- 位置环=位置式PID, 速度环=增量式PID ---- */
    motor->posPID.ref = ref;
    motor->posPID.fdb = fused_pos;
    float saved_KI = motor->posPID.KI;
    if (error > 100.0f) motor->posPID.KI = 0;   // 大误差时关KI，防积分饱和
    PID_Calc_P(&motor->posPID);
    motor->posPID.KI = saved_KI;                 // 恢复KI

    motor->speedPID.ref = motor->posPID.output;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);

    if (error < 3.0f) {
        motor->speedPID.output = 0;
    }
}

//速度伺服函数
void speedServo(float ref, DJI_t * motor){
	motor->speedPID.ref = ref;
	motor->speedPID.fdb = motor->FdbData.rpm;
	PID_Calc(&motor->speedPID);
}