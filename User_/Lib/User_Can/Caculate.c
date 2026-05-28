/*pid算法和速度位置伺服*/

#include "Caculate.h"
#include "math.h"

//增量式PID算法
void PID_Calc(PID_t *pid){
	pid->cur_error = pid->ref - pid->fdb;
	pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
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
	PID_Calc(&motor->posPID);
	
	motor->speedPID.ref = motor->posPID.output;
	motor->speedPID.fdb = motor->FdbData.rpm;
	PID_Calc(&motor->speedPID);
	
	//死区
	if(fabs(motor->posPID.fdb - ref) < 3)
	{
		motor->speedPID.output = 0;
	}
	
}


extern volatile uint32_t usart1_frame_cnt;

//底盘位置伺服函数（LIDAR绝对测距 + 编码器增量插值 + 编码器速度反馈）
void positionServo_chassis(float ref, DJI_t *motor)
{
    static float fused_pos    = 0;
    static float enc_latch    = 0;
    static uint32_t last_fr   = 0;

    /* ---- 编码器插值融合 ---- */
    if (usart1_frame_cnt != last_fr) {
        // 新LIDAR帧 → 绝对校准
        last_fr   = usart1_frame_cnt;
        fused_pos = motor->AxisData.lidar_distance;
        enc_latch = motor->AxisData.AxisAngle_inDegree;
    } else {
        // 帧间 → 编码器增量估算
        float enc_delta = motor->AxisData.AxisAngle_inDegree - enc_latch;
        fused_pos += enc_delta * CHASSIS_MM_PER_DEG;
        enc_latch  = motor->AxisData.AxisAngle_inDegree;
    }

    /* ---- 接近目标时线性减速 ---- */
    float error = fabs(fused_pos - ref);
    if (error < 200.0f) {
        motor->posPID.outputMax = CHASSIS_MAX * (error / 200.0f);
        if (motor->posPID.outputMax < 500.0f)
            motor->posPID.outputMax = 500.0f;
    } else {
        motor->posPID.outputMax = CHASSIS_MAX;
    }

    /* ---- 双层PID ---- */
    motor->posPID.ref = ref;
    motor->posPID.fdb = fused_pos;
    PID_Calc(&motor->posPID);

    motor->speedPID.ref = motor->posPID.output;
    motor->speedPID.fdb = motor->FdbData.rpm;
    PID_Calc(&motor->speedPID);

    /* ---- 死区 ---- */
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