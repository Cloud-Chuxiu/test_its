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


//底盘位置伺服函数（激光雷达位置反馈 + 编码器速度反馈）
void positionServo_chassis(float ref, DJI_t *motor){

	float error = fabs(motor->AxisData.lidar_distance - ref);
	if (error < 200.0f) {
    motor->posPID.outputMax = CHASSIS_MAX * (error / 200.0f);
    if (motor->posPID.outputMax < 500.0f)
        motor->posPID.outputMax = 500.0f;  // 最低速度保证能到达
	} else {
    motor->posPID.outputMax = CHASSIS_MAX;
	}

	motor->posPID.ref = ref;
	motor->posPID.fdb = motor->AxisData.lidar_distance;
	PID_Calc(&motor->posPID);

	motor->speedPID.ref = motor->posPID.output;
	motor->speedPID.fdb = motor->FdbData.rpm;
	PID_Calc(&motor->speedPID);

	if(fabs(motor->AxisData.lidar_distance - ref) < 3)
	{
		motor->speedPID.output = 0;
	}
}

//速度伺服函数
void speedServo(float ref, DJI_t * motor){
	motor->speedPID.ref = ref;
	motor->speedPID.fdb = motor->FdbData.rpm;
	PID_Calc(&motor->speedPID);
}