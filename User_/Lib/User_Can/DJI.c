/*包含DJI的初始化，DJI的Transmit，DJI的解码算法*/

#include "DJI.h"


DJI_t hDJI[8];
//使用DJI init前需要指定motorType

//大疆电机初始化
void DJI_Init()
{
	for (int i = 0; i < 8; i++)
    {
	 //底盘电机的pid参数
	 if(i == 0 || i == 1)
 	 {
		hDJI[i].motorType = M3508;
        hDJI[i].reductionRate = 3591.0f / 187.0f;
        hDJI[i].encoder_resolution = 8192.0f;
        //speedPID
        hDJI[i].speedPID.KP = 12;
        hDJI[i].speedPID.KI = 0.2;
        hDJI[i].speedPID.KD = 5;
        hDJI[i].speedPID.outputMax = 5000;  //speed limit
		//posPID
		hDJI[i].posPID.KP = 11.0f;
        hDJI[i].posPID.KI = 0.0f;
        hDJI[i].posPID.KD = 6.5f;
        hDJI[i].posPID.outputMax = 5000;
	 }
	 //升降结构的pid参数
	 else if(i == 5)
	 {
		//speedPID
        hDJI[i].speedPID.KP = 12;
        hDJI[i].speedPID.KI = 0.2;
        hDJI[i].speedPID.KD = 5;
        hDJI[i].speedPID.outputMax = 3000
		;  //speed limit
		//posPID

		hDJI[i].posPID.KP = 30.0f;
        hDJI[i].posPID.KI = 0.0f;
        hDJI[i].posPID.KD = 10.0f;
        hDJI[i].posPID.outputMax = 3000;

		//hDJI[i].posPID.outputMin = 1500;
	 }
	 //云台pid
	 else if(i == 2)
	 {
		//PID初始化
		//可能不同电机需要设定不同的参数
		//speedPID
        hDJI[i].speedPID.KP = 12;
        hDJI[i].speedPID.KI = 0.2;
        hDJI[i].speedPID.KD = 5;
        hDJI[i].speedPID.outputMax = 4000
		;  //speed limit
		//posPID
		hDJI[i].posPID.KP = 18.0f;
        hDJI[i].posPID.KI = 0.0f;
        hDJI[i].posPID.KD = 10.0f;
        hDJI[i].posPID.outputMax = 4000;
		//hDJI[i].posPID.outputMin = 1500;
	 }
	 else{
		//PID初始化
		//可能不同电机需要设定不同的参数
		//speedPID
        hDJI[i].speedPID.KP = 12;
        hDJI[i].speedPID.KI = 0.2;
        hDJI[i].speedPID.KD = 5;
        hDJI[i].speedPID.outputMax = 2000
		;  //speed limit
		//posPID
		hDJI[i].posPID.KP = 80.0f;
        hDJI[i].posPID.KI = 2.0f;
        hDJI[i].posPID.KD = 0.0f;
        hDJI[i].posPID.outputMax = 4000;
		//hDJI[i].posPID.outputMin = 1500;
	 }

		if( hDJI[i].motorType == M3508 ){
			hDJI[i].reductionRate = 3591.0f/187.0f;//2006减速比为36 3508减速比约为19
		}
		else if(hDJI[i].motorType == M2006){
			hDJI[i].reductionRate = 36.0f;

		}

        hDJI[i].encoder_resolution = 8192.0f;
    }

}

static uint32_t TxMailbox;

void CanTransmit_DJI_1234(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.DLC   = 0x08;
    TxMessage.StdId = 0x200;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;

    uint8_t TxData[8];
    TxData[0] = (uint8_t)(cm1_iq >> 8);
    TxData[1] = (uint8_t)cm1_iq;
    TxData[2] = (uint8_t)(cm2_iq >> 8);
    TxData[3] = (uint8_t)cm2_iq;
    TxData[4] = (uint8_t)(cm3_iq >> 8);
    TxData[5] = (uint8_t)cm3_iq;
    TxData[6] = (uint8_t)(cm4_iq >> 8);
    TxData[7] = (uint8_t)cm4_iq;
    uint32_t tx_timeout = HAL_GetTick() + 5;
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0) {
        if ((int32_t)(HAL_GetTick() - tx_timeout) > 0) break;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcanx) > 0) {
        HAL_CAN_AddTxMessage(hcanx, &TxMessage, TxData, &TxMailbox);
    }
}

void CanTransmit_DJI_5678(CAN_HandleTypeDef *hcanx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq)
{
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.DLC   = 0x08;
    TxMessage.StdId = 0x1FF;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;

    uint8_t TxData[8];
    TxData[0] = (uint8_t)(cm5_iq >> 8);
    TxData[1] = (uint8_t)cm5_iq;
    TxData[2] = (uint8_t)(cm6_iq >> 8);
    TxData[3] = (uint8_t)cm6_iq;
    TxData[4] = (uint8_t)(cm7_iq >> 8);
    TxData[5] = (uint8_t)cm7_iq;
    TxData[6] = (uint8_t)(cm8_iq >> 8);
    TxData[7] = (uint8_t)cm8_iq;

    uint32_t tx_timeout = HAL_GetTick() + 5;
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0) {
        if ((int32_t)(HAL_GetTick() - tx_timeout) > 0) break;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcanx) > 0) {
        HAL_CAN_AddTxMessage(hcanx, &TxMessage, TxData, &TxMailbox);
    }
}

//上传与处理DJI电机反馈信息
void DJI_Update(DJI_t *motor, uint8_t* fdbData){
	/*  反馈信息计算  */
	motor->FdbData.RotorAngle_0_360              =   (fdbData[0]<<8|fdbData[1])*360.0f/motor->encoder_resolution ;     /* unit:degree*/
	motor->FdbData.rpm                      =   (int16_t)(fdbData[2]<<8|fdbData[3]);                /* unit:rom   */
	motor->FdbData.current = (int16_t)(fdbData[4]<<8|fdbData[5]);
	/*  计算数据处理  */
	/*  更新反馈速度/位置  */
	motor->Calculate.RotorAngle_0_360_Log[LAST]  =   motor->Calculate.RotorAngle_0_360_Log[NOW];
	motor->Calculate.RotorAngle_0_360_Log[NOW]   =   motor->FdbData.RotorAngle_0_360;
	/* 电机圈数更新        */
	if(motor->Calculate.RotorAngle_0_360_Log[NOW] -  motor->Calculate.RotorAngle_0_360_Log[LAST] > (180.0f))
		motor->Calculate.RotorRound--;
	else if(motor->Calculate.RotorAngle_0_360_Log[NOW] - motor->Calculate.RotorAngle_0_360_Log[LAST] < -(180.0))
		motor->Calculate.RotorRound++;
	/* 电机输出轴角度      */
	motor->AxisData.AxisAngle_inDegree  =  motor->Calculate.RotorRound * 360.0f ;
	motor->AxisData.AxisAngle_inDegree  += motor->Calculate.RotorAngle_0_360_Log[NOW] - motor->Calculate.RotorAngle_0_360_OffSet;
	motor->AxisData.AxisAngle_inDegree  /= motor->reductionRate;

	motor->AxisData.AxisVelocity        =  motor->FdbData.rpm / motor->reductionRate;
	motor->Calculate.RotorAngle_all		  =  motor->Calculate.RotorRound * 360 + motor->Calculate.RotorAngle_0_360_Log[NOW] - motor->Calculate.RotorAngle_0_360_OffSet;
}

//获取dji电机反馈信息
void get_dji_offset(DJI_t *motor, uint8_t* fdbData){
	motor->FdbData.RotorAngle_0_360 = (fdbData[0]<<8|fdbData[1])*360.0f/motor->encoder_resolution;
	motor->Calculate.RotorAngle_0_360_Log[LAST] = motor->FdbData.RotorAngle_0_360;
	motor->Calculate.RotorAngle_0_360_Log[NOW] = motor->Calculate.RotorAngle_0_360_Log[LAST];

	motor->Calculate.RotorAngle_0_360_OffSet = motor->FdbData.RotorAngle_0_360;
}



//解码电机回传信息
HAL_StatusTypeDef DJI_CanMsgDecode(uint32_t Stdid, uint8_t* fdbData){
	int i=Stdid - 0x201;
	if(i>=0 &&i<8){
		if(hDJI[i].FdbData.msg_cnt<50)
		{
			get_dji_offset(&hDJI[i], fdbData);
			hDJI[i].FdbData.msg_cnt++;
		}
		else
		{
			DJI_Update(&hDJI[i], fdbData);
		}
		return HAL_OK;
	}
	return HAL_ERROR;
}
