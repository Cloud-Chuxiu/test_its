#ifndef __USER_SMS_STS_H__
#define __USER_SMS_STS_H__
#include "SMS_STS.h"


void FT_Write_Speed_Setup(void);
void FT_Write_Speed(void);
//void FT_Write_Position_Setup(void);
void FT_Write_Position(FT_STS_t *STS_t, int16_t position, uint16_t speed, uint8_t ACC);
void FT_Sync_Write_Position_Setup(void);
void FT_Sync_Write_Position(void);
void FT_Reg_Write_Position_Setup(void);
void FT_Reg_Write_Position(void);
void FT_Program_Eprom_Setup(void);
void FT_Program_Eprom(void);
void FT_Get_FT_ID_Setup(void);
void FT_Get_FT_ID(void);
void FT_FeedBack_Setup(void);
void FT_FeedBack(void);
void FT_Broadcast_Setup(void);
void FT_Broadcast(void);

#endif // __USER_SMS_STS_H__






















