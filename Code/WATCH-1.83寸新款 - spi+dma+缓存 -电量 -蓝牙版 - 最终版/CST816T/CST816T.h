#ifndef __CST816T_H__
#define __CST816T_H__
// #include "stdint.h"


#define CST816T_I2CAddress_7bit 0x15
#define CST816T_I2CAddress_8bit_Read 	(CST816T_I2CAddress_7bit << 1) | 0x01
#define CST816T_I2CAddress_8bit_Write (CST816T_I2CAddress_7bit << 1)

#define CST816T_Register_GestureID			0x01	
#define CST816T_Register_FingerNum			0x02	
#define CST816T_Register_XposH					0x03	
#define CST816T_Register_XposL					0x04	
#define CST816T_Register_YposH					0x05	
#define CST816T_Register_YposL					0x06	
#define CST816T_Register_ChipID					0xA7	
#define CST816T_Register_FwVersion			0xA9	
#define CST816T_Register_FactoryID 			0xAA	
#define CST816T_Register_BPC0H					0xB0	
#define CST816T_Register_BPC0L					0xB1	
#define CST816T_Register_BPC1H					0xB2	
#define CST816T_Register_BPC1L 					0xB3	
#define CST816T_Register_SleepMode 			0xE5	
#define CST816T_Register_ErrResetCtl 		0xEA	
#define CST816T_Register_LongPressTick 	0xEB	
#define CST816T_Register_MotionMask 		0xEC	
#define CST816T_Register_IrqPluseWidth 	0xED	
#define CST816T_Register_NorScanPer 		0xEE
#define CST816T_Register_MotionSlAngle 	0xEF	
#define CST816T_Register_LpScanRaw1H 		0xF0	
#define CST816T_Register_LpScanRaw1L 		0xF1	
#define CST816T_Register_LpScanRaw2H 		0xF2	
#define CST816T_Register_LpScanRaw2L 		0xF3	
#define CST816T_Register_LpAutoWakeTime 0xF4	
#define CST816T_Register_LpScanTH 			0xF5	
#define CST816T_Register_LpScanWin 			0xF6	
#define CST816T_Register_LpScanFreq 		0xF7	
#define CST816T_Register_LpScanIdac 		0xF8	
#define CST816T_Register_AutoSleepTime 	0xF9	
#define CST816T_Register_IrqCtl 				0xFA	
#define CST816T_Register_AutoReset 			0xFB	
#define CST816T_Register_LongPressTime 	0xFC	
#define CST816T_Register_IOCtl 					0xFD	
#define CST816T_Register_DisAutoSleep 	0xFE	


void CST816T_Init(void);								
uint8_t CST816T_GetGestureID(void);		
uint16_t CST816T_GetXpos(void);				
uint16_t CST816T_GetYpos(void);				
uint8_t CST816T_GetChipID(void);			
uint8_t CST816T_GetRegisterData(uint8_t reg);	
uint8_t CST816T_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t CST816T_ReadReg(uint8_t RegAddress);
uint8_t CST816T_ID(void);
uint8_t CST816T_GetGestureID(void);
uint8_t CST816T_Pressed(void);
void enter_cst816t_sleep(void);
void enter_cst816t_reset(void);



#endif
