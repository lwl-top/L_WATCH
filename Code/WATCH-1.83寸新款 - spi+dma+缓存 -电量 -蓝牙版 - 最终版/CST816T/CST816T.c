#include "main.h"
#include "CST816T.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"

void enter_cst816t_sleep(void)
{
	CST816T_WriteReg(CST816T_Register_SleepMode,0x03);
}
void enter_cst816t_reset(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	vTaskDelay(pdMS_TO_TICKS(10)); 
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(100)); 
}
void CST816T_Init(void)
{
	  MX_I2C1_Init();
	enter_cst816t_reset();
}	

uint8_t CST816T_WriteReg(uint8_t RegAddress, uint8_t Data)	
{
 uint8_t  *pData;
 pData = &Data;
HAL_I2C_Mem_Write(&hi2c1,CST816T_I2CAddress_8bit_Write, RegAddress,I2C_MEMADD_SIZE_8BIT,pData,1,100);

}
uint8_t CST816T_ReadReg(uint8_t RegAddress)
{
	 uint8_t buf[1];
	 HAL_I2C_Mem_Read(&hi2c1, CST816T_I2CAddress_8bit_Write, RegAddress, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);
    return buf[0];
}
uint8_t CST816T_Pressed(void)
{
	return CST816T_ReadReg(CST816T_Register_FingerNum);
}

uint8_t CST816T_ID(void)
{
	return CST816T_ReadReg(CST816T_Register_ChipID);
}
uint8_t CST816T_GetGestureID(void)
{
		return CST816T_ReadReg(CST816T_Register_GestureID);
}
uint16_t CST816T_GetXpos(void)
{
	uint8_t reData[2] = {0};	
	HAL_I2C_Mem_Read(&hi2c1, CST816T_I2CAddress_8bit_Write, CST816T_Register_XposH, I2C_MEMADD_SIZE_8BIT, reData, 2,100);					
	return ((reData[0] & 0x0F) << 8) | reData[1];
}
uint16_t CST816T_GetYpos(void)
{
	uint8_t reData[2] = {0};
	HAL_I2C_Mem_Read(&hi2c1, CST816T_I2CAddress_8bit_Write, CST816T_Register_YposH, I2C_MEMADD_SIZE_8BIT, reData, 2,100);							
	return (((reData[0] & 0x0F) << 8) | reData[1]);
}
//void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//  if (hi2c == &hi2c1) {
//    X=   CST816T_GetXpos();
//		  Y=   CST816T_GetYpos();

//  }
//}
uint8_t TP_Read_XY(uint16_t *x,uint16_t *y)
{
	uint16_t xtemp,ytemp;			 	 		  
	xtemp=CST816T_GetXpos();
	ytemp=CST816T_GetYpos();	  												   
	//if(xtemp<100||ytemp<100)return 0;
	*x=xtemp;
	*y=ytemp;
	return 1;
}
#define ERR_RANGE 6 
uint8_t TP_Read_XY2(uint16_t *x,uint16_t *y) 
{
	uint16_t x1,y1;
 	uint16_t x2,y2;
 	uint8_t flag;    
    flag=TP_Read_XY(&x1,&y1);   
    if(flag==0)return(0);
    flag=TP_Read_XY(&x2,&y2);	   
    if(flag==0)return(0);   
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))
    &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
        *x=(x1+x2)/2;
        *y=(y1+y2)/2;
        return 1;
    }else return 0;	  
}  
//uint8_t CST816T_GetGestureID(void)
//{
//	uint8_t reData[1] = {0};													
//	uint8_t CMD[] = { CST816T_Register_GestureID };	
//	HAL_I2C_Master_Transmit(&hi2c1, CST816T_I2CAddress_8bit_Write, CMD , 1, 1000);
//	HAL_I2C_Master_Receive(&hi2c1, CST816T_I2CAddress_8bit_Read, reData , 1, 1000);
//	return reData[0];
//}

//uint16_t CST816T_GetXpos(void)
//{
//	uint8_t reData[2] = {0};											
//	uint8_t CMD[] = { CST816T_Register_XposH };	
//	HAL_I2C_Master_Transmit(&hi2c1, CST816T_I2CAddress_8bit_Write, CMD , 1, 1000);
//	HAL_I2C_Master_Receive(&hi2c1, CST816T_I2CAddress_8bit_Read, reData , 2, 1000);
//	return ((reData[0] & 0x0F) << 8) | reData[1];
//}

//uint16_t CST816T_GetYpos(void)
//{
//	uint8_t reData[2] = {0};											
//	uint8_t CMD[] = { CST816T_Register_YposH };	
//	HAL_I2C_Master_Transmit(&hi2c1, CST816T_I2CAddress_8bit_Write, CMD , 1, 1000);
//	HAL_I2C_Master_Receive(&hi2c1, CST816T_I2CAddress_8bit_Read, reData , 2, 1000);
//	return ((reData[0] & 0x0F) << 8) | reData[1];
//}

//uint8_t CST816T_GetChipID(void)
//{
//	uint8_t reData[1] = {0};											
//	uint8_t CMD[] = { CST816T_Register_ChipID };
//	HAL_I2C_Master_Transmit(&hi2c1, CST816T_I2CAddress_8bit_Write, CMD , 1, 1000);
//	HAL_I2C_Master_Receive(&hi2c1, CST816T_I2CAddress_8bit_Read, reData , 1, 1000);
//	return reData[0];
//}

//uint8_t CST816T_GetRegisterData(uint8_t reg)
//{
//	uint8_t reData[1] = {0};	
//	uint8_t CMD[] = { reg };	
//	HAL_I2C_Master_Transmit(&hi2c1, CST816T_I2CAddress_8bit_Write, CMD , 1, 1000);
//	HAL_I2C_Master_Receive(&hi2c1, CST816T_I2CAddress_8bit_Read, reData , 1, 1000);
//	return reData[0];
//}
