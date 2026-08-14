#include "stm32f4xx.h"                  
#include "MPU6050_I2C.h"                  
#include "HMC.h"                  

void HMC_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MPU6050_Start();						
	MPU6050_SendByte(HMC_ADDRESS);
	MPU6050_ReceiveAck();				
	MPU6050_SendByte(RegAddress);		
	MPU6050_ReceiveAck();					
	MPU6050_SendByte(Data);				
	MPU6050_ReceiveAck();					
	MPU6050_Stop();					
}


uint8_t HMC_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MPU6050_Start();						
	MPU6050_SendByte(HMC_ADDRESS);	
	MPU6050_ReceiveAck();					
	MPU6050_SendByte(RegAddress);			
	MPU6050_ReceiveAck();					
	
	MPU6050_Start();					
	MPU6050_SendByte(HMC_ADDRESS | 0x01);	
	MPU6050_ReceiveAck();				
	Data = MPU6050_ReceiveByte();			
	MPU6050_SendAck(1);					
	MPU6050_Stop();					
	
	return Data;
}

void HMC_ReadRegs(uint8_t reg, uint8_t* buf, uint8_t len)
{
    MPU6050_Start();
    MPU6050_SendByte(HMC_ADDRESS);    
    MPU6050_ReceiveAck();
    MPU6050_SendByte(reg);                
    MPU6050_ReceiveAck();

    MPU6050_Start();                      
    MPU6050_SendByte(HMC_ADDRESS | 0x01);  
    MPU6050_ReceiveAck();

    for(uint8_t i = 0; i < len; i++)
    {
        buf[i] = MPU6050_ReceiveByte();
        if(i == len - 1)
            MPU6050_SendAck(1);            
        else
            MPU6050_SendAck(0);           
    }
    MPU6050_Stop();
}
void HMC_sleep(void)
{
	HMC_WriteReg(0x02, 0x03);
}
void HMC_wake(void)
{
	HMC_WriteReg(0x02, 0x00);
}
	
void HMC_READ_DATA(int16_t *data)
{
	 uint8_t buffer[6];
	HMC_ReadRegs(0x03,buffer,6);
	data[0] = (int16_t)((buffer[0] << 8) | buffer[1]); 
  data[2] = (int16_t)((buffer[2] << 8) | buffer[3]); 
  data[1] = (int16_t)((buffer[4] << 8) | buffer[5]); 
}

void HMC_Init(void)
{
	HMC_WriteReg(0x00,0x70);
	HMC_WriteReg(0x01,0x40); 
	HMC_WriteReg(0x02,0x00);
}