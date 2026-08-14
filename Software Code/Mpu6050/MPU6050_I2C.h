#ifndef __MPU6050_I2C_H
#define __MPU6050_I2C_H


void MPU6050_GPIO_Init(void);
void MPU6050_Start(void);
void MPU6050_Stop(void);
void MPU6050_SendByte(uint8_t Byte);
uint8_t MPU6050_ReceiveByte(void);
void MPU6050_SendAck(uint8_t AckBit);
uint8_t MPU6050_ReceiveAck(void);


#endif
