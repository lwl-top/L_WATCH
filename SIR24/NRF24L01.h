#ifndef __nRF24L01_API_H
#define __nRF24L01_API_H

#include "stm32f10x.h"                  // Device header

uint8_t SPI_SwapByte(uint8_t byte);
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value);
uint8_t NRF24L01_Read_Reg(uint8_t reg);
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_GetRxBuf(uint8_t *rxbuf);
uint8_t NRF24L01_SendTxBuf(uint8_t *txbuf);
uint8_t NRF24L01_Check(void);
void NRF24L01_RT_Init(void);
void NRF24L01_Init(void);
void NRF24L01_SendBuf(uint8_t *Buf);
void NRF24L01_Pin_Init(void);
uint8_t NRF24L01_Get_Value_Flag();

#endif
