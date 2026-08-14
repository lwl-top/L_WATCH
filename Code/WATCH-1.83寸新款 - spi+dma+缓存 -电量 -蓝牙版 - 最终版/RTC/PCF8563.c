#include "stm32f4xx.h"                  // Device header
#include "MYI2C.h"                  // Device header
#include "PCF8563.h"                  // Device header

 unsigned char buffer1[4]; 
 unsigned char buffer[4];    
 
#define Read_Add  0xA3
#define Write_Add  0xA2


#define Year_Add  0x08
#define Mother_Add  0x07
#define Week_Add  0x06
#define Day_Add  0x05
#define Hour_Add  0x04
#define Min_Add  0x03
#define Sec_Add  0x02

unsigned char Timelist[] = {25, 5, 14,00,28,3, 2}; // 年月日时分秒周


#include <stdint.h>

// 寄存器地址定义
#define PCF8563_ADDR       0x51
#define SEC_REG            0x02
#define MIN_REG            0x03
#define HOUR_REG          0x04
#define DAY_REG           0x05
#define WEEKDAY_REG       0x06
#define MONTH_REG         0x07
#define YEAR_REG          0x08
#define CONTROL1_REG      0x00

// I2C接口函数声明（需根据实际平台实现）
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_WriteByte(uint8_t data);
uint8_t I2C_ReadByte(uint8_t ack);

// 时间结构体

// 初始化PCF8563
void PCF8563_Init(void) {
    MyI2C_Init();
    
}

// BCD转十进制
static uint8_t bcd2dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// 十进制转BCD
static uint8_t dec2bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}



// 示例用法
//int main(void) {
//    PCF8563_Time current_time = {
//        .year = 25,
//        .month = 5,
//        .day = 13,
//        .weekday = 2,  // 周二
//        .hour = 16,
//        .minute = 35,
//        .second = 0
//    };
//    
//    PCF8563_Init();
//    PCF8563_SetTime(&current_time);
//    
//    while(1) {
//        PCF8563_GetTime(&current_time);
//        // 在此处理时间数据...
//    }
//    
//    return 0;
//}


void PCF8563_WriteTime(unsigned char TimeAdd, unsigned char TimeData)
{
	TimeData = (TimeData / 10 * 16) + (TimeData % 10);
	MyI2C_Start();
	MyI2C_SendByte(Write_Add);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(TimeAdd);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(TimeData);
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}

unsigned char PCF8563_ReadTime(unsigned char Add)
{
	unsigned char TimeData;
	MyI2C_Start();
	MyI2C_SendByte(Write_Add);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(Add);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();
	MyI2C_SendByte(Read_Add);
	MyI2C_ReceiveAck();

	TimeData = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	MyI2C_Stop();
	return TimeData;
}

void Set_PCF8563time(void)
{
	PCF8563_WriteTime(Year_Add, Timelist[0]);
	PCF8563_WriteTime(Mother_Add, Timelist[1]);
	PCF8563_WriteTime(Day_Add, Timelist[2]);
	PCF8563_WriteTime(Hour_Add, Timelist[3]);
	PCF8563_WriteTime(Min_Add, Timelist[4]);
	PCF8563_WriteTime(Sec_Add, Timelist[5]);
	PCF8563_WriteTime(Week_Add, Timelist[6]);
}

void Read_PCF8563time(void)
{

	Timelist[5] = ((PCF8563_ReadTime(Sec_Add)&0x7f)/16*10)+((PCF8563_ReadTime(Sec_Add)&0x7f)%16);
	Timelist[4] =  ((PCF8563_ReadTime(Min_Add)&0x7f)/16*10)+((PCF8563_ReadTime(Min_Add)&0x7f)%16);
	Timelist[3] = ((PCF8563_ReadTime(Hour_Add)&0x3f)/16*10)+((PCF8563_ReadTime(Hour_Add)&0x3f)%16);
	Timelist[2] = ((PCF8563_ReadTime(Day_Add)&0x3f)/16*10)+((PCF8563_ReadTime(Day_Add)&0x3f)%16);
	Timelist[6] = ((PCF8563_ReadTime(Week_Add)&0x07)/16*10)+((PCF8563_ReadTime(Week_Add)&0x07)%16)+1;
  Timelist[1] = ((PCF8563_ReadTime(Mother_Add)&0x1f)/16*10)+((PCF8563_ReadTime(Mother_Add)&0x1f)%16);
	Timelist[0] = ((PCF8563_ReadTime(Year_Add)&0xff)/16*10)+((PCF8563_ReadTime(Year_Add)&0xff)%16);
}