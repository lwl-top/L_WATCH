#ifndef _pcf8563_h_ 
#define _pcf8563_h_
 
#include "MYI2C.h"

 typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t weekday;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} PCF8563_Time;
extern unsigned char Timelist[]; // 年月日时分秒周
void PCF8563_WriteTime(unsigned char TimeAdd, unsigned char TimeData);
unsigned char PCF8563_ReadTime(unsigned char Add);
void Set_PCF8563time(void);
void Read_PCF8563time(void);
void PCF8563_Init(void);

 
#endif  /* end pcf8563.h */