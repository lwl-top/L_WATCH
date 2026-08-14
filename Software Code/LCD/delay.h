#ifndef _DELAY_H
#define _DELAY_H
#include "stm32f4xx.h"                  // Device header

#include "sys.h"	  


void Delay_init(u8 SYSCLK);
void Delay_ms(u16 nms);
void Delay_us(u32 nus);
#endif

