#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "sys.h"

#define USE_HORIZONTAL 0


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 300

#else
#define LCD_W 284
#define LCD_H 240
#endif

#define LCD_RES_Clr()  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET)
 		     
#define LCD_CS_Clr()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET)//CS
#define LCD_CS_Set()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET)

#define LCD_BLK_Clr()  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)//BLK
#define LCD_BLK_Set()  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)
#define LCD_RAM_SIZE   240*284*2


 

void LCD_GPIO_Init(void);
void LCD_Writ_Bus(u16 dat);
void LCD_WR_DATA8(u16 dat);
void LCD_WR_DATA(u16 dat);
void LCD_WR_REG(u16 dat);
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);
void LCD_SetCursor(u16 Xpos, u16 Ypos);

void LCD_Init(void);
#endif




