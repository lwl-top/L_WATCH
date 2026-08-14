 #include "lcd_init.h"
#include "delay.h"
#include "spi.h"
#include "tim.h"

void LCD_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */

  /*Configure GPIO pins : PA2 PA3 PA5 PA6
                           PA7 */
//  GPIO_InitStruct.Pin = GPIO_PIN_11;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void LCD_Writ_Bus(u16 dat) 
{	
	HAL_SPI_Transmit(&hspi1, (uint8_t*)&dat, 1,100);
}

void LCD_WR_DATA8(u16 dat)
{
	LCD_Writ_Bus(dat);
}

void LCD_WR_DATA(u16 dat)
{
	 LCD_DC_Set(); 
	uint8_t temp[2];
	temp[0]=(dat>>8)&0xff;
	temp[1]=dat&0xff;
	HAL_SPI_Transmit(&hspi1,temp,2,100);
}


void LCD_WR_REG(u16 dat)
{
	LCD_DC_Clr();
	LCD_Writ_Bus(dat);
	LCD_DC_Set();
}


#define LCD_WIDTH 240
#define LCD_HEIGHT 280

void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2)
{
    // 根据屏幕方向进行不同的坐标映射
    switch(USE_HORIZONTAL)
    {
        case 0:  // 竖屏 - 正常方向
            LCD_WR_REG(0x2a);  // 列地址设置
            LCD_WR_DATA(x1);
            LCD_WR_DATA(x2);
            LCD_WR_REG(0x2b);  // 行地址设置
            LCD_WR_DATA(y1);
            LCD_WR_DATA(y2);
            break;
            
        case 1:  // 横屏 - 旋转90度
            LCD_WR_REG(0x2a);  // 列地址设置 (使用Y坐标)
            LCD_WR_DATA(y1);
            LCD_WR_DATA(y2);
            LCD_WR_REG(0x2b);  // 行地址设置 (使用X坐标)
            LCD_WR_DATA(x1);
            LCD_WR_DATA(x2);
            break;
            
        case 2:  // 竖屏 - 旋转180度
            LCD_WR_REG(0x2a);  // 列地址设置 (反向)
            LCD_WR_DATA(LCD_WIDTH - 1 - x2);
            LCD_WR_DATA(LCD_WIDTH - 1 - x1);
            LCD_WR_REG(0x2b);  // 行地址设置 (反向)
            LCD_WR_DATA(LCD_HEIGHT - 1 - y2);
            LCD_WR_DATA(LCD_HEIGHT - 1 - y1);
            break;
            
        case 3:  // 横屏 - 旋转270度
            LCD_WR_REG(0x2a);  // 列地址设置 (使用反向Y坐标)
            LCD_WR_DATA(LCD_HEIGHT - 1 - y2);
            LCD_WR_DATA(LCD_HEIGHT - 1 - y1);
            LCD_WR_REG(0x2b);  // 行地址设置 (使用X坐标)
            LCD_WR_DATA(x1);
            LCD_WR_DATA(x2);
            break;
            
        default:  // 默认按竖屏处理
            LCD_WR_REG(0x2a);
            LCD_WR_DATA(x1);
            LCD_WR_DATA(x2);
            LCD_WR_REG(0x2b);
            LCD_WR_DATA(y1);
            LCD_WR_DATA(y2);
            break;
    }
    
    // 写内存命令
    LCD_WR_REG(0x2c);
}

void LCD_SetCursor(u16 Xpos, u16 Ypos)
{
	if(USE_HORIZONTAL==0)
	{
    LCD_WR_REG(0x2a);
    LCD_WR_DATA(Xpos);
    LCD_WR_DATA(240-1);
		LCD_WR_REG(0x2b);
		LCD_WR_DATA(Ypos);
    LCD_WR_DATA(285-1);
		LCD_WR_REG(0x2c);
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);
    LCD_WR_DATA(Xpos);
    LCD_WR_DATA(240-1);
		LCD_WR_REG(0x2b);
		LCD_WR_DATA(Ypos);
    LCD_WR_DATA(285-1);
		LCD_WR_REG(0x2c);
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);
		LCD_WR_DATA(Xpos);
		LCD_WR_DATA(285-1);
		LCD_WR_REG(0x2b);
		LCD_WR_DATA(Ypos);
		LCD_WR_DATA(240-1);
		LCD_WR_REG(0x2c);
	}

}




void LCD_Init(void)
{
	MX_SPI1_Init();
	LCD_GPIO_Init();//
//	
	LCD_RES_Clr();//
	HAL_Delay(100);
	LCD_RES_Set();
	HAL_Delay(100);
//	
	
//	LCD_BLK_Set();//
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 49);
   HAL_Delay(100);
	
	  LCD_CS_Clr();

    LCD_WR_REG(0x11);        // Sleep out 退出休眠
    HAL_Delay(120);          // 等待120ms
    
    LCD_WR_REG(0x36);        // MADCTL 显示方向配置
    LCD_WR_DATA8(0x00);      // 修正：用十六进制，根据屏幕硬件调整（比如0xC0是旋转180度）
    
    LCD_WR_REG(0x3A);        // COLMOD 像素格式
    LCD_WR_DATA8(0x55);      // 0x06=RGB666，0x05=RGB565（根据需求改）
    
    LCD_WR_REG(0xB2);        
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x33);

    LCD_WR_REG(0xB7);       
    LCD_WR_DATA8(0x3F);

    LCD_WR_REG(0xBB);        
    LCD_WR_DATA8(0x18);

    LCD_WR_REG(0xC0);       
    LCD_WR_DATA8(0x2C);

    LCD_WR_REG(0xC2);       
    LCD_WR_DATA8(0x01);

    LCD_WR_REG(0xC3);       
    LCD_WR_DATA8(0x10);//
		
		 LCD_WR_REG(0xC4);       
    LCD_WR_DATA8(0x26);//

    LCD_WR_REG(0xC6);        
    LCD_WR_DATA8(0x0F);     

    LCD_WR_REG(0xD0);       
    LCD_WR_DATA8(0xA9);  
		
//    LCD_WR_REG(0xD0);       
//    LCD_WR_DATA8(0xA6);  
//		LCD_WR_DATA8(0xA2);    
    // 移除重复的0xD0寄存器写入
    
    LCD_WR_REG(0xD6);        
    LCD_WR_DATA8(0xA1);
//原始伽马
    LCD_WR_REG(0xE0);        // Positive Gamma 正伽马校正
    LCD_WR_DATA8(0xF0);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x10);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x19);
    LCD_WR_DATA8(0x3B);
    LCD_WR_DATA8(0x56);
    LCD_WR_DATA8(0x4E);
    LCD_WR_DATA8(0x37);
    LCD_WR_DATA8(0x12);
    LCD_WR_DATA8(0x12);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0x37);

    LCD_WR_REG(0xE1);        // Negative Gamma 负伽马校正
    LCD_WR_DATA8(0xF0);
    LCD_WR_DATA8(0x12);
    LCD_WR_DATA8(0x16);
    LCD_WR_DATA8(0x0F);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x3B);
    LCD_WR_DATA8(0x46);
    LCD_WR_DATA8(0x4D);      // 伽马参数，注释修正
    LCD_WR_DATA8(0x38);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x2E);
    LCD_WR_DATA8(0x35);

// LCD_WR_REG(0xE0); // 正Gamma
//    LCD_WR_DATA8(0xD0);
//    LCD_WR_DATA8(0x00);
//    LCD_WR_DATA8(0x03);
//    LCD_WR_DATA8(0x0E);
//    LCD_WR_DATA8(0x13);
//    LCD_WR_DATA8(0x1C);
//    LCD_WR_DATA8(0x37);
//    LCD_WR_DATA8(0x54);
//    LCD_WR_DATA8(0x4F);
//    LCD_WR_DATA8(0x44);
//    LCD_WR_DATA8(0x3A);
//    LCD_WR_DATA8(0x31);
//    LCD_WR_DATA8(0x29);
//    LCD_WR_DATA8(0x23);

//    LCD_WR_REG(0xE1); // 负Gamma
//    LCD_WR_DATA8(0xD0);
//    LCD_WR_DATA8(0x00);
//    LCD_WR_DATA8(0x03);
//    LCD_WR_DATA8(0x0E);
//    LCD_WR_DATA8(0x13);
//    LCD_WR_DATA8(0x1C);
//    LCD_WR_DATA8(0x37);
//    LCD_WR_DATA8(0x54);
//    LCD_WR_DATA8(0x4F);
//    LCD_WR_DATA8(0x44);
//    LCD_WR_DATA8(0x3A);
//    LCD_WR_DATA8(0x31);
//    LCD_WR_DATA8(0x29);
//    LCD_WR_DATA8(0x23);

    LCD_WR_REG(0xE4);        // Power Control 4 电源控制4
    LCD_WR_DATA8(0x23);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0x21);        // Display Inversion On 显示反转开启
    LCD_WR_REG(0x29);        // Display On 开启显示
//    LCD_WR_REG(0x2C);        // Memory Write 进入显存写入模式

HAL_Delay(10);
} 




