#include "stm32f4xx.h"                  // Device header
#include "spi.h"


void MySPI_W_SS(uint8_t PinState)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (GPIO_PinState)PinState);		//根据BitValue，设置SS引脚的电平
}

void MySPI_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/*GPIO初始化*/
	 GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
	
	/*设置默认电平*/
	MySPI_W_SS(1);											//SS默认高电平
}

/*协议层*/

/**
  * 函    数：SPI起始
  * 参    数：无
  * 返 回 值：无
  */
void MySPI_Start(void)
{
	MySPI_W_SS(0);				//拉低SS，开始时序
}

/**
  * 函    数：SPI终止
  * 参    数：无
  * 返 回 值：无
  */
void MySPI_Stop(void)
{
	MySPI_W_SS(1);				//拉高SS，终止时序
}

/**
  * 函    数：SPI交换传输一个字节，使用SPI模式0
  * 参    数：ByteSend 要发送的一个字节
  * 返 回 值：接收的一个字节
  */
uint8_t MySPI_SwapByte(uint8_t ByteSend)
{
	uint8_t ByteReceive = 0x00;					//定义接收的数据，并赋初值0x00，此处必须赋初值0x00，后面会用到
	
	HAL_SPI_TransmitReceive(&hspi2,&ByteSend,&ByteReceive,1,5);
//	for (i = 0; i < 8; i ++)						//循环8次，依次交换每一位数据
//	{
//		MySPI_W_MOSI(ByteSend & (0x80 >> i));		//使用掩码的方式取出ByteSend的指定一位数据并写入到MOSI线
//		MySPI_W_SCK(1);								//拉高SCK，上升沿移出数据
//		if (MySPI_R_MISO() == 1){ByteReceive |= (0x80 >> i);}	//读取MISO数据，并存储到Byte变量
//																//当MISO为1时，置变量指定位为1，当MISO为0时，不做处理，指定位为默认的初值0
//		MySPI_W_SCK(0);								//拉低SCK，下降沿移入数据
//	}
	
	return ByteReceive;								//返回接收到的一个字节数据
}
//uint8_t MySPI_SwapByte(uint8_t ByteSend)
//{
//	uint8_t ByteReceive = 0x00;					//定义接收的数据，并赋初值0x00，此处必须赋初值0x00，后面会用到

//	HAL_SPI_TransmitReceive_DMA(&hspi2,&ByteSend,&ByteReceive,1);
////	for (i = 0; i < 8; i ++)						//循环8次，依次交换每一位数据
////	{
////		MySPI_W_MOSI(ByteSend & (0x80 >> i));		//使用掩码的方式取出ByteSend的指定一位数据并写入到MOSI线
////		MySPI_W_SCK(1);								//拉高SCK，上升沿移出数据
////		if (MySPI_R_MISO() == 1){ByteReceive |= (0x80 >> i);}	//读取MISO数据，并存储到Byte变量
////																//当MISO为1时，置变量指定位为1，当MISO为0时，不做处理，指定位为默认的初值0
////		MySPI_W_SCK(0);								//拉低SCK，下降沿移入数据
////	}
//	
//	return ByteReceive;								//返回接收到的一个字节数据
//}


