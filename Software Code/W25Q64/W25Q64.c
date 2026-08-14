#include "stm32f4xx.h"                  // Device header
#include "MySPI.h"
#include "W25Q64_Ins.h"
#include "spi.h"
#include "MySPI.h"

/**
  * 函    数：W25Q64初始化
  * 参    数：无
  * 返 回 值：无
  */
void W25Q64_Init(void)
{
  MX_SPI2_Init();
	MySPI_Init();					//先初始化底层的SPI
	MySPI_W_SS(0);
  MySPI_SwapByte(0XAB);
  MySPI_W_SS(1);
}

/**
  * 函    数：MPU6050读取ID号
  * 参    数：MID 工厂ID，使用输出参数的形式返回
  * 参    数：DID 设备ID，使用输出参数的形式返回
  * 返 回 值：无
  */
void W25Q64_ReadID(uint8_t *MID, uint16_t *DID)
{
	MySPI_Start();								//SPI起始
	MySPI_SwapByte(W25Q64_JEDEC_ID);			//交换发送读取ID的指令
	*MID = MySPI_SwapByte(W25Q64_DUMMY_BYTE);	//交换接收MID，通过输出参数返回
	*DID = MySPI_SwapByte(W25Q64_DUMMY_BYTE);	//交换接收DID高8位
	*DID <<= 8;									//高8位移到高位
	*DID |= MySPI_SwapByte(W25Q64_DUMMY_BYTE);	//或上交换接收DID的低8位，通过输出参数返回
	MySPI_Stop();								//SPI终止
}
//void W25Q64_ReadID(uint8_t *MID, uint16_t *DID)
//{
//    uint8_t high_byte, low_byte;
//    
//    MySPI_Start();
//    MySPI_SwapByte(W25Q64_JEDEC_ID);        // 0x9F 发送读JEDEC ID命令
//    *MID = MySPI_SwapByte(W25Q64_DUMMY_BYTE);   // 读取制造商ID (0xEF)
//    high_byte = MySPI_SwapByte(W25Q64_DUMMY_BYTE); // 读取内存类型高8位 (0x40)
//    low_byte = MySPI_SwapByte(W25Q64_DUMMY_BYTE);  // 读取容量低8位 (0x17)
//    MySPI_Stop();
//    
//    *DID = (high_byte << 8) | low_byte;     // 组合成16位设备ID
//}
/**
  * 函    数：W25Q64写使能
  * 参    数：无
  * 返 回 值：无
  */
void W25Q64_WriteEnable(void)
{
	MySPI_Start();								//SPI起始
	MySPI_SwapByte(W25Q64_WRITE_ENABLE);		//交换发送写使能的指令
	MySPI_Stop();								//SPI终止
}

/**
  * 函    数：W25Q64等待忙
  * 参    数：无
  * 返 回 值：无
  */
void W25Q64_WaitBusy(void)
{
	uint32_t Timeout;
	MySPI_Start();								//SPI起始
	MySPI_SwapByte(W25Q64_READ_STATUS_REGISTER_1);				//交换发送读状态寄存器1的指令
	Timeout = 100000;							//给定超时计数时间
	while ((MySPI_SwapByte(W25Q64_DUMMY_BYTE) & 0x01) == 0x01)	//循环等待忙标志位
	{
		Timeout --;								//等待时，计数值自减
		if (Timeout == 0)						//自减到0后，等待超时
		{
			/*超时的错误处理代码，可以添加到此处*/
			break;								//跳出等待，不等了
		}
	}
	MySPI_Stop();								//SPI终止
}

/**
  * 函    数：W25Q64页编程
  * 参    数：Address 页编程的起始地址，范围：0x000000~0x7FFFFF
  * 参    数：DataArray	用于写入数据的数组
  * 参    数：Count 要写入数据的数量，范围：0~256
  * 返 回 值：无
  * 注意事项：写入的地址范围不能跨页
  */
void W25Q64_PageProgram(uint32_t Address, uint8_t *DataArray, uint16_t Count)
{
	uint16_t i;
	
	W25Q64_WriteEnable();						//写使能
	
	MySPI_Start();								//SPI起始
	MySPI_SwapByte(W25Q64_PAGE_PROGRAM);		//交换发送页编程的指令
	MySPI_SwapByte(Address >> 16);				//交换发送地址23~16位
	MySPI_SwapByte(Address >> 8);				//交换发送地址15~8位
	MySPI_SwapByte(Address);					//交换发送地址7~0位
	for (i = 0; i < Count; i ++)				//循环Count次
	{
		MySPI_SwapByte(DataArray[i]);			//依次在起始地址后写入数据
	}
	MySPI_Stop();								//SPI终止
	
	W25Q64_WaitBusy();							//等待忙
}

//全片擦除
void W25Q64_Erase_Chip(void)   
{                                   
  W25Q64_WriteEnable();                  //SET WEL 
	W25Q64_WaitBusy();							//等待忙
	MySPI_Start();								//SPI起始
  MySPI_SwapByte(W25Q64_CHIP_ERASE);        //发送片擦除命令  
	MySPI_Stop();								//SPI终止
	W25Q64_WaitBusy();							//等待忙
}  

/**
  * 函    数：W25Q64扇区擦除（4KB）
  * 参    数：Address 指定扇区的地址，范围：0x000000~0x7FFFFF
  * 返 回 值：无
  */
void W25Q64_SectorErase(uint32_t Address)
{
	W25Q64_WriteEnable();						//写使能
	
	MySPI_Start();								//SPI起始
	MySPI_SwapByte(W25Q64_SECTOR_ERASE_4KB);	//交换发送扇区擦除的指令
	MySPI_SwapByte(Address >> 16);				//交换发送地址23~16位
	MySPI_SwapByte(Address >> 8);				//交换发送地址15~8位
	MySPI_SwapByte(Address);					//交换发送地址7~0位
	MySPI_Stop();								//SPI终止
	
	W25Q64_WaitBusy();							//等待忙
}

/**
  * 函    数：W25Q64读取数据
  * 参    数：Address 读取数据的起始地址，范围：0x000000~0x7FFFFF
  * 参    数：DataArray 用于接收读取数据的数组，通过输出参数返回
  * 参    数：Count 要读取数据的数量，范围：0~0x800000
  * 返 回 值：无
  */
//void W25Q64_ReadData(uint8_t *DataArray,  uint32_t Address,  uint32_t Count)
//{
//	uint32_t i;
//	MySPI_Start();								//SPI起始
//	MySPI_SwapByte(W25Q64_READ_DATA);			//交换发送读取数据的指令
//	MySPI_SwapByte((uint8_t)(Address >> 16));				//交换发送地址23~16位
//	MySPI_SwapByte((uint8_t)(Address >> 8));				//交换发送地址15~8位
//	MySPI_SwapByte((uint8_t)Address);					//交换发送地址7~0位
//	for (i = 0; i < Count; i ++)				//循环Count次
//	{
//		DataArray[i] = MySPI_SwapByte(W25Q64_DUMMY_BYTE);	//依次在起始地址后读取数据
//	}
//	MySPI_Stop();								//SPI终止
//}

volatile uint8_t spiTransferComplete = 0;
void W25Q64_ReadData(uint8_t *DataArray, uint32_t Address, uint32_t Count)
{
//		uint8_t cmd[4];

//		// 构造命令：READ + 24位地址
//		cmd[0] = W25Q64_READ_DATA;
//		cmd[1] = (uint8_t)(Address >> 16);
//		cmd[2] = (uint8_t)(Address >> 8);
//		cmd[3] = (uint8_t)Address;
//		MySPI_Start();
//		HAL_SPI_Transmit(&hspi2, cmd, 4,2);
//		HAL_SPI_Receive(&hspi2, DataArray, Count,3);
//		MySPI_Stop(); 
	
	uint8_t cmd[4];

		// 构造命令：READ + 24位地址
		cmd[0] = W25Q64_READ_DATA;
		cmd[1] = (uint8_t)(Address >> 16);
		cmd[2] = (uint8_t)(Address >> 8);
		cmd[3] = (uint8_t)Address;
		MySPI_Start();
		HAL_SPI_Transmit(&hspi2, cmd, 4,5);
		spiTransferComplete=0;
		HAL_SPI_Receive_DMA(&hspi2, DataArray, Count);
		while(spiTransferComplete==0);
		MySPI_Stop();
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi == &hspi2)
    {
			spiTransferComplete=1;
    }
}


void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint32_t NumByteToWrite)   
{ 			 		 
	uint32_t pageremain;	   
	pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
	while(1)
	{	   
		W25Q64_PageProgram(WriteAddr,pBuffer,pageremain);
		if(NumByteToWrite==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
			else pageremain=NumByteToWrite; 	  //不够256个字节了
		}
	};	    
} 

uint8_t W25QXX_BUFFER[4096];		 
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint32_t NumByteToWrite)   
{ 
	uint32_t secpos;
	uint32_t secoff;
	uint32_t secremain;	   
 	uint32_t i;    
	uint8_t * W25QXX_BUF;	  
   W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;//扇区地址  
	secoff=WriteAddr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小   
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//不大于4096个字节
	while(1) 
	{	
		W25Q64_ReadData(W25QXX_BUF,secpos*4096,4096);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;//需要擦除  	  
		}
		if(i<secremain)//需要擦除
		{
			W25Q64_SectorErase(secpos);//擦除这个扇区
			for(i=0;i<secremain;i++)	   //复制
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//写入整个扇区  
 
		}else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		if(NumByteToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;//扇区地址增1
			secoff=0;//偏移位置为0 	 
 
		   	pBuffer+=secremain;  //指针偏移
			WriteAddr+=secremain;//写地址偏移	   
		   	NumByteToWrite-=secremain;				//字节数递减
			if(NumByteToWrite>4096)secremain=4096;	//下一个扇区还是写不完
			else secremain=NumByteToWrite;			//下一个扇区可以写完了
		}	 
	};	 
}


void W25Q64_EraseByAddressExact(uint32_t StartAddr, uint32_t DataSize)
{
    // 关键修正：确保起始地址是扇区对齐的
    if (StartAddr % 4096 != 0) {
        return;
    }
    
    uint32_t StartSector = StartAddr / 4096;
    // 修正：只擦除到数据结束的扇区，不额外多擦除
    uint32_t EndSector = (StartAddr + DataSize) / 4096;
    
    // 如果数据正好在扇区边界结束，不需要擦除下一个扇区
    if ((StartAddr + DataSize) % 4096 == 0) {
        EndSector = (StartAddr + DataSize) / 4096 - 1;
    }
    
    uint32_t SectorCount = EndSector - StartSector + 1;
    
    for (uint32_t i = 0; i < SectorCount; i++) {
        uint32_t CurrentSectorAddr = (StartSector + i) * 4096;
        W25Q64_SectorErase(CurrentSectorAddr);
    }
}