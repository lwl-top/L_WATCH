#ifndef __W25Q64_H
#define __W25Q64_H

void W25Q64_Init(void);
void W25Q64_ReadID(uint8_t *MID, uint16_t *DID);
void W25Q64_PageProgram(uint32_t Address, uint8_t *DataArray, uint16_t Count);
void W25Q64_SectorErase(uint32_t Address);
void W25Q64_ReadData(uint8_t *DataArray,uint32_t Address, uint32_t Count);
void W25Q64_Erase_Chip(void);
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint32_t NumByteToWrite);  
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint32_t NumByteToWrite);   
void W25Q64_EraseByAddressExact(uint32_t StartAddr, uint32_t DataSize);


#endif
