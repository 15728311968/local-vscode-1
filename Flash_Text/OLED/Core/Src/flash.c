#include "flash.h"

/**
 *@功能：从内部Flash读取指定字节数据
 *@参数1：ReadAddress：数据起始地址
 *@参数2：*data：      读取到的数据缓存首地址
 *@参数3：length：     读取字节个数
 */
void ReadFlashData(uint32_t ReadAddress, uint8_t *data, uint32_t length)
{
    for(uint32_t i=0;i<length;i++)
    {
        data[i]=*(uint8_t*)(ReadAddress+i); //读取数据
    }
}
#define FMC_FLASH_BASE      0x08000000   // FLASH的起始地址
#define FMC_FLASH_END       0x08080000   // FLASH的结束地址

#define FLASH_WAITETIME     50000        //FLASH等待超时时间

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) 	//扇区11起始地址,128 Kbytes 

//读取指定地址的字(32位数据) 
//faddr:读地址 
//返回值:对应数据.
static uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
    return *(uint32_t*)faddr; 
}

//获取某个地址所在的flash扇区
//addr:flash地址
//返回值:0~11,即addr所在的扇区
uint8_t STMFLASH_GetFlashSector(uint32_t addr)
{
    if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
    else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
    else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
    else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
    else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
    else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
    else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6;
    else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_SECTOR_7;
    else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_SECTOR_8;
    else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_SECTOR_9;
    else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_SECTOR_10;   
    return FLASH_SECTOR_11;	
}
/**
  * @brief  擦除指定地址范围的Flash扇区
  * @param  StartAddress: 起始地址
  * @param  Length: 需要擦除的字节长度
  * @retval HAL_StatusTypeDef: 操作状态
  */
HAL_StatusTypeDef EraseFlashArea(uint32_t StartAddress, uint32_t Length)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    uint32_t SectorError = 0;
    
    // 参数检查
    if((StartAddress < FMC_FLASH_BASE) || 
       (StartAddress + Length > FMC_FLASH_END) || 
       (Length == 0))
    {
        return HAL_ERROR;
    }
    
    // 计算起始扇区和结束扇区
    uint32_t start_sector = STMFLASH_GetFlashSector(StartAddress);
    uint32_t end_sector = STMFLASH_GetFlashSector(StartAddress + Length - 1);
    uint32_t sectors_to_erase = end_sector - start_sector + 1;
    
    // 解锁Flash
    HAL_FLASH_Unlock();
    
    // 配置擦除参数
    FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    FlashEraseInit.Sector = start_sector;
    FlashEraseInit.NbSectors = sectors_to_erase;
    FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    
    // 执行擦除操作
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError);
    
    // 等待操作完成
    if(status == HAL_OK)
    {
        status = FLASH_WaitForLastOperation(FLASH_WAITETIME);
    }
    
    // 上锁Flash
    HAL_FLASH_Lock();
    
    return status;
}
/**
  * @brief  向Flash写入数据
  * @param  WriteAddress: 写入起始地址
  * @param  data: 要写入的数据指针
  * @param  length: 要写入的数据长度(字节)
  * @retval HAL_StatusTypeDef: 操作状态
  */
HAL_StatusTypeDef WriteFlashData(uint32_t WriteAddress, uint8_t *data, uint32_t length)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t endaddr = WriteAddress + length;
    
    // 参数检查
    if((WriteAddress < FMC_FLASH_BASE) || (endaddr >= FMC_FLASH_END) || (length <= 0))
        return HAL_ERROR;
    
    HAL_FLASH_Unlock();  // 解锁Flash
    
    // 等待上次操作完成
    status = FLASH_WaitForLastOperation(FLASH_WAITETIME);
    if(status != HAL_OK)
    {
        HAL_FLASH_Lock();  // 上锁
        return status;     // 操作失败
    }
    
    // 写入数据
    while(WriteAddress < endaddr)
    {
        // 按字节写入
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, WriteAddress, *data) != HAL_OK)
        {
            HAL_FLASH_Lock();  // 上锁
            return HAL_ERROR;  // 写入失败
        }
        
        WriteAddress += 1;  // 地址递增
        data++;             // 数据指针递增
    }
    
    HAL_FLASH_Lock();  // 上锁
    return HAL_OK;     // 写入成功
}
