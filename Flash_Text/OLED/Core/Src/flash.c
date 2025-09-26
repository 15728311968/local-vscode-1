#include "flash.h"

/**
 *@���ܣ����ڲ�Flash��ȡָ���ֽ�����
 *@����1��ReadAddress��������ʼ��ַ
 *@����2��*data��      ��ȡ�������ݻ����׵�ַ
 *@����3��length��     ��ȡ�ֽڸ���
 */
void ReadFlashData(uint32_t ReadAddress, uint8_t *data, uint32_t length)
{
    for(uint32_t i=0;i<length;i++)
    {
        data[i]=*(uint8_t*)(ReadAddress+i); //��ȡ����
    }
}
#define FMC_FLASH_BASE      0x08000000   // FLASH����ʼ��ַ
#define FMC_FLASH_END       0x08080000   // FLASH�Ľ�����ַ

#define FLASH_WAITETIME     50000        //FLASH�ȴ���ʱʱ��

//FLASH ��������ʼ��ַ
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) 	//����0��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) 	//����1��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) 	//����2��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) 	//����3��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) 	//����4��ʼ��ַ, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) 	//����5��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) 	//����6��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) 	//����8��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) 	//����9��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) 	//����10��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) 	//����11��ʼ��ַ,128 Kbytes 

//��ȡָ����ַ����(32λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
static uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
    return *(uint32_t*)faddr; 
}

//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
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
  * @brief  ����ָ����ַ��Χ��Flash����
  * @param  StartAddress: ��ʼ��ַ
  * @param  Length: ��Ҫ�������ֽڳ���
  * @retval HAL_StatusTypeDef: ����״̬
  */
HAL_StatusTypeDef EraseFlashArea(uint32_t StartAddress, uint32_t Length)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    uint32_t SectorError = 0;
    
    // �������
    if((StartAddress < FMC_FLASH_BASE) || 
       (StartAddress + Length > FMC_FLASH_END) || 
       (Length == 0))
    {
        return HAL_ERROR;
    }
    
    // ������ʼ�����ͽ�������
    uint32_t start_sector = STMFLASH_GetFlashSector(StartAddress);
    uint32_t end_sector = STMFLASH_GetFlashSector(StartAddress + Length - 1);
    uint32_t sectors_to_erase = end_sector - start_sector + 1;
    
    // ����Flash
    HAL_FLASH_Unlock();
    
    // ���ò�������
    FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    FlashEraseInit.Sector = start_sector;
    FlashEraseInit.NbSectors = sectors_to_erase;
    FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    
    // ִ�в�������
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError);
    
    // �ȴ��������
    if(status == HAL_OK)
    {
        status = FLASH_WaitForLastOperation(FLASH_WAITETIME);
    }
    
    // ����Flash
    HAL_FLASH_Lock();
    
    return status;
}
/**
  * @brief  ��Flashд������
  * @param  WriteAddress: д����ʼ��ַ
  * @param  data: Ҫд�������ָ��
  * @param  length: Ҫд������ݳ���(�ֽ�)
  * @retval HAL_StatusTypeDef: ����״̬
  */
HAL_StatusTypeDef WriteFlashData(uint32_t WriteAddress, uint8_t *data, uint32_t length)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t endaddr = WriteAddress + length;
    
    // �������
    if((WriteAddress < FMC_FLASH_BASE) || (endaddr >= FMC_FLASH_END) || (length <= 0))
        return HAL_ERROR;
    
    HAL_FLASH_Unlock();  // ����Flash
    
    // �ȴ��ϴβ������
    status = FLASH_WaitForLastOperation(FLASH_WAITETIME);
    if(status != HAL_OK)
    {
        HAL_FLASH_Lock();  // ����
        return status;     // ����ʧ��
    }
    
    // д������
    while(WriteAddress < endaddr)
    {
        // ���ֽ�д��
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, WriteAddress, *data) != HAL_OK)
        {
            HAL_FLASH_Lock();  // ����
            return HAL_ERROR;  // д��ʧ��
        }
        
        WriteAddress += 1;  // ��ַ����
        data++;             // ����ָ�����
    }
    
    HAL_FLASH_Lock();  // ����
    return HAL_OK;     // д��ɹ�
}
