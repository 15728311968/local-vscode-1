#include "w25q128.h"

extern SPI_HandleTypeDef hspi1;

uint8_t spi1_read_write_byte(uint8_t data)  
{
    uint8_t rec_data = 0;
    
    HAL_SPI_TransmitReceive(&hspi1, &data, &rec_data, 1, 1000);  //spi��д���ݺ���������2����������͵����ݣ�����3����������յ�����
   
    return rec_data;  
}

//����24λ��ַ
static void w25q128_send_address(uint32_t address)  /*address����ַ��Χ0~16777215�ֽڣ���Ѱַ��ΧΪ0x00 ~ 0xFFFFFF */
{
    spi1_read_write_byte((uint8_t)((address)>>16));     /* ���� bit23 ~ bit16 ��ַ */
    spi1_read_write_byte((uint8_t)((address)>>8));      /* ���� bit15 ~ bit8  ��ַ */
    spi1_read_write_byte((uint8_t)address);             /* ���� bit7  ~ bit0  ��ַ */
}
//w25q128��ʼ��
void W25Q128_Init(void)
{
	uint16_t flash_type;
    spi1_read_write_byte(0xFF); /* ���DR�����ݼĴ�����,д��һ��0xFF */
    W25Q128_CS_Set();   //����Ƭѡ�źŲ�����SPIͨ��
	flash_type = w25q128_read_id();   /* ��ȡFLASH ID. */
	if (flash_type == W25Q128)
	{
		
	}
}
//��ȡw25q128оƬID
uint16_t w25q128_read_id(void)
{
    uint16_t deviceid;
 
    W25Q128_CS_Clr();  //����Ƭѡ�źŽ���SPIͨ��
    spi1_read_write_byte(FLASH_ManufactDeviceID);   /* ���Ͷ�ȡ ID ���� */
	
	w25q128_send_address(0x000000);
	
    deviceid = spi1_read_write_byte(0xFF) << 8;     /* ��ȡ��8λ�ֽ� */
    deviceid |= spi1_read_write_byte(0xFF);         /* ��ȡ��8λ�ֽ� */
    W25Q128_CS_Set();
 
    return deviceid;
}


//��ȡ״̬�Ĵ�����ֵ
uint8_t w25q128_rd_sr1(void)
{
    uint8_t rec_data = 0;
    
    W25Q128_CS_Clr();
	spi1_read_write_byte(FLASH_ReadStatusReg1);     // д��ָ��0x05����״̬�Ĵ���1
    rec_data = spi1_read_write_byte(0xFF);  //��ȡ״̬�Ĵ���1��ֵ
    W25Q128_CS_Set()
    
    return rec_data;
}

//�ȴ�W25Q128����
static void w25q128_wait_busy(void)
{
    while ((w25q128_rd_sr1() & 0x01) == 1);   /* �ȴ�״̬�Ĵ�����BUSYλ��� */
}
 
//W25Q128дʹ��,����λWELΪ1
void w25q128_write_enable(void)
{
    W25Q128_CS_Clr();
    spi1_read_write_byte(FLASH_WriteEnable);   /* ����ָ��0x06��дʹ�� */
    W25Q128_CS_Set();
}

//��������оƬ
void w25q128_erase_chip(void)
{
    w25q128_write_enable();    /* дʹ�� */
    w25q128_wait_busy();       /* �ȴ����� */
    W25Q128_CS_Clr();
    spi1_read_write_byte(FLASH_ChipErase);  /* ����ָ��0xC7����������оƬ */ 
    W25Q128_CS_Set();
    w25q128_wait_busy();       /* �ȴ�оƬ�������� */
}
 
//����һ������
void w25q128_erase_sector(uint32_t saddr)  /* saddr���ò����ǵڼ������� */
{
    saddr *= 4096;  /* һ��������СΪ4096�ֽ� */
    w25q128_write_enable();        /* дʹ�� */
    w25q128_wait_busy();           /* �ȴ����� */
    W25Q128_CS_Clr();
    spi1_read_write_byte(FLASH_SectorErase);    /* ����ָ��0x20������ָ������ */
    w25q128_send_address(saddr);   /* ���Ͳ�����������ַ */
    W25Q128_CS_Set();
    w25q128_wait_busy();           /* �ȴ������������ */
}

/*
��ȡW25Q128��FLASH����ָ����ַ��ʼ��ȡָ�����ȵ�����
pubf����Ҫ��ȡ������
addr��ָ���ĵ�ַ
datalen��ָ�������ݴ�С
*/
void W25Q128_Read(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    uint16_t i;
 
    W25Q128_CS_Clr();
    spi1_read_write_byte(FLASH_ReadData);       /* ����ָ��0x03����ȡ���� */
    w25q128_send_address(addr);                /* ������Ҫ��ȡ�����ݵ�ַ */
    
    for(i=0;i<datalen;i++)
    {
        pbuf[i] = spi1_read_write_byte(0XFF);   /* ѭ����ȡ */
    }
    
    W25Q128_CS_Set();
}

/*
��ҳд����ָ����ַд��С��256�ֽڵ�ָ�����ȵ����ݣ��ڷ�0xFF��д������ݻ�ʧ��
pubf����Ҫд�������
addr��ָ���ĵ�ַ
datalen��ָ�������ݴ�С
*/
static void w25q128_write_page(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    uint16_t i;
 
    w25q128_write_enable();    /* дʹ�� */
 
    W25Q128_CS_Clr();
    spi1_read_write_byte(FLASH_PageProgram);    /* ��������0x02��ҳд */
    w25q128_send_address(addr);                /* ����д���ҳ��ַ */
 
    for(i=0;i<datalen;i++)
    {
        spi1_read_write_byte(pbuf[i]);          /* ѭ��д�� */
    }
    
    W25Q128_CS_Set();
    w25q128_wait_busy();       /* �ȴ�д����� */
}

/*
��ҳд����ָ����ַд��ָ�����ȵ����ݣ��ڷ�0xFF��д������ݻ�ʧ��
pubf����Ҫд�������
addr��ָ���ĵ�ַ
datalen��ָ�������ݴ�С
*/
static void w25q128_write_nocheck(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    uint16_t pageremain;
    pageremain = 256 - addr % 256;  /* ��ȡָ����ַ��ҳ��ʣ���ֽ��� */
 
    if (datalen <= pageremain)      /* ָ����ַ��ҳ��ʣ���ֽ�����װ��ָ�����ݴ�С */
    {
        pageremain = datalen;  
    }
 
    while (1)
    { 
			/* ��ָ����ַ��ҳ��ʣ���ֽ�����װ��ָ�����ݴ�Сʱ,һ����д�� */
            
			/* ��ָ�����ݴ�С��ָ����ַ��ҳ��ʣ���ֽ���Ҫ��ʱ, ��д��ָ����ַ��ҳ��ʣ���ֽ�, Ȼ�����ʣ�����ݴ�С���в�ͬ���� */
        w25q128_write_page(pbuf, addr, pageremain);  //ҳд
 
        if (datalen == pageremain)   /* д������� */
        {
            break;  
        }
        else     /* datalen > pageremain */
        {
            pbuf += pageremain;         /* pbufָ���ַƫ��,ǰ���Ѿ�д��pageremain�ֽ� */
            addr += pageremain;         /* д��ַƫ��,ǰ���Ѿ�д��pageremain�ֽ� */
            datalen -= pageremain;      /* д���ܳ��ȼ�ȥ�Ѿ�д���˵��ֽ��� */
 
            if (datalen > 256)          /* ʣ�����ݴ�С������һҳ */
            {
                pageremain= 256;       /* һ��д��256���ֽڣ���һ��дһҳ */
            }
            else     /* ʣ�����ݴ�СС��һҳ  */
            {
                pageremain= datalen;   /* һ����д�� */
            }
        }
    }
}

/*
//д��W25Q128��FLASH����ָ����ַ��д��ȡָ�����ȵ�����
pubf����Ҫд�������
addr��ָ���ĵ�ַ
datalen��ָ�������ݴ�С
*/
uint8_t g_w25q128_buf[4096];   /* �������� */
 
void W25Q128_Write(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;
    uint8_t *w25q128_buf;
 
    w25q128_buf = g_w25q128_buf;
    secpos = addr / 4096;       /* ��ȡָ����ַ����Ƭ���� */
    secoff = addr % 4096;       /* ָ���������������ڵ�ƫ�����ݴ�С */
    secremain = 4096 - secoff;  /* ����ʣ���ֽ��� */
 
    if (datalen <= secremain)  /* ָ����ַ��Ƭ������ʣ���ֽ�����װ��ָ�����ݴ�С */
    {
        secremain = datalen;    
    }
 
    while (1)
    {
        W25Q128_Read(w25q128_buf, secpos * 4096, 4096);   /* ����ָ����ַ��Ƭ������ȫ������ */
 
        for (i = 0; i < secremain; i++)   /* У�����ݣ���ֹ���ݳ��ַ�0xFF */
        {
            if (w25q128_buf[secoff + i] != 0xFF)  //����������һ�����ݲ���0xFF
            {
                break;      /* ��Ҫ����, ֱ���˳�forѭ�� */
            }
        }
 
        if (i < secremain)   /* ��Ҫ���� */
        {
            w25q128_erase_sector(secpos);  /* ����������� */
 
            for (i = 0; i < secremain; i++)   /* ���� */
            {
                w25q128_buf[i + secoff] = pbuf[i];
            }
 
            w25q128_write_nocheck(w25q128_buf, secpos * 4096, 4096);  /* д���������� */
        }
        else        /* д�Ѿ������˵�,ֱ��д������ʣ������. */
        {
            w25q128_write_nocheck(pbuf, addr, secremain);  /* ֱ��д���� */
        }
 
        if (datalen == secremain)
        {
            break;  /* д������� */
        }
        else        /* д��δ���� */
        {
            secpos++;               /* ������ַ��1���µ�һ������ */
            secoff = 0;             /* ƫ��λ��Ϊ0 */
 
            pbuf += secremain;      /* ָ��ƫ�� */
            addr += secremain;      /* д��ַƫ�� */
            datalen -= secremain;   /* �ֽ����ݼ� */
 
            if (datalen > 4096)
            {
                secremain = 4096;   /* һ��д��һ������ */
            }
            else
            {
                secremain = datalen;/* һ����д�� */
            }
        }
    }
}

