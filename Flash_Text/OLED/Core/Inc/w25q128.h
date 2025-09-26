#ifndef __W25Q128_H
#define __W25Q128_H

#include "main.h"


#define W25Q128_CS_GPIO_PORT           GPIOD
#define W25Q128_CS_GPIO_PIN            GPIO_PIN_2

#define W25Q128_CS_Set()   HAL_GPIO_WritePin(W25Q128_CS_GPIO_PORT, W25Q128_CS_GPIO_PIN, GPIO_PIN_SET);
#define W25Q128_CS_Clr()   HAL_GPIO_WritePin(W25Q128_CS_GPIO_PORT, W25Q128_CS_GPIO_PIN, GPIO_PIN_RESET);

#define W25Q128     0XEF17          /* W25Q128  –æ∆¨ID */
 
/* ÷∏¡Ó±Ì */
#define FLASH_WriteEnable                   0x06 
#define FLASH_ReadStatusReg1                0x05 
#define FLASH_ReadData                      0x03 
#define FLASH_PageProgram                   0x02 
#define FLASH_SectorErase                   0x20 
#define FLASH_ChipErase                     0xC7 
#define FLASH_ManufactDeviceID              0x90 

uint16_t w25q128_read_id(void);
void W25Q128_Init(void);
void W25Q128_Write(uint8_t *pbuf, uint32_t addr, uint16_t datalen);
void W25Q128_Read(uint8_t *pbuf, uint32_t addr, uint16_t datalen);

#endif  

