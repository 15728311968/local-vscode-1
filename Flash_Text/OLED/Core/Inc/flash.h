#ifndef __FLASH_H
#define __FLASH_H
#include "main.h"

void ReadFlashData(uint32_t ReadAddress, uint8_t *data, uint32_t length);
HAL_StatusTypeDef WriteFlashData(uint32_t WriteAddress, uint8_t *data, uint32_t length);
HAL_StatusTypeDef EraseFlashArea(uint32_t StartAddress, uint32_t Length);
#endif  
