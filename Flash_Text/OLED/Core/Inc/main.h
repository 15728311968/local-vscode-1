/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_CS_Pin GPIO_PIN_8
#define OLED_CS_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_9
#define OLED_DC_GPIO_Port GPIOC
#define OLED_RST_Pin GPIO_PIN_8
#define OLED_RST_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
// Flash 基础定义
#define STM32_FLASH_BASE        0x08000000UL  // Flash 起始地址
#define STM32_FLASH_SIZE        (512 * 1024)  // Flash 总大小 (512KB)

// Bootloader 和应用程序分区
#define BOOTLOADER_SIZE         (128 * 1024)  // Bootloader 大小 (128KB)
#define APPLICATION_BASE        (STM32_FLASH_BASE + BOOTLOADER_SIZE)  // 应用程序起始地址
#define APPLICATION_SIZE        (STM32_FLASH_SIZE - BOOTLOADER_SIZE)  // 应用程序大小

// 扇区定义（用于擦除操作）
#define BOOTLOADER_SECTORS      0, 1, 2, 3, 4  // Bootloader 占用的扇区 (0-4)
#define APPLICATION_SECTORS     5, 6, 7        // 应用程序占用的扇区 (5-7)

// 扇区大小和边界
#define SECTOR_0_SIZE           (16 * 1024)    // 扇区 0 大小
#define SECTOR_1_SIZE           (16 * 1024)    // 扇区 1 大小
#define SECTOR_2_SIZE           (16 * 1024)    // 扇区 2 大小
#define SECTOR_3_SIZE           (16 * 1024)    // 扇区 3 大小
#define SECTOR_4_SIZE           (64 * 1024)    // 扇区 4 大小
#define SECTOR_5_SIZE           (128 * 1024)   // 扇区 5 大小
#define SECTOR_6_SIZE           (128 * 1024)   // 扇区 6 大小
#define SECTOR_7_SIZE           (128 * 1024)   // 扇区 7 大小

#define UPDATA_A_FLAG			0x00000001

#define OTA_SET_FLAG  		0xAABB1122
typedef struct{
	uint32_t OTA_flag;
	uint32_t Firelen[11];   //0号成员固定对应OTA的大小
}OTA_InfoCB;
#define OTA_INFOCB_SIZE sizeof(OTA_InfoCB)
	
typedef struct{
	uint8_t Updatabuff[1024];
	uint32_t W25Q128_BlockNB;
}UpDataA_CB;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
