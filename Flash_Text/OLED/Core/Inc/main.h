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
// Flash ��������
#define STM32_FLASH_BASE        0x08000000UL  // Flash ��ʼ��ַ
#define STM32_FLASH_SIZE        (512 * 1024)  // Flash �ܴ�С (512KB)

// Bootloader ��Ӧ�ó������
#define BOOTLOADER_SIZE         (128 * 1024)  // Bootloader ��С (128KB)
#define APPLICATION_BASE        (STM32_FLASH_BASE + BOOTLOADER_SIZE)  // Ӧ�ó�����ʼ��ַ
#define APPLICATION_SIZE        (STM32_FLASH_SIZE - BOOTLOADER_SIZE)  // Ӧ�ó����С

// �������壨���ڲ���������
#define BOOTLOADER_SECTORS      0, 1, 2, 3, 4  // Bootloader ռ�õ����� (0-4)
#define APPLICATION_SECTORS     5, 6, 7        // Ӧ�ó���ռ�õ����� (5-7)

// ������С�ͱ߽�
#define SECTOR_0_SIZE           (16 * 1024)    // ���� 0 ��С
#define SECTOR_1_SIZE           (16 * 1024)    // ���� 1 ��С
#define SECTOR_2_SIZE           (16 * 1024)    // ���� 2 ��С
#define SECTOR_3_SIZE           (16 * 1024)    // ���� 3 ��С
#define SECTOR_4_SIZE           (64 * 1024)    // ���� 4 ��С
#define SECTOR_5_SIZE           (128 * 1024)   // ���� 5 ��С
#define SECTOR_6_SIZE           (128 * 1024)   // ���� 6 ��С
#define SECTOR_7_SIZE           (128 * 1024)   // ���� 7 ��С

#define UPDATA_A_FLAG			0x00000001

#define OTA_SET_FLAG  		0xAABB1122
typedef struct{
	uint32_t OTA_flag;
	uint32_t Firelen[11];   //0�ų�Ա�̶���ӦOTA�Ĵ�С
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
