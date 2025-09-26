/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "w25q128.h"
#include "string.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t datatemp[20];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int flag;
OTA_InfoCB OTA_Info;
OTA_InfoCB OTA_Text;
UpDataA_CB UpDataA;

uint32_t BootSatFlag;

typedef  void (*load_a)(void);			//定义一个函数类型的参数.

//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(uint32_t addr) {
    MSR MSP, r0 			//set Main Stack value
    BX r14
}

load_a load_A; 

//跳转到应用程序段
//appxaddr:用户代码起始地址.
void LOAD_A(uint32_t addr) {
	if(((*(__IO uint32_t*)addr)&0x2FFE0000)==0x20000000) 	//检查栈顶地址是否合法.
	{ 
		load_A=(load_a)*(__IO uint32_t*)(addr+4);		//用户代码区第二个字为程序开始地址(复位地址)		
		MSR_MSP(*(__IO uint32_t*)addr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		load_A();									//跳转到APP.
	}
}

void BootLoader_Clear(void)
{
//	HAL_GPIO_DeInit();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
 
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();//OLED初始化
  OLED_Clear(); //OLED清屏
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//	W25Q128_Init();
//	memset(&OTA_Info,0,OTA_INFOCB_SIZE);
//	W25Q128_Read((uint8_t*)&OTA_Info,20*1024*64,OTA_INFOCB_SIZE);//读标志位
//	if(OTA_Info.OTA_flag == OTA_SET_FLAG)
//	{
//		BootSatFlag |= UPDATA_A_FLAG;
//		UpDataA.W25Q128_BlockNB = 0;
//	}
//	else
//	{
//		LOAD_A(APPLICATION_BASE);//跳转执行FLASH_APP代码
//	}
//	char data[]="Hello MQTT";
//	uint32_t data = 0xAABB1122;
//	OTA_Text.OTA_flag = 0x11223344;
//	for(int i=0;i<11;i++)
//	{
//		OTA_Text.Firelen[i] = 0x55;
//	}
//    W25Q128_Write((uint8_t*)&OTA_Text, 0x000000, OTA_INFOCB_SIZE);
//	memset(&OTA_Info,0,OTA_INFOCB_SIZE);
//	W25Q128_Read((uint8_t*)&OTA_Info,0x000000,OTA_INFOCB_SIZE);
//    
//    W25Q128_Read((uint8_t*)&datatemp, 0x000000, 4);


//	uint8_t i;
char data[] = "Hello MQTT";
EraseFlashArea(APPLICATION_BASE,APPLICATION_SIZE);
WriteFlashData(APPLICATION_BASE, (uint8_t*)data, 10);
ReadFlashData(APPLICATION_BASE,datatemp,10);
  while (1)
  {
//	  if(BootSatFlag&UPDATA_A_FLAG)
//	  {
//		  if(OTA_Info.Firelen[UpDataA.W25Q128_BlockNB] %4 ==0)
//		  {
//			  EraseFlashArea(APPLICATION_BASE,APPLICATION_SIZE);
//			  for(i=0;i<OTA_Info.Firelen[UpDataA.W25Q128_BlockNB]/1024;i++)
//			  {
//				  W25Q128_Read(UpDataA.Updatabuff, i*1024 + UpDataA.W25Q128_BlockNB*64*1024, 1024);
//				  WriteFlashData(APPLICATION_BASE+i*1024, UpDataA.Updatabuff, 1024);
//			  }
//			  if(OTA_Info.Firelen[UpDataA.W25Q128_BlockNB] % 1024 != 0)
//			  {
//				  W25Q128_Read(UpDataA.Updatabuff, i*1024 + UpDataA.W25Q128_BlockNB*64*1024, OTA_Info.Firelen[UpDataA.W25Q128_BlockNB] % 1024);
//				  WriteFlashData(APPLICATION_BASE+i*1024, UpDataA.Updatabuff, OTA_Info.Firelen[UpDataA.W25Q128_BlockNB] % 1024);				  
//			  }
//			  if(UpDataA.W25Q128_BlockNB == 0)
//			  {
//				  OTA_Info.OTA_flag = 0;
//				  W25Q128_Write((uint8_t*)&OTA_Info, 20*1024*64, OTA_INFOCB_SIZE);	//写入标志位  第20个块
//			  }
//			  NVIC_SystemReset();
//		  }
//		  else
//		  {
//			  //长度错误
//			  BootSatFlag &=~UPDATA_A_FLAG;
//		  }
//	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	OLED_ShowCHinese(0, 0, 0);  // 显示"科"
    OLED_ShowCHinese(16, 0, 1); // 显示"技"
	OLED_ShowNum(0,2,1233,4,16);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
