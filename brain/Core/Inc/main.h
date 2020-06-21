/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define FLASH_BASE_ADDR      (uint32_t)(FLASH_BASE)
#define FLASH_END_ADDR       (uint32_t)(0x081FFFFF)

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */
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
#define flashButton_Pin GPIO_PIN_13
#define flashButton_GPIO_Port GPIOC
#define manualRobot_RX_Pin GPIO_PIN_6
#define manualRobot_RX_GPIO_Port GPIOF
#define manualRobot_TX_Pin GPIO_PIN_7
#define manualRobot_TX_GPIO_Port GPIOF
#define RollRigt_Pin GPIO_PIN_0
#define RollRigt_GPIO_Port GPIOC
#define RollLeft_Pin GPIO_PIN_1
#define RollLeft_GPIO_Port GPIOC
#define Pitch_Pin GPIO_PIN_3
#define Pitch_GPIO_Port GPIOC
#define passArm_Pin GPIO_PIN_2
#define passArm_GPIO_Port GPIOA
#define legEn_Pin GPIO_PIN_7
#define legEn_GPIO_Port GPIOE
#define flashSwitch_Pin GPIO_PIN_8
#define flashSwitch_GPIO_Port GPIOE
#define PS2_TX_Pin GPIO_PIN_10
#define PS2_TX_GPIO_Port GPIOB
#define PS2_RX_Pin GPIO_PIN_11
#define PS2_RX_GPIO_Port GPIOB
#define compass_TX_Pin GPIO_PIN_14
#define compass_TX_GPIO_Port GPIOB
#define compass_RX_Pin GPIO_PIN_15
#define compass_RX_GPIO_Port GPIOB
#define legDir_Pin GPIO_PIN_14
#define legDir_GPIO_Port GPIOD
#define legPul_Pin GPIO_PIN_15
#define legPul_GPIO_Port GPIOD
#define mainBoard_TX_Pin GPIO_PIN_6
#define mainBoard_TX_GPIO_Port GPIOC
#define mainBoard_RX_Pin GPIO_PIN_7
#define mainBoard_RX_GPIO_Port GPIOC
#define passHand_Pin GPIO_PIN_10
#define passHand_GPIO_Port GPIOA
#define leftArmDir_Pin GPIO_PIN_9
#define leftArmDir_GPIO_Port GPIOG
#define rigtArmEn_Pin GPIO_PIN_10
#define rigtArmEn_GPIO_Port GPIOG
#define rigtArmPul_Pin GPIO_PIN_11
#define rigtArmPul_GPIO_Port GPIOG
#define leftArmEn_Pin GPIO_PIN_12
#define leftArmEn_GPIO_Port GPIOG
#define rigtArmDir_Pin GPIO_PIN_13
#define rigtArmDir_GPIO_Port GPIOG
#define leftArmPul_Pin GPIO_PIN_1
#define leftArmPul_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
