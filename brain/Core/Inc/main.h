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
#define manualRobot_RX_Pin GPIO_PIN_6
#define manualRobot_RX_GPIO_Port GPIOF
#define manualRobot_TX_Pin GPIO_PIN_7
#define manualRobot_TX_GPIO_Port GPIOF
#define RollRigt_Pin GPIO_PIN_0
#define RollRigt_GPIO_Port GPIOC
#define Pitch_Pin GPIO_PIN_3
#define Pitch_GPIO_Port GPIOC
#define cylinder_SetTee_Pin GPIO_PIN_2
#define cylinder_SetTee_GPIO_Port GPIOA
#define RollLeft_Pin GPIO_PIN_3
#define RollLeft_GPIO_Port GPIOA
#define cylinder_RigtArmTrans_Pin GPIO_PIN_1
#define cylinder_RigtArmTrans_GPIO_Port GPIOB
#define cylinder_PassBall_Pin GPIO_PIN_2
#define cylinder_PassBall_GPIO_Port GPIOB
#define legEn_Pin GPIO_PIN_7
#define legEn_GPIO_Port GPIOE
#define PS2_TX_Pin GPIO_PIN_10
#define PS2_TX_GPIO_Port GPIOB
#define PS2_RX_Pin GPIO_PIN_11
#define PS2_RX_GPIO_Port GPIOB
#define cylinder_LeftArmTrans_Pin GPIO_PIN_13
#define cylinder_LeftArmTrans_GPIO_Port GPIOB
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
#define cylinder_RigtArmHoldBallTop_Pin GPIO_PIN_8
#define cylinder_RigtArmHoldBallTop_GPIO_Port GPIOA
#define cylinder_HoldBall_Pin GPIO_PIN_9
#define cylinder_HoldBall_GPIO_Port GPIOA
#define cylinder_LeftArmHoldBall_Pin GPIO_PIN_10
#define cylinder_LeftArmHoldBall_GPIO_Port GPIOA
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
#define cylinder_RigtArmHoldBallBot_Pin GPIO_PIN_5
#define cylinder_RigtArmHoldBallBot_GPIO_Port GPIOB
#define cylinder_LiftBall_Pin GPIO_PIN_6
#define cylinder_LiftBall_GPIO_Port GPIOB
#define leftArmPul_Pin GPIO_PIN_1
#define leftArmPul_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
