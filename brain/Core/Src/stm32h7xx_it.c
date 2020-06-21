/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern void compassRequest(void);
extern uint8_t compassGetDataPeriod;
extern uint8_t compassRxPacket[2];
extern int16_t compassData;

extern uint8_t PS2RxPacket[8];
extern uint8_t PS2CheckbyteCount, PS2Data[6], PS2DataIndex;
extern int16_t PS2Button, PS2JoyLeft, PS2JoyRigt;
extern int16_t joyLeftMidVer;
extern int16_t joyLeftMidHor;
extern int16_t joyRigtMidVer;
extern int16_t joyRigtMidHor;
extern int16_t joyLeftHor;
extern int16_t joyLeftVer;
extern int16_t joyRigtHor;
extern int16_t joyRigtVer;
extern uint8_t btn_leftLeft, btn_leftRigt, btn_leftUp, btn_leftDown;
extern uint8_t btn_Sta, btn_joyLeft, btn_joyRigt, btn_Sel;
extern uint8_t btn_A, btn_X, btn_D, btn_W, btn_E, btn_Q, btn_C, btn_Z;

extern uint16_t adc3Value[3];
extern double leftRawDistance;
extern double rigtRawDistance;
extern double pitchRawDistance;
extern double leftDistance;
extern double rigtDistance;
extern double pitchDistance;
extern double aPitch_Linear;
extern double bPitch_Linear;
extern double aLeft_Linear;
extern double bLeft_Linear;
extern double aRigt_Linear;
extern double bRigt_Linear;
//#define aPitch_Linear 	1
//#define bPitch_Linear 	0
//#define aLeft_Linear 	0.7140957447
//#define bLeft_Linear 	145.2393617
//#define aRigt_Linear 	0.7168784029
//#define bRigt_Linear 	141.1651543

#define k 1

double kalmanGain_Pitch;
double x_Pitch[2];
double P_Pitch = 2;         //covariance estimation (err_estimate)
double R_Pitch = 2;         //covariance of the observation noise (err_measure)
double Q_Pitch = 0.0009;    //process variance

double kalmanGain_Left;
double x_Left[2];
double P_Left = 2;         //covariance estimation (err_estimate)
double R_Left = 2;         //covariance of the observation noise (err_measure)
double Q_Left = 0.0009;    //process variance

double kalmanGain_Rigt;
double x_Rigt[2];
double P_Rigt = 2;         //covariance estimation (err_estimate)
double R_Rigt = 2;         //covariance of the observation noise (err_measure)
double Q_Rigt = 0.0009;    //process variance

double kalmanFilter_Pitch(double mea)
{
  kalmanGain_Pitch = P_Pitch /(P_Pitch + R_Pitch);
  x_Pitch[k] = x_Pitch[k-1] + kalmanGain_Pitch *(mea - x_Pitch[k-1]);
  P_Pitch =  (1.0 - kalmanGain_Pitch) *P_Pitch + fabs(x_Pitch[k-1]-x_Pitch[k]) *Q_Pitch;
  x_Pitch[k-1] = x_Pitch[k];
  return x_Pitch[k];
}

double kalmanFilter_Left(double mea)
{
  kalmanGain_Left = P_Left /(P_Left + R_Left);
  x_Left[k] = x_Left[k-1] + kalmanGain_Left *(mea - x_Left[k-1]);
  P_Left =  (1.0 - kalmanGain_Left) *P_Left + fabs(x_Left[k-1]-x_Left[k]) *Q_Left;
  x_Left[k-1] = x_Left[k];
  return x_Left[k];
}

double kalmanFilter_Rigt(double mea)
{
  kalmanGain_Rigt = P_Rigt /(P_Rigt + R_Rigt);
  x_Rigt[k] = x_Rigt[k-1] + kalmanGain_Rigt *(mea - x_Rigt[k-1]);
  P_Rigt =  (1.0 - kalmanGain_Rigt) *P_Rigt + fabs(x_Rigt[k-1]-x_Rigt[k]) *Q_Rigt;
  x_Rigt[k-1] = x_Rigt[k];
  return x_Rigt[k];
}

//extern char* controlData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc3;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  compassGetDataPeriod++;
  if(compassGetDataPeriod > 5)
  {
	  compassGetDataPeriod = 0;
	  compassRequest();
  }
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

	rigtRawDistance = aRigt_Linear*adc3Value[0] + bRigt_Linear;
	leftRawDistance = aLeft_Linear*adc3Value[1] + bLeft_Linear;
	pitchRawDistance = aPitch_Linear*adc3Value[2] + bPitch_Linear;
	rigtDistance = kalmanFilter_Rigt(rigtRawDistance);
	pitchDistance = kalmanFilter_Pitch(pitchRawDistance);
	leftDistance = kalmanFilter_Left(leftRawDistance);

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
	  compassData = (compassRxPacket[0]<<8)|compassRxPacket[1];
  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
	if(PS2CheckbyteCount == 4 )
	{
	  PS2Data[PS2DataIndex++] = PS2RxPacket[0];
		if(PS2DataIndex > 5)
		{
			PS2DataIndex = 0;
			PS2CheckbyteCount = 0;
			PS2Button = (PS2Data[0]<<8) | PS2Data[1];
			joyRigtHor = PS2Data[2] - joyRigtMidHor;
			joyRigtVer = PS2Data[3] - joyRigtMidVer;
			joyLeftHor = PS2Data[4] - joyLeftMidHor;
			joyLeftVer = PS2Data[5] - joyLeftMidVer;
			btn_leftLeft = (PS2Button >> 15) & 1U;
			btn_leftDown = (PS2Button >> 14) & 1U;
			btn_leftRigt = (PS2Button >> 13) & 1U;
			btn_leftUp   = (PS2Button >> 12) & 1U;
			btn_Sta		 = (PS2Button >> 11) & 1U;
			btn_joyRigt  = (PS2Button >> 10) & 1U;
			btn_joyLeft  = (PS2Button >>  9) & 1U;
			btn_Sel  	 = (PS2Button >>  8) & 1U;
			btn_A  		 = (PS2Button >>  7) & 1U;
			btn_X  		 = (PS2Button >>  6) & 1U;
			btn_D  		 = (PS2Button >>  5) & 1U;
			btn_W  		 = (PS2Button >>  4) & 1U;
			btn_E  		 = (PS2Button >>  3) & 1U;
			btn_Q  		 = (PS2Button >>  2) & 1U;
			btn_C  		 = (PS2Button >>  1) & 1U;
			btn_Z  		 = (PS2Button >>  0) & 1U;
		}
	}
	if(PS2RxPacket[0] == 0xAA)
		PS2CheckbyteCount++;
	else
		if(PS2CheckbyteCount != 4)
			PS2CheckbyteCount = 0;
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
  /* USER CODE BEGIN UART7_IRQn 0 */

  /* USER CODE END UART7_IRQn 0 */
  HAL_UART_IRQHandler(&huart7);
  /* USER CODE BEGIN UART7_IRQn 1 */

  /* USER CODE END UART7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
