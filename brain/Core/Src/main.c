/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
//#define BLUEFIELD
#define REDFIELD
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_UART7_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "DNL_SemiAuto_Brain_DebugVariable.h"
#include "DNL_SemiAuto_Brain_DelayUs.h"
#include "DNL_SemiAuto_Brain_UART.h"
#include "DNL_SemiAuto_Brain_LCD.h"
#include "DNL_SemiAuto_Brain_ADC.h"
#include "DNL_SemiAuto_Brain_PID.h"
#include "DNL_SemiAuto_Brain_Position.h"
#include "DNL_SemiAuto_Brain_Leg.h"
#include "DNL_SemiAuto_Brain_FLASH.h"
#include "DNL_SemiAuto_Brain_Cylinder.h"
#include "DNL_SemiAuto_Brain_Sequence.h"

#include "DNL_Callback.h"
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
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_UART7_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  delayUs_Init();
  peripheralUART_Init();
  peripheralADC_Init();
  positionControl_Init();
  cylinder_Init();
  ST7920_Init();
  brake();
  freeMotor();
  compassReset();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);

  if(HAL_GPIO_ReadPin(flashSwitch_GPIO_Port, flashSwitch_Pin) == 0)	//nếu như gạt phải khởi động
  {
	  ST7920_SendString(0,0, "SAMPLE:");
	  getSample();
	  while(HAL_GPIO_ReadPin(flashSwitch_GPIO_Port, flashSwitch_Pin) == 0)	//ch�? gạt phải
	  {
		  ST7920_SendString(0,0, "PLEASE SWITCH");
		  HAL_Delay(1000);
		  ST7920_Clear();
		  HAL_Delay(1000);
	  }
	  while(1)
	  {
		  tracking++;
		  ST7920_SendString(0,0, "PLEASE RESET");
		  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
		  HAL_Delay(500);
		  ST7920_Clear();
		  HAL_Delay(500);
	  }
  }
  else	// nếu như gạt trái HAL_Read == 1
  {
	  ST7920_SendString(0,0, "FREE MODE");
	  tracking = 148;
	  readFLASH();
	  tracking = 150;
  }
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_Delay(INIT_TIME);
  if(btn_Sta == 0)
  {
	  while(1)
	  {
		  ST7920_SendString(0, 0, "disconBluetooth");
		  ST7920_SendString(1, 0, "pleaseReset");
		  HAL_Delay(500);
		  ST7920_Clear();
		  HAL_Delay(500);

	  }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#ifdef BLUEFIELD
	while (1)
	{
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  wait4SelectMode();
	  if(startMode == TO_SHOOT)
	  {

		  passArm(PASSARM_UP);
		  passHand(PASSHAND_OPEN);
		  startTime = HAL_GetTick();
		  ST7920_SendString(0,0, "btn_Sel waiting");
		  while((btn_Sel==1)&&(HAL_GetTick()-startTime<4700))
		  {
			  ST7920_Clear();
			  ST7920_SendString(0, 0, "autotuning");
			  roL_Pit_Yaw_GoTo(1200, 3700, 900);
		  }
		  startTime = HAL_GetTick();
		  while(HAL_GetTick()-startTime<1000)
		  {
			  ST7920_SendString(0, 0, "cross");
			  goCross(100, 2, 900);
		  }
		  ST7920_SendString(0,0, "btn_X   waiting");
		  while(btn_X==1)
		  {
			  ST7920_Clear();
			  ST7920_SendString(0, 0, "finish");
			  roL_Pit_Yaw_GoTo(2500,1500,900);
		  }
		  while(btn_Sel == 1)
		  {

//			  PIDyaw(compassData, 1250);
			  controlMotor1(-10);
			  controlMotor2(-10);
			  controlMotor3(-10);
			  controlMotor4(-10);
			  spinalCordTrans();
		  }
//		  ST7920_Clear();
//		  ST7920_SendString(0, 0, "ready2Put Q");
//		  ST7920_SendString(1, 0, "AIMING");
//		  ST7920_SendString(2, 0, "btn_Q waiting");
//		  while(btn_Q!=0)	//khi chưa nhấn nút Q;
//		  {
//			  if((btn_leftLeft==1)&&(btn_leftRigt==1))	//khi chưa nhấn 2 nút < và >
//			  {
//				  brake();
//			  }
//			  else if((btn_leftLeft==0)&&(btn_leftRigt==0))	//khi nhấn < + >
//			  {
//				  brake();
//			  }
//			  else if((btn_leftLeft==0)&&(btn_leftRigt==1))	//khi nhấn <
//			  {
//				  controlMotor1(turnSpeed);
//				  controlMotor2(turnSpeed);
//				  controlMotor3(turnSpeed);
//				  controlMotor4(turnSpeed);
//				  spinalCordTrans();
//			  }
//			  else if((btn_leftLeft==1)&&(btn_leftRigt==0))	//khi nhấn >
//			  {
//				  controlMotor1(-turnSpeed);
//				  controlMotor2(-turnSpeed);
//				  controlMotor3(-turnSpeed);
//				  controlMotor4(-turnSpeed);
//				  spinalCordTrans();
//			  }
//		  }
//		  ST7920_SendString(2,0,"              ");
//		  ST7920_SendString(2,0,"PUTTING Q");
//		  putQ(PUTQ_DOWN);//hạ putQ
//		  HAL_Delay(1000);//ch�? ...s
//		  gripperQ(GRIPPERQ_OPEN);//mở gripperQ
//		  HAL_Delay(500);//ch�? ...s
//		  while((btn_A==1)&&(btn_E==1))
//		  {
//			  ST7920_SendString(1, 0, "                ");
//			  ST7920_SendString(2, 0, "                ");
//			  ST7920_SendString(2, 0, "A or E");
//		  }
//		  if(btn_A == 0)
//		  {
//			  ST7920_Clear();
//			  ST7920_SendString(0,0,"SHOOT!!!!");
//			  legShoot();//shoot
//			  HAL_Delay(500);
//		  }
//		  else if(btn_E == 0)
//		  {
//		  }
//		  putQ(PUTQ_UP);//nâng putQ
//		  HAL_Delay(500);
//		  gripperQ(GRIPPERQ_CLOSE);//đóng gripperQ
//	//		  HAL_Delay(1000);//ch�? ...s
//		  ST7920_Clear();
//		  ST7920_SendString(0, 0, "ready2Put E");
//		  ST7920_SendString(1, 0, "AIMING");
//		  ST7920_SendString(2, 0, "btn_E waiting");
		  while(btn_E==1)	//khi chưa nhấn E
		  {

			  if((btn_leftLeft==1)&&(btn_leftRigt==1))	//khi chưa nhấn 2 nút < và >
			  {
				  brake();
				  HAL_Delay(300);
				  freeMotor();
			  }
			  else if((btn_leftLeft==0)&&(btn_leftRigt==0))	//khi nhấn < + >
			  {
				  brake();
				  HAL_Delay(300);
				  freeMotor();
			  }
			  else if((btn_leftLeft==0)&&(btn_leftRigt==1))	//khi nhấn <
			  {
				  controlMotor1(turnSpeed);
				  controlMotor2(turnSpeed);
				  controlMotor3(turnSpeed);
				  controlMotor4(turnSpeed);
				  spinalCordTrans();
			  }
			  else if((btn_leftLeft==1)&&(btn_leftRigt==0))	//khi nhấn >
			  {
				  controlMotor1(-turnSpeed);
				  controlMotor2(-turnSpeed);
				  controlMotor3(-turnSpeed);
				  controlMotor4(-turnSpeed);
				  spinalCordTrans();
			  }
		  }
		  ST7920_SendString(2,0,"              ");
		  ST7920_SendString(2,0,"PUTTING E");
		  putE(PUTE_DOWN);//hạ putE
		  HAL_Delay(1000);//ch�? ...s
		  gripperE(GRIPPERE_OPEN);//mở gripperE
	//		  HAL_Delay(500);//ch�? ...s
		  while((btn_A == 1)&&(btn_Q == 1))
		  {
			  ST7920_SendString(1, 0, "                ");
			  ST7920_SendString(2, 0, "                ");
			  ST7920_SendString(2, 0, "A or E");
		  }
		  if(btn_A == 0)
		  {
			  ST7920_Clear();
			  ST7920_SendString(0,0,"SHOOT!!!!");
			  legShoot();//shoot
			  HAL_Delay(500);
		  }
		  else if(btn_E == 0)
		  {
		  }
		  ST7920_Clear();
		  ST7920_SendString(0,0,"SHOOTED");
		  putE(PUTE_UP);//nâng putE
		  HAL_Delay(500);
		  gripperE(GRIPPERE_CLOSE);//đóng gripperE
	//		  while(btn_Sel==1)//khi chưa nhấn Sel
	//		  {
	//			  roL_Pit_Yaw_Goto(posRoL, posPit, -900);
	//		  }
		  MAX_PIT_PID = 100;
		  MAX_ROL_PID = 100;
		  MAX_ROR_PID = 100;
		  startTime = HAL_GetTick();
		  while(HAL_GetTick()-startTime < 500)
		  {
			  ST7920_Clear();

			  PIDyaw(compassData, 0);
			  controlMotor1(yawPID);
			  controlMotor2(yawPID);
			  controlMotor3(yawPID);
			  controlMotor4(yawPID);
			  spinalCordTrans();
		  }
		  while(btn_Sel == 1)
		  {
			  roR_Pit_Yaw_GoTo(1200, 3700, 900);
		  }
		  startTime = HAL_GetTick();
		  while(HAL_GetTick() - startTime < 500)
		  {
			  goCross(100, 1, 900);
		  }
		  startTime = HAL_GetTick();
		  while(HAL_GetTick()-startTime<300)
		  {
			  roR_Pit_Yaw_GoTo(1000, 400, 900);
		  }
		  startTime = HAL_GetTick();
		  while(HAL_GetTick() - startTime <800)
		  {
			  PIDyaw(compassData,0);
			  controlMotor1(yawPID);
			  controlMotor2(yawPID);
			  controlMotor3(yawPID);
			  controlMotor4(yawPID);
			  spinalCordTrans();
		  }
	  }
	  else if(startMode == BALL1)
	  {
		  goToBallLeft(ball1);
	  }
	  else if(startMode == BALL2)
	  {
		  goToBallLeft(ball2);
	  }
	  else if(startMode == BALL3)
	  {
		  goToBallLeft(ball3);
	  }
	  else if(startMode == BALL4)
	  {
		  goToBallLeft(ball4);
	  }
	  else if(startMode == LOAD_BALL)	//v�? vị trí load ball
	  {
		  passArm(PASSARM_UP);
		  startTime = HAL_GetTick();
		  while(HAL_GetTick()-startTime<2000)
		  {
			  roL_Pit_Yaw_GoTo(600, 500, 0);
		  }
		  ST7920_SendString(3, 0, "                ");
		  ST7920_SendString(3, 0, "returning2-900d");
		  startTime = HAL_GetTick();
		  while(((compassData-(900))>=10)&&((HAL_GetTick()-startTime)<800))//chưa đủ góc->hiệu chỉnh xoay đến -900
		  {
			  PIDyaw(compassData, 900);
			  controlMotor1(yawPID);
			  controlMotor2(yawPID);
			  controlMotor3(yawPID);
			  controlMotor4(yawPID);
			  spinalCordTrans();
			  if(abs(compassData-(900))<10)
			  {
				  brake();
				  HAL_Delay(DEBOUNCE_MOVING_TIME);
			  }
		  }
		  ST7920_Clear();

		  ST7920_SendString(2, 0, "Sel");
		  ST7920_SendString(3, 0, "autTningStaPoint");
		  while(btn_Sel!= 0)//chưa nhấn nút Sel -> hiệu chỉnh tự động
		  {
			  roL_Pit_Yaw_GoTo(350, 250, 900);
		  }
		  ST7920_Clear();
		  ST7920_SendString(2, 0, "btnX ");
		  ST7920_SendString(3, 0, "manualTuning");
		  while(btn_X!=0)//chưa nhấn nút X	->hiệu chỉnh vị trí
		  {
			  PIDyaw(compassData, 900);
			  leftVer = !btn_leftUp - !btn_leftDown;
			  leftHor = -!btn_leftLeft + !btn_leftRigt;
			  _dir = atan2(leftHor, -leftVer);
			  _controlSpeed = sqrt(leftVer*leftVer + leftHor*leftHor);
			  _motor1Speed = yawPID*factorYawPID + (factorSpeed*_controlSpeed *cos(3*M_PI/4 - _dir) + 0);
			  _motor2Speed = yawPID*factorYawPID + (factorSpeed*_controlSpeed *cos(3*M_PI/4 + _dir) - 0);
			  _motor3Speed = yawPID*factorYawPID +  factorSpeed*_controlSpeed *cos(  M_PI/4 + _dir) + 0;
			  _motor4Speed = yawPID*factorYawPID +  factorSpeed*_controlSpeed *cos(  M_PI/4 - _dir) - 0;
			  controlMotor1(_motor1Speed);
			  controlMotor2(_motor2Speed);
			  controlMotor3(_motor3Speed);
			  controlMotor4(_motor4Speed);
			  spinalCordTrans();
		  }
		  brake();
		  ST7920_Clear();
		  ST7920_SendString(2, 0, "Sel");
		  ST7920_SendString(3, 0, "loadingBall");
		  while(btn_Sel == 1)	//khi chưa nhấn nút Sel
		  {
			  gripperE(GRIPPERE_CLOSE);
			  gripperQ(GRIPPERQ_CLOSE);
		  }
	  }
	  else if(startMode == MANUAL_MODE)
	  {
		  manualMode();
	  }
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////test shoot/////////////////////////////////////////////
	//	while(HAL_GPIO_ReadPin(flashSwitch_GPIO_Port, flashSwitch_Pin)==preFlashSwitch)	//trong khi còn gạt trái
	//	{
	////		tracking=423;
	//	}
	//	for(int i = 0; i< 2600; ++i)
	//	{
	//		HAL_GPIO_WritePin(legEn_GPIO_Port, legEn_Pin, GPIO_PIN_RESET);
	//		HAL_GPIO_TogglePin(legPul_GPIO_Port, legPul_Pin);
	//		delayUs(8);
	//		tracking++;
	//	}
	//	tracking++;
	//	preFlashSwitch = HAL_GPIO_ReadPin(flashSwitch_GPIO_Port, flashSwitch_Pin);
	//	  legControl(LEG_STATUS_RUNUP);
	//	  HAL_Delay(5000);
	//	  while(1);
	//	  legShoot();
	//	  HAL_Delay(1000);
	//	  legControl(LEG_STATUS_RUNUP2);
	//	  HAL_Delay(1000);
	//	  legControl(legEnd);
	//	  HAL_Delay(3000);
	//
	//	  HAL_GPIO_WritePin(legEn_GPIO_Port, legEn_Pin, GPIO_PIN_RESET);	//enable legEn Pin
	//	  HAL_GPIO_WritePin(legDir_GPIO_Port, legDir_Pin, legBackward);
	//	  HAL_Delay(50);
	//	  for(int i = 0; i < LEG_PUL_SHOOT; ++i)
	//	  {
	//		  HAL_GPIO_WritePin(legPul_GPIO_Port, legPul_Pin, GPIO_PIN_SET);
	//		  delayUs(120);
	//		  HAL_GPIO_WritePin(legPul_GPIO_Port, legPul_Pin, GPIO_PIN_RESET);
	//		  delayUs(120);
	//		  trackingLegShoot++;
	//	  }

	///////////////////////////////////////////////////////////////////////
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
		}
	/* USER CODE END 3 */
	}
#endif


#ifdef REDFIELD
while (1)
{
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  wait4SelectMode();
  if(startMode == TO_SHOOT)
  {

	  passArm(PASSARM_UP);
	  passHand(PASSHAND_OPEN);
	  startTime = HAL_GetTick();
	  ST7920_SendString(0,0, "btn_Sel waiting");
	  while((btn_Sel==1)&&(HAL_GetTick()-startTime<4700))
	  {
		  ST7920_Clear();
		  ST7920_SendString(0, 0, "autotuning");
		  roR_Pit_Yaw_GoTo(1400, 3500, -900);
	  }
	  startTime = HAL_GetTick();
	  while(HAL_GetTick()-startTime<1000)
	  {
		  ST7920_SendString(0, 0, "cross");
		  goCross(100, 1, -900);
	  }
	  ST7920_SendString(0,0, "btn_X   waiting");
	  while(btn_X==1)
	  {
		  ST7920_Clear();
		  ST7920_SendString(0, 0, "finish");
		  roL_Pit_Yaw_GoTo(1300,1600,-900);
	  }
	  while(btn_Sel == 1)
	  {
//		  PIDyaw(compassData, -1200);
		  controlMotor1(8);
		  controlMotor2(8);
		  controlMotor3(8);
		  controlMotor4(8);
		  spinalCordTrans();
	  }
	  ST7920_Clear();
	  ST7920_SendString(0, 0, "ready2Put Q");
	  ST7920_SendString(1, 0, "AIMING");
	  ST7920_SendString(2, 0, "btn_Q waiting");
	  while(btn_E==1)	//khi chưa nhấn E
	  {

		  if((btn_leftLeft==1)&&(btn_leftRigt==1))	//khi chưa nhấn 2 nút < và >
		  {
			  brake();
			  HAL_Delay(300);
			  freeMotor();
		  }
		  else if((btn_leftLeft==0)&&(btn_leftRigt==0))	//khi nhấn < + >
		  {
			  brake();
			  HAL_Delay(300);
			  freeMotor();
		  }
		  else if((btn_leftLeft==0)&&(btn_leftRigt==1))	//khi nhấn <
		  {
			  controlMotor1(turnSpeed);
			  controlMotor2(turnSpeed);
			  controlMotor3(turnSpeed);
			  controlMotor4(turnSpeed);
			  spinalCordTrans();
		  }
		  else if((btn_leftLeft==1)&&(btn_leftRigt==0))	//khi nhấn >
		  {
			  controlMotor1(-turnSpeed);
			  controlMotor2(-turnSpeed);
			  controlMotor3(-turnSpeed);
			  controlMotor4(-turnSpeed);
			  spinalCordTrans();
		  }
	  }
	  ST7920_SendString(2,0,"              ");
	  ST7920_SendString(2,0,"PUTTING E");
	  putE(PUTE_DOWN);//hạ putE
	  HAL_Delay(1000);//ch�? ...s
	  gripperE(GRIPPERE_OPEN);//mở gripperE
//		  HAL_Delay(500);//ch�? ...s
	  while((btn_A == 1)&&(btn_Q == 1))
	  {
		  ST7920_SendString(1, 0, "                ");
		  ST7920_SendString(2, 0, "                ");
		  ST7920_SendString(2, 0, "A or E");
	  }
	  if(btn_A == 0)
	  {
		  ST7920_Clear();
		  ST7920_SendString(0,0,"SHOOT!!!!");
		  legShoot();//shoot
		  HAL_Delay(500);
	  }
	  else if(btn_E == 0)
	  {
	  }
	  ST7920_Clear();
	  ST7920_SendString(0,0,"SHOOTED");
	  putE(PUTE_UP);//nâng putE
	  HAL_Delay(500);
	  gripperE(GRIPPERE_CLOSE);//đóng gripperE
//		  while(btn_Sel==1)//khi chưa nhấn Sel
//		  {
//			  roL_Pit_Yaw_Goto(posRoL, posPit, -900);
//		  }
	  MAX_PIT_PID = 100;
	  MAX_ROL_PID = 100;
	  MAX_ROR_PID = 100;
	  startTime = HAL_GetTick();
//	  while(HAL_GetTick()-startTime < 500)
//	  {
//		  ST7920_Clear();
//
//		  PIDyaw(compassData, 0);
//		  controlMotor1(yawPID);
//		  controlMotor2(yawPID);
//		  controlMotor3(yawPID);
//		  controlMotor4(yawPID);
//		  spinalCordTrans();
//	  }
	  while(btn_Sel == 1)
	  {
		  roL_Pit_Yaw_GoTo(1200, 3700, -900);
	  }
	  startTime = HAL_GetTick();
	  while(HAL_GetTick() - startTime < 500)
	  {
		  goCross(100, 2, -900);
	  }
	  startTime = HAL_GetTick();
	  while(HAL_GetTick()-startTime<300)
	  {
		  roR_Pit_Yaw_GoTo(1000, 400, -900);
	  }
	  startTime = HAL_GetTick();
	  while(HAL_GetTick() - startTime <800)
	  {
		  PIDyaw(compassData,0);
		  controlMotor1(yawPID);
		  controlMotor2(yawPID);
		  controlMotor3(yawPID);
		  controlMotor4(yawPID);
		  spinalCordTrans();
	  }
  }
  else if(startMode == BALL1)
  {
	  goToBallRigt(ball1);
  }
  else if(startMode == BALL2)
  {
	  goToBallRigt(ball2);
  }
  else if(startMode == BALL3)
  {
	  goToBallRigt(ball3);
  }
  else if(startMode == BALL4)
  {
	  goToBallRigt(ball4);
  }
  else if(startMode == LOAD_BALL)	//v�? vị trí load ball
  {
	  passArm(PASSARM_UP);
	  startTime = HAL_GetTick();
	  while(HAL_GetTick()-startTime<2000)
	  {
		  roR_Pit_Yaw_GoTo(600, 500, 0);
	  }
	  ST7920_SendString(3, 0, "                ");
	  ST7920_SendString(3, 0, "returning2-900d");
	  startTime = HAL_GetTick();
	  while(((compassData-(900))>=10)&&((HAL_GetTick()-startTime)<800))//chưa đủ góc->hiệu chỉnh xoay đến -900
	  {
		  PIDyaw(compassData, -900);
		  controlMotor1(yawPID);
		  controlMotor2(yawPID);
		  controlMotor3(yawPID);
		  controlMotor4(yawPID);
		  spinalCordTrans();
		  if(abs(compassData-(-900))<10)
		  {
			  brake();
			  HAL_Delay(DEBOUNCE_MOVING_TIME);
		  }
	  }
	  ST7920_Clear();

	  ST7920_SendString(2, 0, "Sel");
	  ST7920_SendString(3, 0, "autTningStaPoint");
	  while(btn_Sel!= 0)//chưa nhấn nút Sel -> hiệu chỉnh tự động
	  {
		  roR_Pit_Yaw_GoTo(350, 250, -900);
	  }
	  ST7920_Clear();
	  ST7920_SendString(2, 0, "btnX ");
	  ST7920_SendString(3, 0, "manualTuning");
	  while(btn_X!=0)//chưa nhấn nút X	->hiệu chỉnh vị trí
	  {
		  PIDyaw(compassData, -900);
		  leftVer = !btn_leftUp - !btn_leftDown;
		  leftHor = -!btn_leftLeft + !btn_leftRigt;
		  _dir = atan2(leftHor, -leftVer);
		  _controlSpeed = sqrt(leftVer*leftVer + leftHor*leftHor);
		  _motor1Speed = yawPID*factorYawPID + (factorSpeed*_controlSpeed *cos(3*M_PI/4 - _dir) + 0);
		  _motor2Speed = yawPID*factorYawPID + (factorSpeed*_controlSpeed *cos(3*M_PI/4 + _dir) - 0);
		  _motor3Speed = yawPID*factorYawPID +  factorSpeed*_controlSpeed *cos(  M_PI/4 + _dir) + 0;
		  _motor4Speed = yawPID*factorYawPID +  factorSpeed*_controlSpeed *cos(  M_PI/4 - _dir) - 0;
		  controlMotor1(_motor1Speed);
		  controlMotor2(_motor2Speed);
		  controlMotor3(_motor3Speed);
		  controlMotor4(_motor4Speed);
		  spinalCordTrans();
	  }
	  brake();
	  ST7920_Clear();
	  ST7920_SendString(2, 0, "Sel");
	  ST7920_SendString(3, 0, "loadingBall");
	  while(btn_Sel == 1)	//khi chưa nhấn nút Sel
	  {
		  gripperE(GRIPPERE_CLOSE);
		  gripperQ(GRIPPERQ_CLOSE);
	  }

  }
  else if(startMode == MANUAL_MODE)
  {
	  manualMode();
  }


///////////////////////////////////////////////////////////////////////
/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
	}
/* USER CODE END 3 */
}
#endif
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART7
                              |RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 2399;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, passArm_Pin|gripperQ_Pin|gripperE_Pin|passHand_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13 
                          |putQ_Pin|putE_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|leftArmDir_Pin 
                          |rigtArmEn_Pin|rigtArmPul_Pin|leftArmEn_Pin|rigtArmDir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, legEn_Pin|leftArmPul_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_10|legDir_Pin|legPul_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : flashButton_Pin */
  GPIO_InitStruct.Pin = flashButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(flashButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : passArm_Pin gripperQ_Pin gripperE_Pin passHand_Pin */
  GPIO_InitStruct.Pin = passArm_Pin|gripperQ_Pin|gripperE_Pin|passHand_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB13 
                           putQ_Pin putE_Pin PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13 
                          |putQ_Pin|putE_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG2 PG3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : legEn_Pin */
  GPIO_InitStruct.Pin = legEn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(legEn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : flashSwitch_Pin */
  GPIO_InitStruct.Pin = flashSwitch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(flashSwitch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD10 legDir_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|legDir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : legPul_Pin */
  GPIO_InitStruct.Pin = legPul_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(legPul_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG4 PG5 PG6 PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : leftArmDir_Pin rigtArmEn_Pin rigtArmPul_Pin leftArmEn_Pin 
                           rigtArmDir_Pin */
  GPIO_InitStruct.Pin = leftArmDir_Pin|rigtArmEn_Pin|rigtArmPul_Pin|leftArmEn_Pin 
                          |rigtArmDir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : leftArmPul_Pin */
  GPIO_InitStruct.Pin = leftArmPul_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(leftArmPul_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
