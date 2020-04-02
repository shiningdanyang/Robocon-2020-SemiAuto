#define leg htim7
#define rigtArm htim12
#define leftArm htim15

#define rigtArm_CCW GPIO_PIN_RESET
#define rigtArm_FCW GPIO_PIN_SET
#define leftArm_CCW GPIO_PIN_RESET
#define leftArm_FCW GPIO_PIN_SET
#define legForward GPIO_PIN_RESET
#define legBackward GPIO_PIN_SET

#define legInitShoot 	0
#define legReInitShoot 	1
#define legEnd 			2

#define legInitShootPulse 450
#define legEndPulse 450
#define legReInitShootPulse 1400
#define legShootPulse 700
//#define rigtArmInitPulse 300
//#define leftArmInitPulse 300
//#define rigtArmSetBallPulse 400
//#define leftArmSetBallPulse 400
//#define rigtArmReturnPulse 400
//#define leftArmReturnPulse 400
#define LEG_SHOOT_DELAYUS 1
int trackingLeg;
int trackingLegShoot;
//int trackingLeftArm;
//int trackingRigtArm;

int legEn;
int rigtArmEn;
int leftArmEn;

int legStatus;
int rigtArmStatus;
int leftArmStatus;

int legElapsedPulses;
int rigtArmElapsedPulses;
int leftArmElapsedPulses;

void positionControl_Init(void)
{
	  HAL_TIM_Base_Start_IT(&leg);
	  HAL_TIM_Base_Start_IT(&leftArm);
	  HAL_TIM_Base_Start_IT(&rigtArm);
}

void legShoot()
{
	  HAL_Delay(50);
	  HAL_GPIO_WritePin(legEn_GPIO_Port, legEn_Pin, GPIO_PIN_RESET);	//enable legEn Pin
	  HAL_GPIO_WritePin(legDir_GPIO_Port, legDir_Pin, legForward);
	  HAL_Delay(50);
	  for(int i = 0; i < legShootPulse; ++i)
	  {
		  HAL_GPIO_WritePin(legPul_GPIO_Port, legPul_Pin, GPIO_PIN_SET);
		  delayUs(LEG_SHOOT_DELAYUS);
		  HAL_GPIO_WritePin(legPul_GPIO_Port, legPul_Pin, GPIO_PIN_RESET);
		  delayUs(LEG_SHOOT_DELAYUS);
		  trackingLegShoot++;
	  }
}
void legControl(int _legStatus)
{
	legEn = 1;
	legStatus = _legStatus;
}

//void leftArmControl(int _leftArmStatus)
//{
//	leftArmEn = 1;
//	leftArmStatus = _leftArmStatus;
//}
//
//void rigtArmControl(int _rigtArmStatus)
//{
//	rigtArmEn = 1;
//	rigtArmStatus = _rigtArmStatus;
//}
