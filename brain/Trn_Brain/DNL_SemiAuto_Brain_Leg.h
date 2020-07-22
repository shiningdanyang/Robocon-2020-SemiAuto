#define leg htim7

#define legForward GPIO_PIN_RESET
#define legBackward GPIO_PIN_SET

#define LEG_STATUS_RUNUP 	0
#define LEG_STATUS_RUNUP2 	1
#define legEnd 			2

#define LEG_PUL_RUNUP 200
#define LEG_PUL_RUNUP2 1400
#define LEG_PUL_END 450
#define LEG_PUL_SHOOT 1300
#define LEG_DELAYUS_SHOOT 8

int trackingLeg;
int trackingLegShoot;

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
	  HAL_GPIO_WritePin(legEn_GPIO_Port, legEn_Pin, GPIO_PIN_RESET);
}
//
void legShoot()
{
	  HAL_Delay(50);
	  HAL_GPIO_WritePin(legEn_GPIO_Port, legEn_Pin, GPIO_PIN_RESET);	//enable legEn Pin
	  HAL_GPIO_WritePin(legDir_GPIO_Port, legDir_Pin, legForward);
	  HAL_Delay(50);
	  for(int i = 0; i < LEG_PUL_SHOOT; ++i)
	  {
		  HAL_GPIO_WritePin(legPul_GPIO_Port, legPul_Pin, GPIO_PIN_SET);
		  delayUs(LEG_DELAYUS_SHOOT);
		  HAL_GPIO_WritePin(legPul_GPIO_Port, legPul_Pin, GPIO_PIN_RESET);
		  delayUs(LEG_DELAYUS_SHOOT);
		  trackingLegShoot++;
	  }
}
void legControl(int _legStatus)
{
	legEn = 1;
	legStatus = _legStatus;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == leg.Instance)
	{
		if(legEn == 1)
		{
			if(legStatus == LEG_STATUS_RUNUP)
			{
				HAL_GPIO_WritePin(legDir_GPIO_Port, legDir_Pin, legBackward);	//cấu hình legDir để lùi
				HAL_GPIO_TogglePin(legPul_GPIO_Port, legPul_Pin);				//tạo xung chân legPul
				trackingLeg++;
				legElapsedPulses++;												//đếm số xung
				if (legElapsedPulses >= LEG_PUL_RUNUP)
				{
					legEn = 0;													//kết thúc quá trình điều khiển
					legElapsedPulses = 0;										//kết thúc quá trình điều khiển
				}
			}
			if(legStatus == LEG_STATUS_RUNUP2)
			{
				HAL_GPIO_WritePin(legDir_GPIO_Port, legDir_Pin, legForward);	//cấu hình chân legDir để tiến
				HAL_GPIO_TogglePin(legPul_GPIO_Port, legPul_Pin);				//tạo xung chân legPul
				trackingLeg++;
				legElapsedPulses++;												//đếm số xung
				if (legElapsedPulses >= LEG_PUL_RUNUP2)
				{
					legEn = 0;													//kết thúc quá trình điều khiển
					legElapsedPulses = 0;										//kết thúc quá trình điều khiển
				}
			}
			if(legStatus == legEnd)
			{
				HAL_GPIO_WritePin(legDir_GPIO_Port, legDir_Pin, legBackward);	//quay ngược từ vị trí sút đến vị trí 0 (ngược chiều sút)
				HAL_GPIO_TogglePin(legPul_GPIO_Port, legPul_Pin);				//tạo xung chân legPul
				trackingLeg++;
				legElapsedPulses++;												//đếm số xung
				if (legElapsedPulses >= LEG_PUL_END)
				{
					legEn = 0;													//kết thúc quá trình điều khiển
					legElapsedPulses = 0;										//kết thúc quá trình điều khiển
				}
			}
		}
	}
}
