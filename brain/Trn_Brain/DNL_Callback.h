//include cuối "USER CODE BEGIN 0"
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == compass.Instance)
	{
		compassTxCplt = 1;
	}
	else if(huart->Instance == PS2.Instance)
	{
		PS2TxCplt = 1;
	}
	else if(huart->Instance == spinalCord.Instance)
	{
		spinalCordTxCplt = 1;
	}
	else if(huart->Instance == manualRobot.Instance)
	{
		manualRobotTxCplt = 1;
	}
}
int trackingReceiSpinalCord;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == compass.Instance)
	{
		compassRxCplt = 1;
	}
	else if(huart->Instance == PS2.Instance)
	{
		PS2RxCplt = 1;
	}
	else if(huart->Instance == spinalCord.Instance)
	{
		if(spinalCordRxPacket[0]!=0)
		{
			trackingReceiSpinalCord++;
			HAL_UART_Transmit_IT(&spinalCord, spinalCordTxPacket, 9);
			// tracking = 0;
		}
		spinalCordRxCplt = 1;
		HAL_UART_Receive_IT(&spinalCord, spinalCordRxPacket, 1);
	}
	else if(huart->Instance == manualRobot.Instance)
	{
		manualRobotRxCplt = 1;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == leg.Instance)
	{
		if(legEn == 1)
		{
			if(legStatus == legInitShoot)
			{
				HAL_GPIO_WritePin(legDir_GPIO_Port, legDir_Pin, legBackward);
				HAL_GPIO_TogglePin(legPul_GPIO_Port, legPul_Pin);
				trackingLeg++;
	//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
				legElapsedPulses++;
				if (legElapsedPulses >= legInitShootPulse)
				{
					legEn = 0;
					legElapsedPulses = 0;
				}
			}
			if(legStatus == legReInitShoot)
			{
				HAL_GPIO_WritePin(legDir_GPIO_Port, legDir_Pin, legBackward);
				HAL_GPIO_TogglePin(legPul_GPIO_Port, legPul_Pin);
				trackingLeg++;
	//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
				legElapsedPulses++;
				if (legElapsedPulses >= legReInitShootPulse)
				{
					legEn = 0;
					legElapsedPulses = 0;
				}
			}
			if(legStatus == legEnd)
			{
				HAL_GPIO_WritePin(legDir_GPIO_Port, legDir_Pin, legBackward);	//quay ngược từ vị trí sút đến vị trí 0 (ngược chiều sút)
				HAL_GPIO_TogglePin(legPul_GPIO_Port, legPul_Pin);
				trackingLeg++;
	//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
				legElapsedPulses++;
				if (legElapsedPulses >= legEndPulse)
				{
					legEn = 0;
					legElapsedPulses = 0;
				}
			}
		}
		if(rigtArmEn == 1)
		{
			if(rigtArmStatus == rigtArmInit)
			{
				HAL_GPIO_WritePin(rigtArmDir_GPIO_Port, rigtArmDir_Pin, rigtArm_CCW);
				HAL_GPIO_TogglePin(rigtArmPul_GPIO_Port, rigtArmPul_Pin);
				trackingRigtArm++;
				rigtArmElapsedPulses++;
				if (rigtArmElapsedPulses >= rigtArmInitPulse)
				{
					rigtArmEn = 0;
					rigtArmElapsedPulses = 0;
				}
			}
			if(rigtArmStatus == rigtArmSetBall)
			{
				HAL_GPIO_WritePin(rigtArmDir_GPIO_Port, rigtArmDir_Pin, rigtArm_CCW);
				HAL_GPIO_TogglePin(rigtArmPul_GPIO_Port, rigtArmPul_Pin);
				trackingRigtArm++;
				rigtArmElapsedPulses++;
				if (rigtArmElapsedPulses >= rigtArmSetBallPulse)
				{
					rigtArmEn = 0;
					rigtArmElapsedPulses = 0;
				}
			}
			if(rigtArmStatus == rigtArmReturn)
			{
				HAL_GPIO_WritePin(rigtArmDir_GPIO_Port, rigtArmDir_Pin, rigtArm_FCW);
				HAL_GPIO_TogglePin(rigtArmPul_GPIO_Port, rigtArmPul_Pin);
				trackingRigtArm++;
				rigtArmElapsedPulses++;
				if (rigtArmElapsedPulses >= rigtArmSetBallPulse)
				{
					rigtArmEn = 0;
					rigtArmElapsedPulses = 0;
				}
			}
		}
		if(leftArmEn == 1)
		{
			if(leftArmStatus == leftArmInit)
			{
				HAL_GPIO_WritePin(leftArmDir_GPIO_Port, leftArmDir_Pin, leftArm_CCW);
				HAL_GPIO_TogglePin(leftArmPul_GPIO_Port, leftArmPul_Pin);
				trackingLeftArm++;
				leftArmElapsedPulses++;
				if (leftArmElapsedPulses >= leftArmInitPulse)
				{
					leftArmEn = 0;
					leftArmElapsedPulses = 0;
				}
			}
			if(leftArmStatus == leftArmSetBall)
			{
				HAL_GPIO_WritePin(leftArmDir_GPIO_Port, leftArmDir_Pin, leftArm_CCW);
				HAL_GPIO_TogglePin(leftArmPul_GPIO_Port, leftArmPul_Pin);
				trackingLeftArm++;
				leftArmElapsedPulses++;
				if (leftArmElapsedPulses >= leftArmSetBallPulse)
				{
					leftArmEn = 0;
					leftArmElapsedPulses = 0;
				}
			}
			if(leftArmStatus == leftArmReturn)
			{
				HAL_GPIO_WritePin(leftArmDir_GPIO_Port, leftArmDir_Pin, leftArm_FCW);
				HAL_GPIO_TogglePin(leftArmPul_GPIO_Port, leftArmPul_Pin);
				trackingLeftArm++;
				leftArmElapsedPulses++;
				if (leftArmElapsedPulses >= leftArmSetBallPulse)
				{
					leftArmEn = 0;
					leftArmElapsedPulses = 0;
				}
			}
		}
	}
	if(htim->Instance == rigtArm.Instance)
	{
		
	}
	if(htim->Instance == leftArm.Instance)
	{

	}
}
