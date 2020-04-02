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
//		if(spinalCordRxPacket[0]!=0)
//		{
//			trackingReceiSpinalCord++;
//			HAL_UART_Transmit_IT(&spinalCord, spinalCordTxPacket, 9);
//			// tracking = 0;
//		}
		spinalCordRxCplt = 1;
//		HAL_UART_Receive_IT(&spinalCord, spinalCordRxPacket, 1);
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
				HAL_GPIO_WritePin(legDir_GPIO_Port, legDir_Pin, legBackward);	//cấu hình legDir để lùi
				HAL_GPIO_TogglePin(legPul_GPIO_Port, legPul_Pin);				//tạo xung chân legPul
				trackingLeg++;
	//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
				legElapsedPulses++;												//đếm số xung
				if (legElapsedPulses >= legInitShootPulse)
				{
					legEn = 0;													//kết thúc quá trình điều khiển
					legElapsedPulses = 0;										//kết thúc quá trình điều khiển
				}
			}
			if(legStatus == legReInitShoot)
			{
				HAL_GPIO_WritePin(legDir_GPIO_Port, legDir_Pin, legBackward);	//cấu hình chân legDir để lùi
				HAL_GPIO_TogglePin(legPul_GPIO_Port, legPul_Pin);				//tạo xung chân legPul
				trackingLeg++;
	//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
				legElapsedPulses++;												//đếm số xung
				if (legElapsedPulses >= legReInitShootPulse)
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
	//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
				legElapsedPulses++;												//đếm số xung
				if (legElapsedPulses >= legEndPulse)
				{
					legEn = 0;													//kết thúc quá trình điều khiển
					legElapsedPulses = 0;										//kết thúc quá trình điều khiển
				}
			}
		}
	}
}
