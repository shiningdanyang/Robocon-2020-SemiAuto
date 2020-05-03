//include sau DNL_Slave_Peripheral_UART.h
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == brain.Instance)
	{
		brainRxCplt = 1;
		// HAL_UART_Receive_IT(&master, dataFromMaster, 4);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == brain.Instance)
	{
		brainTxCplt = 1;
		// HAL_UART_Receive_IT(&master, dataFromMaster, 4);
	}
}