#define compass huart1
uint8_t compassTxCpltFlag;
uint8_t compassRxCpltFlag;

void waitForTx()
{
	while(compassTxCpltFlag == 0);
	compassTxCpltFlag = 0;
}
void waitForRx()
{
	while(compassRxCpltFlag == 0);
	compassRxCpltFlag = 0;
}

void compassSendData(uint8_t* _data, int _sizeData)
{
	HAL_UART_Transmit_IT(&compass, _data, _sizeData);
	waitForTx();
}

void compassReceiData(uint8_t* _data, int _sizeData)
{
	HAL_UART_Receive_IT(&compass, _data, _sizeData);
	waitForRx();
}

void blink(void)
{
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
}