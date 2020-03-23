//include sau DNL_Slave_MotorControl.h
#define brain huart1
uint8_t brainTxPacket[9]={'z', 'u', 'v', 'w', 'x', 'y', 'z', 'a'};
uint8_t brainRxPacket[9];
uint8_t brainTxCplt;
uint8_t brainRxCplt;
#define motor1Speed 0
#define motor2Speed 1
#define motor3Speed 2
#define motor4Speed 3
#define motor1Dir 4
#define motor2Dir 5
#define motor3Dir 6
#define motor4Dir 7
void brainRequest(void);
void brainGetData(void);
void wait4BrainTx(void);
void wait4BrainRx(void);

void peripheralUART_Init(void)
{
	HAL_UART_Receive_IT(&brain, brainRxPacket, 9);
}


void brainRequest(void)
{
	HAL_UART_Transmit_IT(&brain, brainTxPacket, 1);
	wait4BrainTx();
}
void brainGetData(void)
{
//	HAL_UART_Receive_IT(&brain, brainRxPacket, 8);
//	wait4BrainRx();
		HAL_UART_Receive(&brain, brainRxPacket, 9,50);
}

void compassRecei(void)
{
	HAL_UART_Receive_IT(&brain, brainRxPacket, 2);
	wait4BrainRx();
}

void wait4BrainTx(void)
{
	while(brainTxCplt == 0);
	brainTxCplt = 0;
}
void wait4BrainRx(void)
{
	while(brainRxCplt == 0);
	brainRxCplt = 0;
}
