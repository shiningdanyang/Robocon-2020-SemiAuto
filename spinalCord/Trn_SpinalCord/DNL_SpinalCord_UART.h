//include sau DNL_Slave_MotorControl.h
#define SPINAL_CORD_MODE_ONEWAY
#define brain huart1
uint8_t brainTxPacket[9]={'z', 'u', 'v', 'w', 'x', 'y', 'z', 'a'};
uint8_t brainRxPacket[9];
uint8_t brainTxCplt;
uint8_t brainRxCplt;
//#define motor1Speed 0
//#define motor2Speed 1
//#define motor3Speed 2
//#define motor4Speed 3
//#define motor1Dir 4
//#define motor2Dir 5
//#define motor3Dir 6
//#define motor4Dir 7
void brainRequest(void);
void brainGetData(void);
void wait4BrainTx(void);
void wait4BrainRx(void);

uint8_t brainCheckbyteCount = 0, brainData[5], brainDataIndex;
char* controlData;

#ifdef SPINAL_CORD_MODE_ONEWAY
void peripheralUART_Init(void)
{
	HAL_UART_Receive_DMA(&brain, brainRxPacket, 1);
}
#endif
#ifndef SPINAL_CORD_MODE_ONEWAY
void peripheralUART_Init(void)
{
	HAL_UART_Receive_IT(&brain, brainRxPacket, 9);
}
#endif

void brainRequest(void)
{
	brainTxPacket[0] = 't';
	HAL_UART_Transmit_IT(&brain, brainTxPacket, 1);
	wait4BrainTx();
}
void brainGetData(void)
{
//	HAL_UART_Receive_IT(&brain, brainRxPacket, 8);
//	wait4BrainRx();
	HAL_UART_Receive(&brain, brainRxPacket, 9, 50);
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

#ifdef SPINAL_CORD_MODE_ONEWAY
//put in function handles DMA stream global interrupt of brainDMA Rx in stm32f4xx_it.c
void brainDMA_ProcessingData(void)
{
		if(brainCheckbyteCount == 4 )
		{
		  brainData[brainDataIndex++] = brainRxPacket[0];
			if(brainDataIndex > 5)
			{
				brainDataIndex = 0;
				brainCheckbyteCount = 0;
			}
		}
		if(brainRxPacket[0] == 0xAA)
			brainCheckbyteCount++;
		else
			if(brainCheckbyteCount != 4)
				brainCheckbyteCount = 0;
}
#endif
#ifndef SPINAL_CORD_MODE_ONEWAY
void brainDMA_ProcessingData(void)
{
}
#endif

