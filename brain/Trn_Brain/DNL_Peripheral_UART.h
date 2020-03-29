//include sau DelayMicroseconds.h
void peripheralUART_Init(void);

#define compass huart1
uint8_t compassTxPacket[9] = "compassTx";
uint8_t compassRxPacket[9];
uint8_t compassTxCplt;
uint8_t compassRxCplt;
int16_t compassData;
uint8_t compassGetDataPeriod;
void compassReset(void);
void compassRequest(void);
void compassGetData(void);
void wait4CompassTx(void);
void wait4CompassRx(void);
void compassDeInit(void);
void compassInit(void);

#define PS2 huart3
uint8_t PS2TxPacket[8] = "PS2Tx123";
uint8_t PS2RxPacket[8];
uint8_t PS2TxCplt;
uint8_t PS2RxCplt;
void PS2Trans(void);
void PS2Recei(void);
void wait4PS2Tx(void);
void wait4PS2Rx(void);
void PS2DeInit(void);
void PS2Init(void);
uint8_t PS2CheckbyteCount = 0, PS2Data[6], PS2DataIndex;
int16_t PS2Button, PS2JoyLeft,PS2JoyRigt;
char* controlData;


#define spinalCord huart6
uint8_t spinalCordTxPacket[9] = "mainBoard";
uint8_t spinalCordRxPacket[9];
uint8_t spinalCordTxCplt;
uint8_t spinalCordRxCplt;
#define motor1Speed 	0
#define motor2Speed 	1
#define motor3Speed 	2
#define motor4Speed 	3
#define motor1Dir	4
#define motor2Dir	5
#define motor3Dir	6
#define motor4Dir	7
// #define spinal
void spinalCordTrans(void);
void spinalCordRecei(void);
void wait4SpinalCordTx(void);
void wait4SpinalCordRx(void);
void spinalCordDeInit(void);
void spinalCordInit(void);

#define manualRobot huart7
uint8_t manualRobotTxPacket[8] = "manualRo";
uint8_t manualRobotRxPacket[8];
uint8_t manualRobotTxCplt;
uint8_t manualRobotRxCplt;
void manualRobotTrans(void);
void manualRobotRecei(void);
void wait4ManualRobotTx(void);
void wait4ManualRobotRx(void);
void manualRobotDeinit(void);
void manualRobotInit(void);

int trackingWait4CompassTx;
int trackingWait4CompassRx;
int trackingWait4SpinalCordTx;
int trackingWait4SpinalCordRx;
void peripheralUART_Init()
{
	HAL_UART_Receive_IT(&spinalCord, spinalCordRxPacket, 1);
	HAL_UART_Receive_DMA(&PS2, PS2RxPacket, 1);
	HAL_UART_Receive_DMA(&compass, compassRxPacket, 2);
}

////////////////////////////////////////////////////////////
void compassDeInit()
{
	HAL_UART_DeInit(&compass);
}

void compassInit()
{
	HAL_UART_Init(&compass);
}

void compassReset(void)
{
	compassTxPacket[0] = 'a';
	HAL_UART_Transmit_IT(&compass, compassTxPacket, 1);
	wait4CompassTx();
	compassTxPacket[0] = 'z';
}
void compassRequest(void)
{
	HAL_UART_Transmit_IT(&compass, compassTxPacket, 1);
//	trackingWait4CompassTx = 0;
//	wait4CompassTx();
//	HAL_UART_Transmit(&compass, compassTxPacket, 1, 50);
}
void compassGetData(void)
{
	HAL_UART_Receive_IT(&compass, compassRxPacket, 2);
	wait4CompassRx();
//	HAL_UART_Receive(&compass, compassRxPacket, 2, 50);

	compassData = (compassRxPacket[0]<<8)|compassRxPacket[1];
}

void wait4CompassTx(void)
{
	while(compassTxCplt == 0)
	{
		trackingWait4CompassTx++;
	}
	compassTxCplt = 0;
}

void wait4CompassRx(void)
{
	while(compassRxCplt == 0)
	{
		trackingWait4CompassRx++;
	}
	compassRxCplt = 0;
}
////////////////////////////////////////////////////////////
void spinalCordDeInit(void)
{
	HAL_UART_DeInit(&spinalCord);
}

void spinalCordInit(void)
{
	HAL_UART_Init(&spinalCord);
}

void spinalCordTrans(void)
{
	HAL_UART_Transmit_IT(&spinalCord, (uint8_t*)spinalCordTxPacket, 8);
	wait4SpinalCordTx();
}
void spinalCordRecei(void)
{
	HAL_UART_Receive_IT(&spinalCord, spinalCordRxPacket, 2);
	wait4SpinalCordRx();
}

void wait4SpinalCordTx(void)
{
	while(spinalCordTxCplt == 0)
	{
		trackingWait4SpinalCordTx++;
	}
	spinalCordTxCplt = 0;
}

void wait4SpinalCordRx(void)
{
	while(spinalCordRxCplt == 0)
	{
		trackingWait4SpinalCordRx++;
	}
	spinalCordRxCplt = 0;
}

////////////////////////////////////////////////////////////
void PS2DeInit(void)
{
	HAL_UART_DeInit(&PS2);
}

void PS2Init(void)
{
	HAL_UART_Init(&PS2);
}

void PS2Trans(void)
{
	HAL_UART_Transmit_IT(&PS2, PS2TxPacket, 8);
	wait4PS2Tx();
}
void PS2Recei(void)
{
	HAL_UART_Receive_IT(&PS2, PS2RxPacket, 1);
	wait4PS2Rx();
}

int PS2TxTracking;
void wait4PS2Tx(void)
{
	while(PS2TxCplt == 0)
	{
		PS2TxTracking++;
	}
	PS2TxCplt = 0;
}
int PS2RxTracking;
void wait4PS2Rx(void)
{
	while(PS2RxCplt == 0)
	{
		PS2RxTracking++;
	}
	PS2RxCplt = 0;
}
////////////////////////////////////////////////////////////
void manualRobotDeInit(void)
{
	HAL_UART_DeInit(&manualRobot);
}

void manualRobotInit(void)
{
	HAL_UART_Init(&manualRobot);
}

void manualRobotTrans()
{
	HAL_UART_Transmit_IT(&manualRobot, manualRobotTxPacket, 8);
	wait4ManualRobotTx();
}
void manualRobotRecei()
{
	HAL_UART_Receive_IT(&manualRobot, manualRobotRxPacket, 1);
	wait4ManualRobotRx();
}
void wait4ManualRobotTx(void)
{
	while(manualRobotTxCplt == 0)
	{
	}
	manualRobotTxCplt = 0;
}
int manualRobotRxTracking;
void wait4ManualRobotRx(void)
{
	while(manualRobotRxCplt == 0)
	{
		manualRobotRxTracking++;
	}
	manualRobotRxCplt = 0;
}
