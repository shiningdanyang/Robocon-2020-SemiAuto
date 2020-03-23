//include sau DelayMicroseconds.h
void peripheralUART_Init(void);

#define compass huart1
uint8_t compassTxPacket[9] = "compassTx";
uint8_t compassRxPacket[9];
uint8_t compassTxCplt;
uint8_t compassRxCplt;
int16_t compassData;
void compassReset(void);
void compassRequest(void);
void compassGetData(void);
void wait4CompassTx(void);
void wait4CompassRx(void);

#define PS2 huart3
uint8_t PS2TxPacket[8] = "PS2Tx123";
uint8_t PS2RxPacket[8];
uint8_t PS2TxCplt;
uint8_t PS2RxCplt;
void PS2Trans(void);
void PS2Recei(void);
void wait4PS2Tx(void);
void wait4PS2Rx(void);

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

#define manualRobot huart7
uint8_t manualRobotTxPacket[8] = "manualRo";
uint8_t manualRobotRxPacket[8];
uint8_t manualRobotTxCplt;
uint8_t manualRobotRxCplt;
void manualRobotTrans(void);
void manualRobotRecei(void);
void wait4ManualRobotTx(void);
void wait4ManualRobotRx(void);

int trackingWait4CompassTx;
int trackingWait4CompassRx;
int trackingWait4SpinalCordTx;
int trackingWait4SpinalCordRx;
void peripheralUART_Init()
{
	HAL_UART_Receive_IT(&spinalCord, spinalCordRxPacket, 1);
}

////////////////////////////////////////////////////////////
void compassReset(void)
{
	compassTxPacket[0] = 'a';
	HAL_UART_Transmit_IT(&compass, compassTxPacket, 1);
	wait4CompassTx();
}
void compassRequest(void)
{
	compassTxPacket[0] = 'z';
	HAL_UART_Transmit_IT(&compass, compassTxPacket, 1);
	trackingWait4CompassTx = 0;
	wait4CompassTx();
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
void PS2Trans(void)
{
	HAL_UART_Transmit_IT(&PS2, PS2TxPacket, 8);
	wait4PS2Tx();
}
void PS2Recei(void)
{
	HAL_UART_Receive_IT(&PS2, PS2RxPacket, 2);
	wait4PS2Rx();
}
void wait4PS2Tx(void)
{
	while(PS2TxCplt == 0);
	PS2TxCplt = 0;
}
void wait4PS2Rx(void)
{
	while(PS2RxCplt == 0);
	PS2RxCplt = 0;
}
////////////////////////////////////////////////////////////
void manualRobotTrans()
{
	HAL_UART_Transmit_IT(&manualRobot, manualRobotTxPacket, 8);
	wait4ManualRobotTx();
}
void manualRobotRecei()
{
	HAL_UART_Receive_IT(&manualRobot, manualRobotRxPacket, 2);
}
void wait4ManualRobotTx(void)
{
	while(manualRobotTxCplt == 0);
	manualRobotTxCplt = 0;
}
void wait4ManualRobotRx(void)
{
	while(manualRobotRxCplt == 0);
	manualRobotRxCplt = 0;
}
