//include sau DelayMicroseconds.h
#define SPINAL_CORD_MODE_ONEWAY

#ifdef SPINAL_CORD_MODE_ONEWAY
#define INIT_TIME 500
#endif
#ifndef SPINAL_CORD_MODE_ONEWAY
#define INIT_TIME 2000
#endif

void peripheralUART_Init(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
////////////////////////////////////////////////////////////////
#define compass huart1
uint8_t compassTxPacket[9] = "compassTx";
uint8_t compassRxPacket[9];
uint8_t compassTxCplt_Flag;
uint8_t compassRxCplt_Flag;
int16_t compassData;
uint8_t compassGetDataPeriod;
void compassReset(void);
void compassRequest(void);
void compassGetData(void);
void wait4CompassTx(void);
void wait4CompassRx(void);
void compassDeInit(void);
void compassInit(void);
////////////////////////////////////////////////////////////////
#define PS2 huart3
uint8_t PS2TxPacket[8] = "PS2Tx123";
uint8_t PS2RxPacket[8];
uint8_t PS2TxCplt_Flag;
uint8_t PS2TxCplt_Flag;
void PS2Trans(void);
void PS2Recei(void);
void wait4PS2Tx(void);
void wait4PS2Rx(void);
void PS2DeInit(void);
void PS2Init(void);
uint8_t PS2CheckbyteCount = 0, PS2Data[6], PS2DataIndex;
int16_t PS2Button, PS2JoyLeft,PS2JoyRigt;
int16_t joyLeftMidVer=128;
int16_t joyLeftMidHor=128;
int16_t joyRigtMidVer=128;
int16_t joyRigtMidHor=128;
int16_t joyLeftHor;
int16_t joyLeftVer;
int16_t joyRigtHor;
int16_t joyRigtVer;
int16_t leftVer;
int16_t leftHor;
int16_t _motor1Speed;
int16_t _motor2Speed;
int16_t _motor3Speed;
int16_t _motor4Speed;
uint8_t btn_leftLeft, btn_leftRigt, btn_leftUp, btn_leftDown;
uint8_t btn_Sta, btn_joyLeft, btn_joyRigt, btn_Sel;
uint8_t btn_A, btn_X, btn_D, btn_W, btn_E, btn_Q, btn_C, btn_Z;

char* controlData;
////////////////////////////////////////////////////////////////
#define spinalCord huart6
uint8_t spinalCordTxPacket[13] = "mainBoard";
uint8_t spinalCordRxPacket[13];
uint8_t spinalCordTxCplt_Flag;
uint8_t spinalCordRxCplt_Flag;

#ifdef SPINAL_CORD_MODE_ONEWAY
#define motor1Speed 	4
#define motor2Speed 	5
#define motor3Speed 	6
#define motor4Speed		7
#define motorDir		8
#define motor1Dir		9
#define motor2Dir		10
#define motor3Dir		11
#define motor4Dir		12
#endif
#ifndef SPINAL_CORD_MODE_ONEWAY
#define motor1Speed 0
#define motor2Speed 1
#define motor3Speed 2
#define motor4Speed 3
#define motor1Dir 4
#define motor2Dir 5
#define motor3Dir 6
#define motor4Dir 7
#endif
void spinalCordTrans(void);
void spinalCordRecei(void);
void wait4SpinalCordTx(void);
void wait4SpinalCordRx(void);
void spinalCordDeInit(void);
void spinalCordInit(void);
////////////////////////////////////////////////////////////////
#define zmanual huart7
uint8_t zmanualTxPacket[8] = "manualRo";
uint8_t zmanualRxPacket[8];
uint8_t zmanualTxCplt_Flag;
uint8_t zmanualRxCplt_Flag;
void zmanualTrans(void);
void zmanualRecei(void);
void wait4zmanualTx(void);
void wait4zmanualRx(void);
void zmanualDeinit(void);
void zmanualInit(void);
////////////////////////////////////////////////////////////////
#ifdef SPINAL_CORD_MODE_ONEWAY
void peripheralUART_Init()
{
	spinalCordTxPacket[0] = 0xAA;
	spinalCordTxPacket[1] = 0xAA;
	spinalCordTxPacket[2] = 0xAA;
	spinalCordTxPacket[3] = 0xAA;
	spinalCordTxPacket[motorDir] = 0x00;
	HAL_UART_Receive_IT(&spinalCord, spinalCordRxPacket, 1);
	HAL_UART_Receive_DMA(&PS2, PS2RxPacket, 1);
	HAL_UART_Receive_DMA(&compass, compassRxPacket, 2);
	HAL_UART_Receive_DMA(&zmanual, zmanualRxPacket, 1);
}
#endif
#ifndef SPINAL_CORD_MODE_ONEWAY
void peripheralUART_Init()
{
	spinalCordTxPacket[8] = 'z';
	HAL_UART_Receive_IT(&spinalCord, spinalCordRxPacket, 1);
	HAL_UART_Receive_DMA(&PS2, PS2RxPacket, 1);
	HAL_UART_Receive_DMA(&compass, compassRxPacket, 2);
}
#endif
//////////////////////////////////////////////////////////////////////////////////////////
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == compass.Instance)
	{
		compassTxCplt_Flag = 1;
	}
	else if(huart->Instance == PS2.Instance)
	{
		PS2TxCplt_Flag = 1;
	}
	else if(huart->Instance == spinalCord.Instance)
	{
		spinalCordTxCplt_Flag = 1;
	}
	else if(huart->Instance == zmanual.Instance)
	{
		zmanualTxCplt_Flag = 1;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == compass.Instance)
	{
		compassRxCplt_Flag = 1;
	}
	else if(huart->Instance == PS2.Instance)
	{
		PS2TxCplt_Flag = 1;
	}
	else if(huart->Instance == spinalCord.Instance)
	{
		if(spinalCordRxPacket[0]=='t')
		{
			HAL_UART_Transmit(&spinalCord, spinalCordTxPacket, 9,100);
		}
		spinalCordRxCplt_Flag = 1;
		HAL_UART_Receive_IT(&spinalCord, spinalCordRxPacket, 1);
	}
	else if(huart->Instance == zmanual.Instance)
	{
		zmanualRxCplt_Flag = 1;
	}
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
	compassData = (compassRxPacket[0]<<8)|compassRxPacket[1];
}

void wait4CompassTx(void)
{
	while(compassTxCplt_Flag == 0)
	{
	}
	compassTxCplt_Flag = 0;
}

void wait4CompassRx(void)
{
	while(compassRxCplt_Flag == 0)
	{
	}
	compassRxCplt_Flag = 0;
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
	HAL_UART_Transmit_IT(&spinalCord, (uint8_t*)spinalCordTxPacket, 9);
	wait4SpinalCordTx();
}
void spinalCordRecei(void)
{
	HAL_UART_Receive_IT(&spinalCord, spinalCordRxPacket, 2);
	wait4SpinalCordRx();
}

void wait4SpinalCordTx(void)
{
	while(spinalCordTxCplt_Flag == 0)
	{
	}
	spinalCordTxCplt_Flag = 0;
}

void wait4SpinalCordRx(void)
{
	while(spinalCordRxCplt_Flag == 0)
	{
	}
	spinalCordRxCplt_Flag = 0;
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
	while(PS2TxCplt_Flag == 0)
	{
		PS2TxTracking++;
	}
	PS2TxCplt_Flag = 0;
}
int PS2RxTracking;
void wait4PS2Rx(void)
{
	while(PS2TxCplt_Flag == 0)
	{
		PS2RxTracking++;
	}
	PS2TxCplt_Flag = 0;
}
////////////////////////////////////////////////////////////
void zmanualDeInit(void)
{
	HAL_UART_DeInit(&zmanual);
}

void zmanualInit(void)
{
	HAL_UART_Init(&zmanual);
}

void zmanualTrans()
{
	HAL_UART_Transmit_IT(&zmanual, zmanualTxPacket, 8);
	wait4zmanualTx();
}
void zmanualRecei()
{
	HAL_UART_Receive_IT(&zmanual, zmanualRxPacket, 1);
	wait4zmanualRx();
}
void wait4zmanualTx(void)
{
	while(zmanualTxCplt_Flag == 0)
	{
	}
	zmanualTxCplt_Flag = 0;
}
void wait4zmanualRx(void)
{
	while(zmanualRxCplt_Flag == 0)
	{
	}
	zmanualRxCplt_Flag = 0;
}
