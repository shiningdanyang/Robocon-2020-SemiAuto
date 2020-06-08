void loop(void)
{
	spinalCordTrans();
	HAL_Delay(500);
	spinalCordTxPacket[0] = 'T';
	spinalCordTxPacket[1] = 'B';
	spinalCordTxPacket[2] = '.';
	spinalCordTxPacket[3] = 'T';
	spinalCordTxPacket[4] = 'r';
	spinalCordTxPacket[5] = 'a';
	spinalCordTxPacket[6] = 'a';
	spinalCordTxPacket[7] = 'n';
}