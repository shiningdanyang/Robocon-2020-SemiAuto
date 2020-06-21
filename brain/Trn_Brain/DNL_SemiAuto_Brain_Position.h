uint32_t pos1[2];
uint32_t pos2[2];


uint16_t posBall1[2] = {};
uint16_t posBall2[2] = {};
uint16_t posBall3[2] = {};
uint16_t posBall4[2] = {};
uint16_t posBall5[2] = {};
//void roL_Pit_Yaw_GoTo(uint32_t posRoL, uint32_t posPit, uint32_t posYaw)
//{
//	PIDyaw(compassData, posYaw);
//	PIDroL(leftDistance, posRoL);
//	PIDpit(pitchDistance, posPit);
//	roL_pit_yaw_mixSpeed();
//}

void roL_Pit_Yaw_GoTo(uint32_t posRoL, uint32_t posPit, uint32_t posYaw)
{
	PIDyaw(compassData, posYaw);
	PIDroL(leftDistance, posRoL);
	PIDpit(pitchDistance, posPit);
	roL_pit_yaw_mixSpeed();
}


void roR_Pit_Yaw_GoTo(uint32_t posRoR, uint32_t posPit, uint32_t posYaw)
{
	PIDyaw(compassData, posYaw);
	PIDroR(rigtDistance, posRoR);
	PIDpit(pitchDistance, posPit);
	roR_pit_yaw_mixSpeed();
}

void roL_Pit_Goto(uint32_t posRoL, uint32_t posPit)
{
	PIDroL(leftDistance, posRoL);
	PIDpit(pitchDistance, posPit);
	roL_pit_yaw_mixSpeed();
}

void roR_Pit_Goto(uint32_t posRoR, uint32_t posPit)
{
	PIDroR(leftDistance, posRoR);
	PIDpit(pitchDistance, posPit);
	roR_pit_yaw_mixSpeed();
}
