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

void roL_Pit_GoTo(uint32_t posRoL, uint32_t posPit)
{
	PIDroL(leftDistance, posRoL);
	PIDpit(pitchDistance, posPit);
	roL_pit_yaw_mixSpeed();
}

void roR_Pit_GoTo(uint32_t posRoR, uint32_t posPit)
{
	PIDroR(leftDistance, posRoR);
	PIDpit(pitchDistance, posPit);
	roR_pit_yaw_mixSpeed();
}


void goCross(uint8_t _speed, int cross, int posYaw)
{
	if(cross == 1)
	{
		PIDyaw(compassData, posYaw);
		controlMotor1(_speed+yawPID);
		controlMotor2(yawPID);
		controlMotor3(-_speed + yawPID);
		controlMotor4(yawPID);
	}
	else if(cross == 2)
	{
		PIDyaw(compassData, posYaw);
		controlMotor1(yawPID);
		controlMotor2(_speed+yawPID);
		controlMotor3(yawPID);
		controlMotor4(-_speed + yawPID);
	}
	else if(cross == 3)
	{
		PIDyaw(compassData, posYaw);
		controlMotor1(-_speed + yawPID);
		controlMotor2(yawPID);
		controlMotor3(_speed + yawPID);
		controlMotor4(yawPID);
	}
	else if(cross == 4)
	{
		PIDyaw(compassData, posYaw);
		controlMotor1(yawPID);
		controlMotor2(-_speed+yawPID);
		controlMotor3(yawPID);
		controlMotor4(_speed + yawPID);
	}
	spinalCordTrans();
}

void goStraight(int _speed, int straight, int posYaw)
{
	if(_speed>200)
	{
		_speed = 200;
	}
	if(straight == 1)
	{
		PIDyaw(compassData, posYaw);
		motor1Speed_ = yawPID+_speed;
		motor2Speed_ = yawPID+_speed;
		motor3Speed_ = yawPID-_speed;
		motor4Speed_ = yawPID-_speed;
		controlMotor1(motor1Speed_);
		controlMotor2(motor2Speed_);
		controlMotor3(motor3Speed_);
		controlMotor4(motor4Speed_);
	}
	else if (straight == 2)
	{
		PIDyaw(compassData, posYaw);
		controlMotor1(yawPID-_speed);
		controlMotor2(yawPID+_speed);
		controlMotor3(yawPID+_speed);
		controlMotor4(yawPID-_speed);
	}
	else if (straight == 3)
	{
		PIDyaw(compassData, posYaw);
		controlMotor1(yawPID-_speed);
		controlMotor2(yawPID-_speed);
		controlMotor3(yawPID+_speed);
		controlMotor4(yawPID+_speed);
	}
	else if(straight == 4)
	{
		PIDyaw(compassData, posYaw);
		controlMotor1(yawPID+_speed);
		controlMotor2(yawPID-_speed);
		controlMotor3(yawPID-_speed);
		controlMotor4(yawPID+_speed);
	}
	spinalCordTrans();
}

void roL_Yaw_Goto(uint32_t posRoL, int32_t posYaw)
{
	PIDyaw(compassData, posYaw);
	PIDroL(leftDistance, posYaw);
	roL_yaw_mixSpeed();
}
