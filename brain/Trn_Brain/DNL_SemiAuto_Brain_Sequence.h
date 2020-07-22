#define LEFT 1
#define RIGT 2
#define FIELD LEFT
#define _Rigt 		0
#define _Left		1
#define _Pitc		2
#define _PitcWait	3
#define BALL1 		1
#define BALL2 		2
#define BALL3 		3
#define BALL4 		4
#define TO_SHOOT	5
#define LOAD_BALL	6
#define MANUAL_MODE 7
#define SHOOT_BALL_1 8
#define DEBOUNCE_MOVING_TIME 100

#define turnSpeed (-6)

uint8_t preStatus_btn_E;
uint8_t preStatus_btn_Q;
uint8_t startMode;

uint16_t ball5[4] = {330,330,270,530};
uint16_t ball4[4] = {310,310,270,600};
uint16_t ball3[4] = {590,590,270,600};
uint16_t ball2[4] = {888,888,270,600};
uint16_t ball1[4] = {1156,1156,270,600};

uint16_t ball_temp[4] = {0,0,0,0};

void wait4SelectMode()
{
	  ST7920_Clear();
	  brake();
	  HAL_Delay(300);
	  while((btn_Q == 1)&&(btn_E == 1)&&(btn_joyRigt == 1)&&(btn_joyLeft == 1)&&(btn_Sta == 1)&&(btn_W == 1)&&(btn_D == 1)&&(btn_A==1))	//khi chưa chọn vị trí
	  {
		  ST7920_SendString(0,0, "QECZ:Ball");
		  ST7920_SendString(1,0, "W:load");
		  ST7920_SendString(2,0, "Sta:Shoot");
		  freeMotor();
	  }
	  ST7920_Clear();
	  ST7920_SendString(1,0, 	 "      ");
	  ST7920_SendString(1,0, "RUNNING");
	  if(btn_Sta == 0)
	  {
		  startMode = TO_SHOOT;
		  ST7920_SendString(3,0, "         ");
		  ST7920_SendString(3,0, "TO SHOOT");
	  }
	  else if(btn_E == 0)	//lấy ball2
	  {
		  startMode = BALL2;
		  ST7920_SendString(3,0, "         ");
		  ST7920_SendString(3,0, "BALL2");
	  }
	  else if(btn_joyRigt == 0)	//lấy ball3
	  {
		  startMode = BALL3;
		  ST7920_SendString(3,0, "         ");
		  ST7920_SendString(3,0, "BALL3");
	  }
	  else if(btn_joyLeft == 0)	//lấy ball4
	  {
		  startMode = BALL4;
		  ST7920_SendString(3,0, "         ");
		  ST7920_SendString(3,0, "BALL4");
	  }
	  else if(btn_Q == 0)	//lấy ball1
	  {
		  startMode = BALL1;
		  ST7920_SendString(3,0, "         ");
		  ST7920_SendString(3,0, "BALL1");
	  }
	  else if(btn_W == 0)	//về vị trí load ball
	  {
		  startMode = LOAD_BALL;
		  ST7920_SendString(3,0, "         ");
		  ST7920_SendString(3,0, "LOAD BALL");
	  }
	  else if(btn_D == 0)
	  {
		  startMode = MANUAL_MODE;
		  ST7920_SendString(3,0, "         ");
		  ST7920_SendString(3,0, "MANUAL MODE");
	  }
	  else if(btn_A == 0)
	  {
		  startMode = SHOOT_BALL_1;
		  ST7920_SendString(3,0, "         ");
		  ST7920_SendString(3,0, "SHOOT BALL 1");

	  }
	  ST7920_SendString(2,0, "START");
}

void goToBallLeft(uint16_t *ball)
{
	roLKp=0.2;
	roRKp=0.2;
	pitKp=0.2;
	  startTime = HAL_GetTick();
	  if(ball!=ball1)
	  {
		  passArm(PASSARM_DOWN);
		  passHand(PASSHAND_OPEN);
	  }

	  ST7920_Clear();
	  ST7920_SendString(2, 0, "Btn_Sel waiting");
	  ST7920_SendString(3, 0, "autoTuning");
	  while(((abs(leftDistance-ball[_Left])>10)||(abs(pitchDistance-ball[_PitcWait])>10))&&btn_Sel==1)
	  {
		  roL_Pit_Yaw_GoTo(ball[_Left], ball[_PitcWait], 0);
//		  roR_Pit_Yaw_GoTo(ball[_Rigt], ball[_PitcWait], 0);
		  if((abs(leftDistance-ball[_Left])<10)&&(abs(pitchDistance-ball[_PitcWait])<10))
		  {
			  brake();
			  HAL_Delay(DEBOUNCE_MOVING_TIME);
		  }
	  }
	  brake();
	  HAL_Delay(30);
	  freeMotor();
	  passArm(PASSARM_DOWN);
	  passHand(PASSHAND_OPEN);
	  ST7920_Clear();
	  ST7920_SendString(2, 0, "Btn_X waiting");
	  ST7920_SendString(3, 0, "manualTuning");
	  while(btn_X == 1)	//ch�? nhấn nút X -> tinh chỉnh
	  {
		  PIDyaw(compassData, 0);
		  leftVer = !btn_leftUp - !btn_leftDown;
		  leftHor = -!btn_leftLeft + !btn_leftRigt;
		  _dir = atan2(leftHor, -leftVer);
		  _controlSpeed = sqrt(leftVer*leftVer + leftHor*leftHor);
		  _motor1Speed = yawPID*factorYawPID + (factorSpeed*_controlSpeed *cos(3*M_PI/4 - _dir) + 0);
		  _motor2Speed = yawPID*factorYawPID + (factorSpeed*_controlSpeed *cos(3*M_PI/4 + _dir) - 0);
		  _motor3Speed = yawPID*factorYawPID +  factorSpeed*_controlSpeed *cos(  M_PI/4 + _dir) + 0;
		  _motor4Speed = yawPID*factorYawPID +  factorSpeed*_controlSpeed *cos(  M_PI/4 - _dir) - 0;
		  controlMotor1(_motor1Speed);
		  controlMotor2(_motor2Speed);
		  controlMotor3(_motor3Speed);
		  controlMotor4(_motor4Speed);
		  spinalCordTrans();
	  }
	  brake();
	  HAL_Delay(30);
	  freeMotor();
	  passArm(PASSARM_DOWN);
	  passHand(PASSHAND_OPEN);
	  ball_temp[_Left] = leftDistance;
	  startTime = HAL_GetTick();
	  roLKp=0.1;
	roRKp=0.1;
	pitKp=0.1;
	  while(((HAL_GetTick()-startTime)<10000)&&(btn_Sel==1))	//gắp bóng
	  {
		  roL_Pit_Yaw_GoTo(ball_temp[_Left], ball[_Pitc], 0);
	  }
	  brake();
	  HAL_Delay(30);
	  freeMotor();
	  passHand(PASSHAND_CLOSE);	//gắp bóng
	  HAL_Delay(100);
	  ST7920_Clear();
	  ST7920_SendString(2, 0, "btn_D waiting");
	  ST7920_SendString(3, 0, "readyToPass");
	  startTime = HAL_GetTick();
	  while(((HAL_GetTick()-startTime)<100))
	  {
		  controlMotor1(-20);
		  controlMotor2(-20);
		  controlMotor3(20);
		  controlMotor4(20);
		  spinalCordTrans();
	  }
	  brake();
	  HAL_Delay(300);
	  freeMotor();
	  while(zmanualRxPacket[0] != 'D'&&btn_D!=0)	//chờ manual nhấn nút
	  {
		  roL_Pit_Yaw_GoTo(ball_temp[_Left], ball[_PitcWait], 0);
//		  roR_Pit_Yaw_GoTo(ball_temp[_Rigt], ball[_PitcWait], 0);
	  }
	  brake();
	  HAL_Delay(30);
	  freeMotor();
	  passArm(PASSARM_UP);
	  HAL_Delay(1000);
	  passArm(PASSARM_DOWN);
	  passHand(PASSHAND_OPEN);
	  startMode = 0;
	  ball_temp[_Rigt] = 0;
}
void goToBallRigt(uint16_t *ball)
{
	roLKp=0.2;
	roRKp=0.2;
	pitKp=0.2;
	  startTime = HAL_GetTick();
	  if(ball!=ball1)
	  {
		  passArm(PASSARM_DOWN);
		  passHand(PASSHAND_OPEN);
	  }

	  ST7920_Clear();
	  ST7920_SendString(2, 0, "Btn_Sel waiting");
	  ST7920_SendString(3, 0, "autoTuning");
	  while(((abs(rigtDistance-ball[_Rigt])>10)||(abs(pitchDistance-ball[_PitcWait])>10))&&btn_Sel==1)
	  {
		  roR_Pit_Yaw_GoTo(ball[_Rigt], ball[_PitcWait], 0);
//		  roR_Pit_Yaw_GoTo(ball[_Rigt], ball[_PitcWait], 0);
		  if((abs(rigtDistance-ball[_Rigt])<10)&&(abs(pitchDistance-ball[_PitcWait])<10))
		  {
			  brake();
			  HAL_Delay(DEBOUNCE_MOVING_TIME);
		  }
	  }
	  brake();
	  HAL_Delay(30);
	  freeMotor();
	  passArm(PASSARM_DOWN);
	  passHand(PASSHAND_OPEN);
	  ST7920_Clear();
	  ST7920_SendString(2, 0, "Btn_X waiting");
	  ST7920_SendString(3, 0, "manualTuning");
	  while(btn_X == 1)	//ch�? nhấn nút X -> tinh chỉnh
	  {
		  PIDyaw(compassData, 0);
		  leftVer = !btn_leftUp - !btn_leftDown;
		  leftHor = -!btn_leftLeft + !btn_leftRigt;
		  _dir = atan2(leftHor, -leftVer);
		  _controlSpeed = sqrt(leftVer*leftVer + leftHor*leftHor);
		  _motor1Speed = yawPID*factorYawPID + (factorSpeed*_controlSpeed *cos(3*M_PI/4 - _dir) + 0);
		  _motor2Speed = yawPID*factorYawPID + (factorSpeed*_controlSpeed *cos(3*M_PI/4 + _dir) - 0);
		  _motor3Speed = yawPID*factorYawPID +  factorSpeed*_controlSpeed *cos(  M_PI/4 + _dir) + 0;
		  _motor4Speed = yawPID*factorYawPID +  factorSpeed*_controlSpeed *cos(  M_PI/4 - _dir) - 0;
		  controlMotor1(_motor1Speed);
		  controlMotor2(_motor2Speed);
		  controlMotor3(_motor3Speed);
		  controlMotor4(_motor4Speed);
		  spinalCordTrans();
	  }
	  brake();
	  HAL_Delay(30);
	  freeMotor();
	  passArm(PASSARM_DOWN);
	  passHand(PASSHAND_OPEN);
	  ball_temp[_Rigt] = rigtDistance;
	  startTime = HAL_GetTick();
	  roLKp=0.1;
	roRKp=0.1;
	pitKp=0.1;
	  while(((HAL_GetTick()-startTime)<10000)&&(btn_Sel==1))	//gắp bóng
	  {
		  roR_Pit_Yaw_GoTo(ball_temp[_Rigt], ball[_Pitc], 0);
	  }
	  brake();
	  HAL_Delay(30);
	  freeMotor();
	  passHand(PASSHAND_CLOSE);	//gắp bóng
	  HAL_Delay(100);
	  ST7920_Clear();
	  ST7920_SendString(2, 0, "btn_D waiting");
	  ST7920_SendString(3, 0, "readyToPass");
	  startTime = HAL_GetTick();
	  while(((HAL_GetTick()-startTime)<100))
	  {
		  controlMotor1(-20);
		  controlMotor2(-20);
		  controlMotor3(20);
		  controlMotor4(20);
		  spinalCordTrans();
	  }
	  brake();
	  HAL_Delay(300);
	  freeMotor();
	  while(zmanualRxPacket[0] != 'D'&&btn_D!=0)	//chờ manual nhấn nút
	  {
		  roR_Pit_Yaw_GoTo(ball_temp[_Rigt], ball[_PitcWait], 0);
//		  roR_Pit_Yaw_GoTo(ball_temp[_Rigt], ball[_PitcWait], 0);
	  }
	  brake();
	  HAL_Delay(30);
	  freeMotor();
	  passArm(PASSARM_UP);
	  HAL_Delay(1000);
	  passArm(PASSARM_DOWN);
	  passHand(PASSHAND_OPEN);
	  startMode = 0;
	  ball_temp[_Rigt] = 0;
}

void manualMode()
{
	passArm(PASSARM_DOWN);
	passHand(PASSHAND_OPEN);
	while(btn_Sel == 1)
	{
		  PIDyaw(compassData, 0);
		  leftVer = !btn_leftUp - !btn_leftDown;
		  leftHor = -!btn_leftLeft + !btn_leftRigt;
		  _dir = atan2(leftHor, -leftVer);
		  _controlSpeed = sqrt(leftVer*leftVer + leftHor*leftHor);
		  _motor1Speed = joyRigtHor*0.07 + (factorSpeed*_controlSpeed *cos(3*M_PI/4 - _dir) + 0);
		  _motor2Speed = joyRigtHor*0.07 + (factorSpeed*_controlSpeed *cos(3*M_PI/4 + _dir) - 0);
		  _motor3Speed = joyRigtHor*0.07 +  factorSpeed*_controlSpeed *cos(  M_PI/4 + _dir) + 0;
		  _motor4Speed = joyRigtHor*0.07 +  factorSpeed*_controlSpeed *cos(  M_PI/4 - _dir) - 0;
		  controlMotor1(_motor1Speed);
		  controlMotor2(_motor2Speed);
		  controlMotor3(_motor3Speed);
		  controlMotor4(_motor4Speed);
		  spinalCordTrans();
	}
	passHand(PASSHAND_CLOSE);
	while(btn_X == 1)
	{
		  PIDyaw(compassData, 0);
		  leftVer = !btn_leftUp - !btn_leftDown;
		  leftHor = -!btn_leftLeft + !btn_leftRigt;
		  _dir = atan2(leftHor, -leftVer);
		  _controlSpeed = sqrt(leftVer*leftVer + leftHor*leftHor);
		  _motor1Speed = joyRigtHor*0.07 + (factorSpeed*_controlSpeed *cos(3*M_PI/4 - _dir) + 0);
		  _motor2Speed = joyRigtHor*0.07 + (factorSpeed*_controlSpeed *cos(3*M_PI/4 + _dir) - 0);
		  _motor3Speed = joyRigtHor*0.07 +  factorSpeed*_controlSpeed *cos(  M_PI/4 + _dir) + 0;
		  _motor4Speed = joyRigtHor*0.07 +  factorSpeed*_controlSpeed *cos(  M_PI/4 - _dir) - 0;
		  controlMotor1(_motor1Speed);
		  controlMotor2(_motor2Speed);
		  controlMotor3(_motor3Speed);
		  controlMotor4(_motor4Speed);
		  spinalCordTrans();
	}
	while((zmanualRxPacket[0]!='D')&&(btn_D == 1));
	passArm(PASSARM_UP);
	HAL_Delay(1000);
	passArm(PASSARM_DOWN);
	passHand(PASSHAND_OPEN);
}
