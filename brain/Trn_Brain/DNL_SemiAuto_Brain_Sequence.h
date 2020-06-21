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
uint8_t startMode;

uint16_t ball5[4] = {330,330,300,530};
uint16_t ball4[4] = {330,330,300,530};
uint16_t ball3[4] = {610,610,300,530};
uint16_t ball2[4] = {900,900,300,530};
uint16_t ball1[4] = {1215,1215,300,530};

uint16_t ball_temp[4] = {0,0,0,0};

void wait4SelectMode()
{
	  while((btn_Q == 1)&&(btn_E == 1)&&(btn_C == 1)&&(btn_Z == 1)&&(btn_Sta == 1))	//khi chưa chọn vị trí
	  {
		  ST7920_SendString(1,0, "WAITING");
	  }
	  passArm(PASSARM_DOWN);
	  passHand(PASSHAND_OPEN);
	  ST7920_SendString(1,0, 	 "      ");
	  ST7920_SendString(1,0, "RUNNING");
	  if(btn_Sta == 0)
	  	  {
	  		  startMode = TO_SHOOT;
	  		  ST7920_SendString(3,0, "TOSHOOT");
	  	  }
	  else if(btn_E == 0)
	  {
		  startMode = BALL2;
		  ST7920_SendString(3,0, "BALL2");
	  }
	  else if(btn_C == 0)
	  {
		  startMode = BALL3;
		  ST7920_SendString(3,0, "BALL3");
	  }
	  else if(btn_Z == 0)
	  {
		  startMode = BALL4;
		  ST7920_SendString(3,0, "BALL4");
	  }
	  else if(btn_Q == 0)	//chế độ khởi động ban đầu
	  {
		  startMode = BALL1;
		  ST7920_SendString(3,0, "BALL1");
	  }
	  ST7920_SendString(2,0, "START");
}

void goToBallRigt(uint16_t *ball)
{
	  startTime = HAL_GetTick();
	  while(btn_Sel!=0)	//di chuyển tới vị trí BALL1
	  {
		  roR_Pit_Yaw_GoTo(ball[_Rigt], ball[_PitcWait], 0);
	  }
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
	  ball_temp[_Rigt] = rigtDistance;
	  startTime = HAL_GetTick();
	  while(HAL_GetTick()-startTime < 1000)	//di chuyển vào vị trí lấy ball1
	  {
		  roR_Pit_Yaw_GoTo(ball_temp[_Rigt], ball[_Pitc], 0);
	  }
	  brake();
//	  HAL_Delay(500);
	  passHand(PASSHAND_CLOSE);	//gắp bóng
	  HAL_Delay(100);
	  while(zmanualRxPacket[0] != 'D'&&btn_D!=0)	//chờ manual nhấn nút
	  {
		  roR_Pit_Yaw_GoTo(ball_temp[_Rigt], ball[_PitcWait], 0);
	  }
	  brake();
	  passArm(PASSARM_UP);
	  HAL_Delay(1000);
	  passArm(PASSARM_DOWN);
	  passHand(PASSHAND_OPEN);
	  startMode = 0;
	  ball_temp[_Rigt] = 0;
}
