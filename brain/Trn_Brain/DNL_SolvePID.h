//include sau ADC.h
#include "math.h"

#define PI 3.14159265
#define deg2Rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define rad2Deg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

#define motor1 
#define motor2
#define motor3
#define motor4
#define motor1_channel
#define motor2_channel
#define motor3_channel
#define motor4_channel
// #define motor1Dir
// #define motor2Dir
// #define motor3Dir
// #define motor4Dir
#define ccw 0
#define fcw 1

int16_t yawError, yawPreError;
double yawP, yawI, yawD;
double yawKp = 0.5;
double yawKd = 0;
double yawKi = 0.0000000001;
double yawPID;
#define MAX_YAW_PID 120
#define MIN_YAW_PID -MAX_YAW_PID

int16_t roRError, roRPreError;
double roRP, roRI, roRD;
double roRKp;
double roRKd;
double roRKi;
double roRPID;
#define MAX_ROR_PID 100
#define MIN_ROR_PID -MAX_ROR_PID

int16_t roLError, roLPreError;
double roLP, roLI, roLD;
double roLKp = 5;
double roLKd;
double roLKi;
double roLPID;
#define MAX_ROL_PID 50
#define MIN_ROL_PID -MAX_ROL_PID

double pitError, pitPreError;
double pitP, pitI, pitD;
double pitKp = 0.3;
double pitKd;
double pitKi;
double pitPID;
#define MAX_PIT_PID 100
#define MIN_PIT_PID -MAX_PIT_PID

void controlMotor1(int _speed);
void controlMotor2(int _speed);
void controlMotor3(int _speed);
void controlMotor4(int _speed);
double PIDyaw(int _yawValue, int _yawSetpoint);
double PIDroR(int _roRValue, int _roRSetpoint);
double PIDroL(int _roLValue, int _roLSetpoint);


int trackingControlMotor1;
int trackingControlMotor2;
int trackingControlMotor3;
int trackingControlMotor4;

#define BRAKE_SPEED 1

void controlMotor1(int _speed)
{
	// __HAL_TIM_Set_Compare(&motor1, motor1_channel, _speed);
	// if(_speed >= 0)
	// {
	// 	HAL_GPIO_WritePin(motor1Dir_Pin, motor1Dir_GPIO_Port, ccw);
	// }
	// else
	// {
	// 	HAL_GPIO_WritePin(motor1Dir_Pin, motor1Dir_GPIO_Port, fcw);
	// }
	spinalCordTxPacket[motor1Speed] = abs(_speed);
	if(_speed>=0)
//	(spinalCordTxPacket[motor1Dir] = ccw) : (spinalCordTxPacket[motor1Dir] = fcw);
		spinalCordTxPacket[motorDir] &= ~(1UL << 0);
	else
		spinalCordTxPacket[motorDir] |= (1UL << 0);
//	trackingControlMotor1++;
}
void controlMotor2(int _speed)
{
	// __HAL_TIM_Set_Compare(&motor2, motor2_channel, _speed);
	// if(_speed >= 0)
	// {
	// 	HAL_GPIO_WritePin(motor2Dir_Pin, motor2Dir_GPIO_Port, ccw);
	// }
	// else
	// {
	// 	HAL_GPIO_WritePin(motor2Dir_Pin, motor2Dir_GPIO_Port, fcw);
	// }
	spinalCordTxPacket[motor2Speed] = abs(_speed);
	if(_speed>=0)
		spinalCordTxPacket[motorDir] &= ~(1UL << 1);
	else
		spinalCordTxPacket[motorDir] |= (1UL << 1);
//	trackingControlMotor2++;
}
void controlMotor3(int _speed)
{
	// __HAL_TIM_Set_Compare(&motor3Dir, motor3_channel, _speed);
	// if(_speed >= 0)
	// {
	// 	HAL_GPIO_WritePin(motor3Dir_Pin, motor3Dir_GPIO_Port, ccw);
	// }
	// else
	// {
	// 	HAL_GPIO_WritePin(motor3Dir_Pin, motor3Dir_GPIO_Port, fcw);
	// }
	spinalCordTxPacket[motor3Speed] = abs(_speed);
	if(_speed>=0)
		spinalCordTxPacket[motorDir] &= ~(1UL << 2);
	else
		spinalCordTxPacket[motorDir] |= (1UL << 2);
//	trackingControlMotor3++;
}
void controlMotor4(int _speed)
{
	// __HAL_TIM_Set_Compare(&motor4, motor4_channel, _speed);
	// if(_speed >= 0)
	// {
	// 	HAL_GPIO_WritePin(motor3Dir_Pin, motor3Dir_GPIO_Port, ccw);
	// }
	// else
	// {
	// 	HAL_GPIO_WritePin(motor3Dir_Pin, motor3Dir_GPIO_Port, fcw);
	// }
	spinalCordTxPacket[motor4Speed] = abs(_speed);
	if(_speed>=0)
		spinalCordTxPacket[motorDir] &= ~(1UL << 3);
	else
		spinalCordTxPacket[motorDir] |= (1UL << 3);
//	trackingControlMotor4++;
}

double PIDyaw(int _yawValue, int _yawSetpoint)
{
	yawError = -_yawSetpoint + _yawValue;
	yawP = yawError;
	yawD = yawError - yawPreError;
	yawI = yawError + yawI;
	yawPID = yawKp*yawP + yawKd*yawD + yawKi*yawI;
	if(yawPID > MAX_YAW_PID)
	{
		yawPID = MAX_YAW_PID;
	}
	if(yawPID < MIN_YAW_PID)
	{
		yawPID = MIN_YAW_PID;
	}
	yawPreError = yawError;
	return yawPID;
}

double PIDroR(int _roRValue, int _roRSetpoint)
{
	roRError = _roRSetpoint - _roRValue;
	roRP = roRError;
	roRD = roRError - roRPreError;
	roRI = roRError + roRI;
	roRPID = roRKp*roRP + roRKd*roRD + roRKi*roRI;
	if(roRPID > MAX_ROR_PID)
	{
		roRPID = MAX_ROR_PID;
	}
	if(roRPID < MIN_ROR_PID)
	{
		roRPID = MIN_ROR_PID;
	}
	roRPreError = roRError;
	return roRPID;
}

double PIDroL(int _roLValue, int _roLSetpoint)
{
	roLError = -_roLSetpoint + _roLValue;
	roLP = roLError;
	roLD = roLError - roLPreError;
	roLI = roLError + roLI;
	roLPID = roLKp*roLP + roLKd*roLD + roLKi*roLI;
	if(roLPID > MAX_ROL_PID)
	{
		roLPID = MAX_ROL_PID;
	}
	if(roLPID < MIN_ROL_PID)
	{
		roLPID = MIN_ROL_PID;
	}
	roLPreError = roLError;
	return roLPID;
}

double PIDpit(int _pitValue, int _pitSetpoint)
{
	pitError = _pitSetpoint - _pitValue;
	pitP = pitError;
	pitD = pitError - pitPreError;
	pitI = pitError + pitI;
	pitPID = pitKp*pitP + pitKd*pitD + pitKi*pitI;
	if(pitPID > MAX_PIT_PID)
	{
		pitPID = MAX_PIT_PID;
	}
	if(pitPID < MIN_PIT_PID)
	{
		pitPID = MIN_PIT_PID;
	}
	pitPreError = pitError;
	return pitPID;
}


void testPIDyawMoving(int _yawValue, int _yawSetpoint)
{
	PIDyaw(_yawValue, _yawSetpoint);
	controlMotor1(yawPID);
	controlMotor2(yawPID);
	controlMotor3(yawPID);
	controlMotor4(yawPID);
	// spinalCordTrans();
}

void roL_pit_yaw_mixSpeed(void)
{
	double _roL_pit_speed = sqrt(roLPID*roLPID + pitPID*pitPID);
	double _roL_pit_dir = atan2(pitPID, roLPID);
	double _motor1Speed = yawPID + -(_roL_pit_speed *sin(_roL_pit_dir + M_PI/4) + 0);
	double _motor2Speed = yawPID + -(_roL_pit_speed *cos(_roL_pit_dir + M_PI/4) - 0);
	double _motor3Speed = yawPID +   _roL_pit_speed *sin(_roL_pit_dir + M_PI/4) + 0;
	double _motor4Speed = yawPID +   _roL_pit_speed *cos(_roL_pit_dir + M_PI/4) - 0;
	controlMotor1(_motor1Speed);
	controlMotor2(_motor2Speed);
	controlMotor3(_motor3Speed);
	controlMotor4(_motor4Speed);
//	spinalCordTrans();
}

void roR_pit_yaw_mixSpeed(void)
{
	double _roR_pit_speed = sqrt(roRPID*roRPID + pitPID*pitPID);
	double _roR_pit_dir = atan2(pitPID, roRPID);
	double _motor1Speed = yawPID + -(_roR_pit_speed *sin(_roR_pit_dir + M_PI/4) + 0);
	double _motor2Speed = yawPID + -(_roR_pit_speed *cos(_roR_pit_dir + M_PI/4) - 0);
	double _motor3Speed = yawPID +   _roR_pit_speed *sin(_roR_pit_dir + M_PI/4) + 0;
	double _motor4Speed = yawPID +   _roR_pit_speed *cos(_roR_pit_dir + M_PI/4) - 0;
	controlMotor1(_motor1Speed);
	controlMotor1(_motor2Speed);
	controlMotor1(_motor3Speed);
	controlMotor1(_motor4Speed);
//	spinalCordTrans();
}

int constantMoving(int _speed, double _dir_deg)
{
	controlMotor1(-(_speed *sin(-deg2Rad(_dir_deg) + M_PI/4) + 0));
	controlMotor2(-(_speed *cos(-deg2Rad(_dir_deg) + M_PI/4) - 0));
	controlMotor3(  _speed *sin(-deg2Rad(_dir_deg) + M_PI/4) - 0);
	controlMotor4(  _speed *cos(-deg2Rad(_dir_deg) + M_PI/4) + 0);
	return 1;
}

void rotate(int _dir)
{
	if(_dir == 0)
	{
		controlMotor1(30);
		controlMotor2(30);
		controlMotor3(30);
		controlMotor4(30);
	}
	else
		if(_dir == 1)
		{
			controlMotor1(-30);
			controlMotor2(-30);
			controlMotor3(-30);
			controlMotor4(-30);
		}
}

void brake(void)
{
	controlMotor1(BRAKE_SPEED);
	controlMotor2(-BRAKE_SPEED);
	controlMotor3(BRAKE_SPEED);
	controlMotor4(-BRAKE_SPEED);
	spinalCordTrans();
}

void testPWM(void)
{
  for(int i = 0; i > -255; --i)
  {
	  controlMotor1(i);
	  controlMotor2(i);
	  controlMotor3(i);
	  controlMotor4(i);
	  HAL_Delay(20);
  }
  for(int i = -255; i < 0; ++i)
  {
	  controlMotor1(i);
	  controlMotor2(i);
	  controlMotor3(i);
	  controlMotor4(i);
	  HAL_Delay(20);
  }
  for(int i = 0; i < 255; ++i)
  {
	  controlMotor1(i);
	  controlMotor2(i);
	  controlMotor3(i);
	  controlMotor4(i);
	  HAL_Delay(20);
  }
  for(int i = 255; i > 0; --i)
  {
	  controlMotor1(i);
	  controlMotor2(i);
	  controlMotor3(i);
	  controlMotor4(i);
	  HAL_Delay(20);
  }
}

// int _motor1Speed = speedPID *sin(desireAngle + 45) + 0;
// int _motor2Speed = speedPID *cos(desireAngle + 45) - 0;
// int _motor3Speed = speedPID *cos(desireAngle + 45) + 0;
// int _motor2Speed = speedPID *sin(desireAngle + 45) - 0;
