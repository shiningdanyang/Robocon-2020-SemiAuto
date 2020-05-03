//include sau ADC.h
#include "math.h"

#define PI 3.14159265
#define deg2Rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define rad2Deg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

#define CCW 0
#define FCW 1

#define BRAKE_SPEED 3

//#define MAX_YAW_PID 120
#define MAX_YAW_PID 120
#define MIN_YAW_PID -MAX_YAW_PID
int16_t yawError, yawPreError;
double yawP, yawI, yawD;
double yawKp = 0.5;
double yawKd = 0;
double yawKi = 0.0000000001;
double yawPID;

#define MAX_ROR_PID 100
#define MIN_ROR_PID -MAX_ROR_PID
int16_t roRError, roRPreError;
double roRP, roRI, roRD;
double roRKp = 5;
double roRKd;
double roRKi;
double roRPID;

#define MAX_ROL_PID 100
#define MIN_ROL_PID -MAX_ROL_PID
int16_t roLError, roLPreError;
double roLP, roLI, roLD;
double roLKp = 5;
double roLKd;
double roLKi;
double roLPID;

#define MAX_PIT_PID 100
#define MIN_PIT_PID -MAX_PIT_PID
double pitError, pitPreError;
double pitP, pitI, pitD;
double pitKp = 5;
double pitKd;
double pitKi;
double pitPID;

double motor1Speed_;
double motor2Speed_;
double motor3Speed_;
double motor4Speed_;

void controlMotor1(int _speed);
void controlMotor2(int _speed);
void controlMotor3(int _speed);
void controlMotor4(int _speed);
double PIDyaw(int _yawValue, int _yawSetpoint);
double PIDroR(int _roRValue, int _roRSetpoint);
double PIDroL(int _roLValue, int _roLSetpoint);
double PIDpit(int _pitValue, int _pitSetpoint);
void testPWM(void);
void roL_pit_yaw_mixSpeed(void);
void roR_pit_yaw_mixSpeed(void);
void brake(void);

#ifdef SPINAL_CORD_MODE_ONEWAY
void controlMotor1(int _speed)
{
	spinalCordTxPacket[motor1Speed] = abs(_speed);
	if(_speed>=0)
		spinalCordTxPacket[motorDir] &= ~(1UL << 0);
//		spinalCordTxPacket[motor1Dir] = 0;
	else
		spinalCordTxPacket[motorDir] |= (1UL << 0);
//		spinalCordTxPacket[motor1Dir] = 1;
}
void controlMotor2(int _speed)
{
	spinalCordTxPacket[motor2Speed] = abs(_speed);
	if(_speed>=0)
		spinalCordTxPacket[motorDir] &= ~(1UL << 1);
//		spinalCordTxPacket[motor2Dir] = 0;
	else
		spinalCordTxPacket[motorDir] |= (1UL << 1);
//		spinalCordTxPacket[motor2Dir] = 1;
}
void controlMotor3(int _speed)
{
	spinalCordTxPacket[motor3Speed] = abs(_speed);
	if(_speed>=0)
		spinalCordTxPacket[motorDir] &= ~(1UL << 2);
//		spinalCordTxPacket[motor3Dir] = 0;
	else
		spinalCordTxPacket[motorDir] |= (1UL << 2);
//		spinalCordTxPacket[motor3Dir] = 1;
}
void controlMotor4(int _speed)
{
	spinalCordTxPacket[motor4Speed] = abs(_speed);
	if(_speed>=0)
		spinalCordTxPacket[motorDir] &= ~(1UL << 3);
//		spinalCordTxPacket[motor4Dir] = 0;
	else
		spinalCordTxPacket[motorDir] |= (1UL << 3);
//		spinalCordTxPacket[motor4Dir] = 1;
}
void testPWM(void)
{
  for(int i = -1; i > -255; --i)
  {
	  controlMotor1(i);
	  controlMotor2(i);
	  controlMotor3(i);
	  controlMotor4(i);
	  PS2TxPacket[0]=spinalCordTxPacket[motor1Dir]+65;
	  PS2TxPacket[1]=spinalCordTxPacket[motor2Dir]+65;
	  PS2TxPacket[2]=spinalCordTxPacket[motor3Dir]+65;
	  PS2TxPacket[3]=spinalCordTxPacket[motor4Dir]+65;
	  PS2TxPacket[4]= '\n';
	  spinalCordTrans();
	  HAL_UART_Transmit(&PS2, PS2TxPacket, 5, 50);
	  HAL_Delay(20);
  }
  for(int i = -255; i < 0; ++i)
  {
	  controlMotor1(i);
	  controlMotor2(i);
	  controlMotor3(i);
	  controlMotor4(i);
	  PS2TxPacket[0]=spinalCordTxPacket[motor1Dir]+65;
	  PS2TxPacket[1]=spinalCordTxPacket[motor2Dir]+65;
	  PS2TxPacket[2]=spinalCordTxPacket[motor3Dir]+65;
	  PS2TxPacket[3]=spinalCordTxPacket[motor4Dir]+65;
	  PS2TxPacket[4]= '\n';
	  spinalCordTrans();
	  HAL_UART_Transmit(&PS2, PS2TxPacket, 5, 50);
	  HAL_Delay(20);
  }
  for(int i = 1 ; i < 255; ++i)
  {
	  controlMotor1(i);
	  controlMotor2(i);
	  controlMotor3(i);
	  controlMotor4(i);
	  PS2TxPacket[0]=spinalCordTxPacket[motor1Dir]+65;
	  PS2TxPacket[1]=spinalCordTxPacket[motor2Dir]+65;
	  PS2TxPacket[2]=spinalCordTxPacket[motor3Dir]+65;
	  PS2TxPacket[3]=spinalCordTxPacket[motor4Dir]+65;
	  PS2TxPacket[4]= '\n';
	  spinalCordTrans();
	  HAL_UART_Transmit(&PS2, PS2TxPacket, 5, 50);
	  HAL_Delay(20);
  }
  for(int i = 255; i > 0; --i)
  {
	  controlMotor1(i);
	  controlMotor2(i);
	  controlMotor3(i);
	  controlMotor4(i);
	  PS2TxPacket[0]=spinalCordTxPacket[motor1Dir]+65;
	  PS2TxPacket[1]=spinalCordTxPacket[motor2Dir]+65;
	  PS2TxPacket[2]=spinalCordTxPacket[motor3Dir]+65;
	  PS2TxPacket[3]=spinalCordTxPacket[motor4Dir]+65;
	  PS2TxPacket[4]= '\n';
	  spinalCordTrans();
	  HAL_UART_Transmit(&PS2, PS2TxPacket, 5, 50);
	  HAL_Delay(20);
  }
}
#endif
#ifndef SPINAL_CORD_MODE_ONEWAY
void controlMotor1(int _speed)
{
	spinalCordTxPacket[motor1Speed] = abs(_speed);
	if(_speed>=0)
		(spinalCordTxPacket[motor1Dir] = CCW);
	else
		(spinalCordTxPacket[motor1Dir] = FCW);
}
void controlMotor2(int _speed)
{
	spinalCordTxPacket[motor2Speed] = abs(_speed);
	if(_speed>=0)
		(spinalCordTxPacket[motor2Dir] = CCW);
	else
		(spinalCordTxPacket[motor2Dir] = FCW);
}
void controlMotor3(int _speed)
{
	spinalCordTxPacket[motor3Speed] = abs(_speed);
	if(_speed>=0)
		(spinalCordTxPacket[motor3Dir] = CCW);
	else
		(spinalCordTxPacket[motor3Dir] = FCW);
}
void controlMotor4(int _speed)
{
	spinalCordTxPacket[motor4Speed] = abs(_speed);
	if(_speed>=0)
		(spinalCordTxPacket[motor4Dir] = CCW);
	else
		(spinalCordTxPacket[motor4Dir] = FCW);
}
void testPWM(void)
{
  for(int i = -1; i > -255; --i)
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
  for(int i = 1 ; i < 255; ++i)
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
#endif

double PIDyaw(int _yawValue, int _yawSetpoint)
{
	yawError = _yawSetpoint - _yawValue;
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
	roRError = -_roRSetpoint + _roRValue;
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
	pitError = -_pitSetpoint + _pitValue;
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
	double _roL_pit_dir = atan2(roLPID, pitPID);
//	double _motor1Speed = yawPID + -(_roL_pit_speed *sin(_roL_pit_dir + M_PI/4) + 0);
//	double _motor2Speed = yawPID + -(_roL_pit_speed *cos(_roL_pit_dir + M_PI/4) - 0);
//	double _motor3Speed = yawPID +   _roL_pit_speed *sin(_roL_pit_dir + M_PI/4) + 0;
//	double _motor4Speed = yawPID +   _roL_pit_speed *cos(_roL_pit_dir + M_PI/4) - 0;

	double _motor1Speed = yawPID + (_roL_pit_speed *cos(3.0*M_PI/4.0 - _roL_pit_dir) + 0.0);
	double _motor2Speed = yawPID + (_roL_pit_speed *cos(3.0*M_PI/4.0 + _roL_pit_dir) - 0.0);
	double _motor3Speed = yawPID +  _roL_pit_speed *cos(    M_PI/4.0 + _roL_pit_dir) + 0.0;
	double _motor4Speed = yawPID +  _roL_pit_speed *cos(    M_PI/4.0 - _roL_pit_dir) - 0.0;
	a = 1.0 - _motor1Speed;
	b = 1.0 - _motor3Speed;
	absSpeed = _roL_pit_speed;
	dir = _roL_pit_dir;
	cos_ = cos(3*M_PI/4 - _roL_pit_dir);
	motor1_debug = absSpeed*cos_;
	motor1Speed_ = _motor1Speed;
	motor2Speed_ = _motor2Speed;
	motor3Speed_ = _motor3Speed;
	motor4Speed_ = _motor4Speed;

	controlMotor1(_motor1Speed);
	controlMotor2(_motor2Speed);
	controlMotor3(_motor3Speed);
	controlMotor4(_motor4Speed);
	spinalCordTrans();
}

void roR_pit_yaw_mixSpeed(void)
{
	double _roR_pit_speed = sqrt(roRPID*roRPID + pitPID*pitPID);
	double _roR_pit_dir = atan2(-roRPID, pitPID);
//	double _motor1Speed = yawPID + -(_roR_pit_speed *sin(_roR_pit_dir + M_PI/4) + 0);
//	double _motor2Speed = yawPID + -(_roR_pit_speed *cos(_roR_pit_dir + M_PI/4) - 0);
//	double _motor3Speed = yawPID +   _roR_pit_speed *sin(_roR_pit_dir + M_PI/4) + 0;
//	double _motor4Speed = yawPID +   _roR_pit_speed *cos(_roR_pit_dir + M_PI/4) - 0;

	double _motor1Speed = yawPID + (_roR_pit_speed *cos(3.0*M_PI/4.0 - _roR_pit_dir) + 0.0);
	double _motor2Speed = yawPID + (_roR_pit_speed *cos(3.0*M_PI/4.0 + _roR_pit_dir) - 0.0);
	double _motor3Speed = yawPID +  _roR_pit_speed *cos(    M_PI/4.0 + _roR_pit_dir) + 0.0;
	double _motor4Speed = yawPID +  _roR_pit_speed *cos(    M_PI/4.0 - _roR_pit_dir) - 0.0;

	a = 1.0 - _motor1Speed;
	b = 1.0 - _motor3Speed;
	absSpeed = _roR_pit_speed;
	dir = _roR_pit_dir;
	cos_ = cos(3*M_PI/4 - _roR_pit_dir);
	motor1_debug = absSpeed*cos_;
	motor1Speed_ = _motor1Speed;
	motor2Speed_ = _motor2Speed;
	motor3Speed_ = _motor3Speed;
	motor4Speed_ = _motor4Speed;

	controlMotor1(_motor1Speed);
	controlMotor2(_motor2Speed);
	controlMotor3(_motor3Speed);
	controlMotor4(_motor4Speed);
	spinalCordTrans();
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
