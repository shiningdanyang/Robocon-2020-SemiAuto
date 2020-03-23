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
double yawKp = 0.65, yawKd = 0, yawKi = 0;
double yawPID;
#define MAX_YAW_PID 100
#define MIN_YAW_PID -MAX_YAW_PID

int16_t roRError, roRPreError;
double roRP, roRI, roRD;
double roRKp, roRKd, roRKi;
int16_t roRPID;

int16_t roLError, roLPreError;
double roLP, roLI, roLD;
double roLKp, roLKd, roLKi;
int16_t roLPID;

int16_t pitError, pitPreError;
double pitP, pitI, pitD;
double pitKp, pitKd, pitKi;
int16_t pitPID;

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
	(_speed>=0) ? 
	(spinalCordTxPacket[motor1Dir] = ccw) : (spinalCordTxPacket[motor1Dir] = fcw);
	trackingControlMotor1++;
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
	(_speed>=0) ? 
	(spinalCordTxPacket[motor2Dir] = ccw) : (spinalCordTxPacket[motor2Dir] = fcw);
	trackingControlMotor2++;
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
	(_speed>=0) ? 
	(spinalCordTxPacket[motor3Dir] = ccw) : (spinalCordTxPacket[motor3Dir] = fcw);
	trackingControlMotor3++;
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
	(_speed>=0) ? 
	(spinalCordTxPacket[motor4Dir] = ccw) : (spinalCordTxPacket[motor4Dir] = fcw);
	trackingControlMotor4++;
}

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
	roRError = _roRSetpoint - _roRValue;
	roRP = roRError;
	roRD = roRError - roRPreError;
	roRI = roRError + roRI;
	roRPID = roRKp*roRP + roRKd*roRD + roRKi*roRI;
	roRPreError = roRError;
	return roRPID;
}

double PIDroL(int _roLValue, int _roLSetpoint)
{
	roLError = _roLSetpoint - _roLValue;
	roLP = roLError;
	roLD = roLError - roLPreError;
	roLI = roLError + roLI;
	roLPID = roLKp*roLP + roLKd*roLD + roLKi*roLI;
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

// int _motor1Speed = speedPID *sin(desireAngle + 45) + 0;
// int _motor2Speed = speedPID *cos(desireAngle + 45) - 0;
// int _motor3Speed = speedPID *cos(desireAngle + 45) + 0;
// int _motor2Speed = speedPID *sin(desireAngle + 45) - 0;
