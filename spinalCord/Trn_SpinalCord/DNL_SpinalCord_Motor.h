//include đầu tiên ở "USER CODE BEGIN 0"
#include "math.h"
#define PI 3.14159265

int16_t yawError, yawPreError;
double yawP, yawI, yawD;
double yawKp = 0.55, yawKd = 0.0, yawKi = 0.0;//0.7//0.6//0.6
double yawPID;
#define MAX_YAW_PID 100 // 50// 70 //85
#define MIN_YAW_PID -MAX_YAW_PID//-50//-70
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

#define deg2Rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define rad2Deg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

#define motor1 htim8
#define motor2 htim8
#define motor3 htim1
#define motor4 htim1
#define motor1_channel TIM_CHANNEL_1
#define motor2_channel TIM_CHANNEL_2
#define motor3_channel TIM_CHANNEL_3
#define motor4_channel TIM_CHANNEL_4

#define FCW GPIO_PIN_RESET
#define CCW GPIO_PIN_SET

#define _FCW 0
#define _CCW 1


// float motor1Speed;
// float motor2Speed;
// float motor3Speed;
// float motor4Speed;
// int motor1Dir;
// int motor2Dir;
// int motor3Dir;
// int motor4Dir;
// int receiveCplt;

uint8_t motor1Speed;
uint8_t motor2Speed;
uint8_t motor3Speed;
uint8_t motor4Speed;
int8_t motor1Dir;
int8_t motor2Dir;
int8_t motor3Dir;
int8_t motor4Dir;

void peripheralPWM_Init()
{
//  HAL_TIM_PWM_Start(&motor1, motor1_channel);
//  HAL_TIM_PWM_Start(&motor2, motor2_channel);
//  HAL_TIM_PWM_Start(&motor3, motor3_channel);
//  HAL_TIM_PWM_Start(&motor4, motor4_channel);
  HAL_TIM_PWM_Start(&motor1, motor1_channel);
  HAL_TIM_PWM_Start(&motor2, motor2_channel);
  HAL_TIM_PWM_Start(&motor3, motor3_channel);
  HAL_TIM_PWM_Start(&motor4, motor4_channel);
}

void controlMotor1(int _speed, int _dir)
{
	HAL_GPIO_WritePin(motor1Dir_GPIO_Port, motor1Dir_Pin, _dir);
	__HAL_TIM_SetCompare(&motor1, motor1_channel, _speed);
}
void controlMotor2(int _speed, int _dir)
{
	HAL_GPIO_WritePin(motor2Dir_GPIO_Port, motor2Dir_Pin, _dir);
	__HAL_TIM_SetCompare(&motor2, motor2_channel, _speed);
}
void controlMotor3(int _speed, int _dir)
{
	HAL_GPIO_WritePin(motor3Dir_GPIO_Port, motor3Dir_Pin, _dir);
	__HAL_TIM_SetCompare(&motor3, motor3_channel, _speed);
}
void controlMotor4(int _speed, int _dir)
{
	HAL_GPIO_WritePin(motor4Dir_GPIO_Port, motor4Dir_Pin, _dir);
	__HAL_TIM_SetCompare(&motor4, motor4_channel, _speed);
}

void testPWM()
{
	for(i = 0; i > -250; --i)
	{
		controlMotor1(abs(i), CCW);
		controlMotor2(abs(i), CCW);
		controlMotor3(abs(i), CCW);
		controlMotor4(abs(i), CCW);
		HAL_Delay(20);
		tracking++;
	}
	for(i = -250; i < 0; ++i)
	{
		controlMotor1(abs(i), CCW);
		controlMotor2(abs(i), CCW);
		controlMotor3(abs(i), CCW);
		controlMotor4(abs(i), CCW);
		HAL_Delay(20);
		tracking++;
	}
	for(i = 0; i < 250; ++i)
	{
		controlMotor1(abs(i), FCW);
		controlMotor2(abs(i), FCW);
		controlMotor3(abs(i), FCW);
		controlMotor4(abs(i), FCW);
		HAL_Delay(20);
		tracking++;
	}
	for(i = 250; i > 0; --i)
	{
		controlMotor1(abs(i), FCW);
		controlMotor2(abs(i), FCW);
		controlMotor3(abs(i), FCW);
		controlMotor4(abs(i), FCW);
		HAL_Delay(20);
		tracking++;
	}
}
//void findOrientation(int _speed, float _dir)
//{
//	motor1Speed = -(_speed *sin(_dir + 0.7854) + 0);
//	motor2Speed = -(_speed *cos(_dir + 0.7854) - 0);
//	motor3Speed =   _speed *sin(_dir + 0.7854) - 0;
//	motor4Speed =   _speed *cos(_dir + 0.7854) + 0;
//}

// void findOrientation_rad(int _speed, double _dir_rad)
// {
// 	motor1Speed = -(_speed *sin(-_dir_rad + M_PI/4) + 0);
// 	motor2Speed = -(_speed *cos(-_dir_rad + M_PI/4) - 0);
// 	motor3Speed =   _speed *sin(-_dir_rad + M_PI/4) - 0;
// 	motor4Speed =   _speed *cos(-_dir_rad + M_PI/4) + 0;
// }

// void findOrientation_deg(int _speed, double _dir_deg)
// {
// 	motor1Speed = -(_speed *sin(-deg2Rad(_dir_deg) + M_PI/4) + 0);
// 	motor2Speed = -(_speed *cos(-deg2Rad(_dir_deg) + M_PI/4) - 0);
// 	motor3Speed =   _speed *sin(-deg2Rad(_dir_deg) + M_PI/4) - 0;
// 	motor4Speed =   _speed *cos(-deg2Rad(_dir_deg) + M_PI/4) + 0;
// }

// void controlAllMotor()
// {
// 	(motor1Speed>=0)?
// 			HAL_GPIO_WritePin(motor1Dir_GPIO_Port, motor1Dir_Pin, CCW)
// 	:
// 			HAL_GPIO_WritePin(motor1Dir_GPIO_Port, motor1Dir_Pin, FCW);
// 	(motor2Speed>=0)?
// 			HAL_GPIO_WritePin(motor2Dir_GPIO_Port, motor2Dir_Pin, CCW)
// 	:
// 			HAL_GPIO_WritePin(motor2Dir_GPIO_Port, motor2Dir_Pin, FCW);
// 	(motor3Speed>=0)?
// 			HAL_GPIO_WritePin(motor3Dir_GPIO_Port, motor3Dir_Pin, CCW)
// 	:
// 			HAL_GPIO_WritePin(motor3Dir_GPIO_Port, motor3Dir_Pin, FCW);
// 	(motor4Speed>=0)?
// 			HAL_GPIO_WritePin(motor4Dir_GPIO_Port, motor4Dir_Pin, CCW)
// 	:
// 			HAL_GPIO_WritePin(motor4Dir_GPIO_Port, motor4Dir_Pin, FCW);

// 	  __HAL_TIM_SetCompare(&motor1, TIM_CHANNEL_1, abs((int)motor1Speed));
// 	  __HAL_TIM_SetCompare(&motor2, TIM_CHANNEL_1, abs((int)motor2Speed));
// 	  __HAL_TIM_SetCompare(&motor3, TIM_CHANNEL_1, abs((int)motor3Speed));
// 	  __HAL_TIM_SetCompare(&motor4, TIM_CHANNEL_1, abs((int)motor4Speed));
// }
