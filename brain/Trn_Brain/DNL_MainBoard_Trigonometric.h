#define motor1 
#define motor2
#define motor3
#define motor4
#define ccw 0
#define fcw 1

int yawError, yawPreError;
int yawP, yawI, yawD;
int yawKp, yawKd, yawKi;

int roRError, roRPreError;
int roRP, roRI, roRD;
int roRKp, roRKd, roRKi;

int roLError, roRPreError;
int roLP, roLI, roLD;
int roLKp, roLKd, roLKi;

int pitError, pitPreError;
int pitP, pitI, pitD;
int pitKp, pitKp, pitKil;

void controlMotor1(int _speed, int _dir)
{
	__HAL_TIM_Set_Compare(&motor1, motor1_channel, _speed);
	HAL_GPIO_WritePin(motor1_Pin, motor1_GPIO_Port, _dir);
}
void controlMotor2(int _speed, int _dir)
{
	__HAL_TIM_Set_Compare(&motor1, motor1_channel, _speed);
	HAL_GPIO_WritePin(motor1_Pin, motor1_GPIO_Port, _dir);
}
void controlMotor3(int _speed, int _dir)
{
	__HAL_TIM_Set_Compare(&motor1, motor1_channel, _speed);
	HAL_GPIO_WritePin(motor1_Pin, motor1_GPIO_Port, _dir);
}
void controlMotor4(int _speed, int _dir)
{
	__HAL_TIM_Set_Compare(&motor1, motor1_channel, _speed);
	HAL_GPIO_WritePin(motor1_Pin, motor1_GPIO_Port, _dir);
}

int PIDyaw(int _yawValue, int _yawSetpoint)
{
	yawError = _yawSetpoint - _yawValue;
	yawP = yawError;
	yawD = yawError - yawPreError;
	yawI = yawError + yawI;
	yawPID = yawKp*yawP + yawKd*yawD + yawKi*yawI;
	yawPreError = yawError;
	return yawPID;
}

int PIDroR(int _roRValue, int _roRSetpoint)
{
	roRError = _roRSetpoint - _roRValue;
	roRP = roRError;
	roRD = roRError - roRPreError;
	roRI = roRError + roRI;
	roRPID = roRKp*roRP + roRKd*roRD + roRKi*roRI;
	roRPreError = roRError;
	return roRPID;
}

int PIDroL(int _roLValue, int _roLSetpoint)
{
	roLError = _roLSetpoint - _roLValue;
	roLP = roLError;
	roLD = roLError - roLPreError;
	roLI = roLError + roLI;
	roLPID = roLKp*roLP + roLKd*roLD + roLKi*roLI;
	roLPreError = roLError;
	return roLPID;
}

int PIDpit(int _pitValue, int _pitSetpoint)
{
	pitError = _pitSetpoint - _pitValue;
	pitP = pitError;
	pitD = pitError - pitPreError;
	pitI = pitError + pitI;
	pitPID = pitKp*pitP + pitKd*pitD + pitKi*pitI;
	pitPreError = pitError;
	return pitPID;
}

int roL_pit_yaw_mixSpeed()
{
	int _roL_pit_speed = sqrt(roLPID*roLPID + pitPID*pitPID);
	int _roL_pit_dir = atan2(pitPID, roLPID);
	int _motor1Speed = yawPID + -(_roL_pit_speed *sin(_roL_pit_dir + 45) + 0);
	int _motor2Speed = yawPID + -(_roL_pit_speed *cos(_roL_pit_dir + 45) - 0);
	int _motor3Speed = yawPID +   _roL_pit_speed *cos(_roL_pit_dir + 45) + 0;
	int _motor2Speed = yawPID +   _roL_pit_speed *sin(_roL_pit_dir + 45) - 0;
}

int roR_pit_yaw_mixSpeed()
{
	int _roR_pit_speed = sqrt(roRPID*roRPID + pitPID*pitPID);
	float _roR_pit_dir = atan2(pitPID, roRPID)*180/3.1412;
	int _motor1Speed = yawPID + -(_roR_pit_speed *sin(_roR_pit_dir + 45) + 0);
	int _motor2Speed = yawPID + -(_roR_pit_speed *cos(_roR_pit_dir + 45) - 0);
	int _motor3Speed = yawPID +   _roR_pit_speed *cos(_roR_pit_dir + 45) + 0;
	int _motor2Speed = yawPID +   _roR_pit_speed *sin(_roR_pit_dir + 45) - 0;
}

// int _motor1Speed = speedPID *sin(desireAngle + 45) + 0;
// int _motor2Speed = speedPID *cos(desireAngle + 45) - 0;
// int _motor3Speed = speedPID *cos(desireAngle + 45) + 0;
// int _motor2Speed = speedPID *sin(desireAngle + 45) - 0;