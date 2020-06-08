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

//////////////////////////////////////////////////
//sân đỏ -> rolR + pit + yaw
int roL_pit_yaw_mixSpeed()
{
	int _motor1Speed = yawPID + roLPID + pitPID;
	int _motor1Dir;
	int _motor2Speed = yawPID + roLPID - pitPID;
	int _motor2Dir;
	int _motor3Speed = yawPID - roLPID - pitPID;
	int _motor3Dir;
	int _motor4Speed = yawPID - roLPID + pitPID;
	int _motor4Dir;
	
	if(_motor1Speed >= 0)
	{
		_motor1Dir = ccw;
	}
	else
	{
		_motor1Dir = fcw;
	}

	if(_motor2Speed >= 0)
	{
		_motor2Dir = ccw;
	}
	else
	{
		_motor2Dir = fcw;
	}

	if(_motor3Speed >= 0)
	{
		_motor3Dir = ccw;
	}
	else
	{
		_motor3Dir = fcw;
	}

	if(_motor4Speed >= 0)
	{
		_motor4Dir = ccw;
	}
	else
	{
		_motor4Dir = fcw;
	}

	controlMotor1(_motor1Speed, _motor1Dir);
	controlMotor2(_motor2Speed, _motor2Dir);
	controlMotor3(_motor3Speed, _motor3Dir);
	controlMotor4(_motor4Speed, _motor4Dir);
}

int roR_pit_yaw_mixSpeed()
{
	int _motor1Speed = yawPID - roRPID + pitPID;
	int _motor1Dir;
	int _motor2Speed = yawPID - roRPID - pitPID;
	int _motor2Dir;
	int _motor3Speed = yawPID + roRPID - pitPID;
	int _motor3Dir;
	int _motor4Speed = yawPID + roRPID + pitPID;
	int _motor4Dir;
	
	if(_motor1Speed >= 0)
	{
		_motor1Dir = ccw;
	}
	else
	{
		_motor1Dir = fcw;
	}

	if(_motor2Speed >= 0)
	{
		_motor2Dir = ccw;
	}
	else
	{
		_motor2Dir = fcw;
	}

	if(_motor3Speed >= 0)
	{
		_motor3Dir = ccw;
	}
	else
	{
		_motor3Dir = fcw;
	}

	if(_motor4Speed >= 0)
	{
		_motor4Dir = ccw;
	}
	else
	{
		_motor4Dir = fcw;
	}

	controlMotor1(_motor1Speed, _motor1Dir);
	controlMotor2(_motor2Speed, _motor2Dir);
	controlMotor3(_motor3Speed, _motor3Dir);
	controlMotor4(_motor4Speed, _motor4Dir);
}


int BluFieldControlAllMotor()
{

}

void yawControlMotor(int _PIDyaw)
{
	int _motor1Speed = _PIDyaw;
	int _motor1Dir;
	int _motor2Speed = _PIDyaw;
	int _motor2Dir;
	int _motor3Speed = _PIDyaw;
	int _motor3Dir;
	int _motor4Speed = _PIDyaw;
	int _motor4Dir;

	if (_motor1Speed >= 0)
	{
		_motor1Dir = ccw;
	}
	else	//_motor1Speed<0
	{
		_motor1Dir = fcw;
	}
	
	if (_motor2Speed >= 0)
	{
		_motor2Dir = ccw;
	}
	else	//motor2Speed<0
	{
		_motor2Dir = fcw;
	}
	
	if (_motor3Speed >= 0)
	{
		_motor3Dir = ccw;
	}
	else	//motor3Speed<0
	{
		_motor3Dir = fcw;
	}
	
	if (_motor4Speed >= 0)
	{
		_motor4Dir = ccw;
	}
	else	//motor4Speed<0
	{
		_motor4Dir = fcw;
	}
	
	controlMotor1(_motor1Speed, _motor1Dir);
	controlMotor2(_motor2Speed, _motor2Dir);
	controlMotor3(_motor3Speed, _motor3Dir);
	controlMotor4(_motor4Speed, _motor4Dir);
}

void roRControlMotor(int _PIDrol)
{
	int _motor1Speed = _PIDrol;
	int _motor1Dir;
	int _motor2Speed = _PIDrol;
	int _motor2Dir;
	int _motor3Speed = _PIDrol;
	int _motor3Dir;
	int _motor4Speed = _PIDrol;
	int _motor4Dir;

	if (_motor1Speed >= 0)
	{
		_motor1Dir = fcw;
	}
	else	//_motor1Speed<0
	{
		_motor1Dir = ccw;
	}
	
	if (_motor2Speed >= 0)
	{
		_motor2Dir = fcw;
	}
	else	//motor2Speed<0
	{
		_motor2Dir = ccw;
	}
	
	if (_motor3Speed >= 0)
	{
		_motor3Dir = ccw;
	}
	else	//motor3Speed<0
	{
		_motor3Dir = fcw;
	}
	
	if (_motor4Speed >= 0)
	{
		_motor4Dir = ccw;
	}
	else	//motor4Speed<0
	{
		_motor4Dir = fcw;
	}

	controlMotor1(_motor1Speed, _motor1Dir);
	controlMotor1(_motor2Speed, _motor2Dir);
	controlMotor1(_motor3Speed, _motor3Dir);
	controlMotor1(_motor4Speed, _motor4Dir);
}

void roLControlMotor(int _PIDrol)
{
	int _motor1Speed = _PIDrol;
	int _motor1Dir;
	int _motor2Speed = _PIDrol;
	int _motor2Dir;
	int _motor3Speed = _PIDrol;
	int _motor3Dir;
	int _motor4Speed = _PIDrol;
	int _motor4Dir;

	if (_motor1Speed >= 0)
	{
		_motor1Dir = ccw;
	}
	else	//_motor1Speed<0
	{
		_motor1Dir = fcw;
	}
	
	if (_motor2Speed >= 0)
	{
		_motor2Dir = ccw;
	}
	else	//motor2Speed<0
	{
		_motor2Dir = fcw;
	}
	
	if (_motor3Speed >= 0)
	{
		_motor3Dir = fcw;
	}
	else	//motor3Speed<0
	{
		_motor3Dir = ccw;
	}
	
	if (_motor4Speed >= 0)
	{
		_motor4Dir = fcw;
	}
	else	//motor4Speed<0
	{
		_motor4Dir = ccw;
	}

	controlMotor1(_motor1Speed, _motor1Dir);
	controlMotor1(_motor2Speed, _motor2Dir);
	controlMotor1(_motor3Speed, _motor3Dir);
	controlMotor1(_motor4Speed, _motor4Dir);
}

void pitControlMotor(int _PIDpit)
{
	int _motor1Speed = _PIDrol;
	int _motor1Dir;
	int _motor2Speed = _PIDrol;
	int _motor2Dir;
	int _motor3Speed = _PIDrol;
	int _motor3Dir;
	int _motor4Speed = _PIDrol;
	int _motor4Dir;

	if (_motor1Speed >= 0)
	{
		_motor1Dir = ccw;
	}
	else	//_motor1Speed<0
	{
		_motor1Dir = fcw;
	}
	
	if (_motor2Speed >= 0)
	{
		_motor2Dir = fcw;
	}
	else	//motor2Speed<0
	{
		_motor2Dir = ccw;
	}
	
	if (_motor3Speed >= 0)
	{
		_motor3Dir = fcw;
	}
	else	//motor3Speed<0
	{
		_motor3Dir = ccw;
	}
	
	if (_motor4Speed >= 0)
	{
		_motor4Dir = ccw;
	}
	else	//motor4Speed<0
	{
		_motor4Dir = fcw;
	}

	controlMotor1(_motor1Speed, _motor1Dir);
	controlMotor1(_motor2Speed, _motor2Dir);
	controlMotor1(_motor3Speed, _motor3Dir);
	controlMotor1(_motor4Speed, _motor4Dir);
}