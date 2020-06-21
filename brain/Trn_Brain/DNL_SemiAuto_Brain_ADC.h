//include sau UART.h
//#define ADC_CONTINOUS_CONV

#define pitc 2
#define left 1
#define rigt 0
#define _pitc 2
#define _left 1
#define _rigt 0
#define k 1
#define pitch hadc3
//#define rollLeft hadc1
//#define rollRigt hadc2
#define rollLeft hadc3
#define rollRigt hadc3

uint16_t leftRawADC;
uint16_t rigtRawADC;
uint16_t pitchRawADC;

double leftRawDistance;
double rigtRawDistance;
double pitchRawDistance;

double leftDistance;
double rigtDistance;
double pitchDistance;

double aPitch_Linear;
double bPitch_Linear;
double aLeft_Linear;
double bLeft_Linear;
double aRigt_Linear;
double bRigt_Linear;

uint16_t adc3Value[3];
void peripheralADC_Init(void);
void readADC(void);
void filterADC(void);

double kalmanGain_Pitch;
double x_Pitch[2];
//double P_Pitch=2;         //covariance estimation (err_estimate)
//double R_Pitch=2;         //covariance of the observation noise (err_measure)
//double Q_Pitch = 0.0009;    //process variance
//double kalmanGain_Rigt;
//double x_Rigt[2];
//double P_Rigt=2;         //covariance estimation (err_estimate)
//double R_Rigt=2;         //covariance of the observation noise (err_measure)
//double Q_Rigt = 0.0009;    //process variance
//double kalmanGain_Left;
//double x_Left[2];
//double P_Left=2;         //covariance estimation (err_estimate)
//double R_Left=2;         //covariance of the observation noise (err_measure)
//double Q_Left = 0.0009;    //process variance
double kalmanFilter_Pitch(double mea);
double kalmanFilter_Left(double mea);
double kalmanFilter_Rigt(double mea);
uint16_t rollRawValue[2];
uint16_t pitchRawValue[1];
//uint16_t pitchRawValue[3];
//uint16_t _rollLeftFilterWindow[10] = {0,0,0,0,0,0,0,0,0,0};
//uint16_t _rollLeftFinalFilter;
//uint16_t _rollRigtFilterWindow[10] = {0,0,0,0,0,0,0,0,0,0};
//uint16_t _rollRigtFinalFilter;
//uint16_t _PitchFilterWindow[10] = {0,0,0,0,0,0,0,0,0,0};
//uint16_t _PitchFinalFilter;

#ifndef ADC_CONTINOUS_MODE
void peripheralADC_Init(void)
{
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3Value, 3);
}
#endif
#ifdef ADC_CONTINOUS_MODE
void peripheralADC_Init(void)
{
		HAL_ADC_Start_DMA(&rollLeft, (uint32_t*)rollRawValue, 2);
		HAL_ADC_Start_DMA(&rollRigt, (uint32_t*)pitchRawValue, 1);
		HAL_ADC_Start_DMA(&pitch, (uint32_t*)pitchRawValue, 3);
}
#endif
//void readADC(void)
//{
//	HAL_ADC_Start(&rollLeft);
//	HAL_ADC_Start(&rollRigt);
//	HAL_ADC_Start(&pitch);
//	HAL_ADC_PollForConversion(&rollLeft, 1);
//	HAL_ADC_PollForConversion(&rollRigt, 1);
//	HAL_ADC_PollForConversion(&pitch, 1);
//	leftRawADC = HAL_ADC_GetValue(&rollLeft);
//	rigtRawADC = HAL_ADC_GetValue(&rollRigt);
//	pitchRawADC = HAL_ADC_GetValue(&pitch);
//	leftRawDistance = a_Linear *leftRawADC + b_Linear;
//	rigtRawDistance = a_Linear *rigtRawADC + b_Linear;
//	pitchRawDistance = a_Linear *pitchRawADC + b_Linear;
//}

//void filterADC(void)
//{
//	_rollLeftFilterWindow[0] = rollRawValue[left];
//	_rollRigtFilterWindow[0] = rollRawValue[rigt];
//	_PitchFilterWindow[0] = pitchRawValue[0];
//	int _sumRollRawLeft;
//	int _sumRollRawRigt;
//	int _sumPitchRaw;
//	for(int i = 0; i < 9; ++i)
//	{
//		_sumRollRawLeft += _rollLeftFilterWindow[i];
//		_sumRollRawRigt += _rollRigtFilterWindow[i];
//		_sumPitchRaw += _PitchFilterWindow[i];
//	}
//	_rollLeftFinalFilter = _sumRollRawLeft/10;
//	_rollRigtFinalFilter = _sumRollRawRigt/10;
//	_PitchFinalFilter = _sumPitchRaw/10;
//	for(int i = 9; i > 0; --i)
//	{
//		_rollLeftFilterWindow[i] = _rollLeftFilterWindow[i-1];
//		_rollRigtFilterWindow[i] = _rollRigtFilterWindow[i-1];
//		_PitchFilterWindow[i] = _PitchFilterWindow[i-1];
//	}
//}

//double kalmanFilter_Pitch(double mea)
//{
//  kalmanGain_Pitch = P_Pitch /(P_Pitch + R_Pitch);
//  x_Pitch[k] = x_Pitch[k-1] + kalmanGain_Pitch *(mea - x_Pitch[k-1]);
//  P_Pitch =  (1.0 - kalmanGain_Pitch) *P_Pitch + fabs(x_Pitch[k-1]-x_Pitch[k]) *Q_Pitch;
//  x_Pitch[k-1] = x_Pitch[k];
//  return x_Pitch[k];
//}
//
//double kalmanFilter_Left(double mea)
//{
//  kalmanGain_Left = P_Left /(P_Left + R_Left);
//  x_Left[k] = x_Left[k-1] + kalmanGain_Left *(mea - x_Left[k-1]);
//  P_Left =  (1.0 - kalmanGain_Left) *P_Left + fabs(x_Left[k-1]-x_Left[k]) *Q_Left;
//  x_Left[k-1] = x_Left[k];
//  return x_Left[k];
//}
//
//double kalmanFilter_Rigt(double mea)
//{
//  kalmanGain_Rigt = P_Rigt /(P_Rigt + R_Rigt);
//  x_Rigt[k] = x_Rigt[k-1] + kalmanGain_Rigt *(mea - x_Rigt[k-1]);
//  P_Rigt =  (1.0 - kalmanGain_Rigt) *P_Rigt + fabs(x_Rigt[k-1]-x_Rigt[k]) *Q_Rigt;
//  x_Rigt[k-1] = x_Rigt[k];
//  return x_Rigt[k];
//}
