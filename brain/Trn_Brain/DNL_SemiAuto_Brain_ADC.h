//include sau UART.h
//#define ADC_CONTINOUS_CONV

#define left 1
#define rigt 0

#define pitch hadc3
#define rollLeft hadc1
#define rollRigt hadc2

uint16_t leftRawADC;
uint16_t rigtRawADC;
uint16_t pitchRawADC;
uint16_t leftRawDistance;
uint16_t rigtRawDistance;
uint16_t pitchRawDistance;
double a_Linear = 0.09007;
double b_Linear = 10.04528;

void peripheralADC_Init(void);
void readADC(void);
void filterADC(void);

uint16_t rollRawValue[2];
uint16_t pitchRawValue[1];
//uint16_t pitchRawValue[3];
uint16_t _rollLeftFilterWindow[10] = {0,0,0,0,0,0,0,0,0,0};
uint16_t _rollLeftFinalFilter;
uint16_t _rollRigtFilterWindow[10] = {0,0,0,0,0,0,0,0,0,0};
uint16_t _rollRigtFinalFilter;
uint16_t _pitchFilterWindow[10] = {0,0,0,0,0,0,0,0,0,0};
uint16_t _pitchFinalFilter;

#ifndef ADC_CONTINOUS_MODE
void peripheralADC_Init(void)
{

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
void readADC(void)
{
	HAL_ADC_Start(&rollLeft);
	HAL_ADC_Start(&rollRigt);
	HAL_ADC_Start(&pitch);
	HAL_ADC_PollForConversion(&rollLeft, 1);
	HAL_ADC_PollForConversion(&rollRigt, 1);
	HAL_ADC_PollForConversion(&pitch, 1);
	leftRawADC = HAL_ADC_GetValue(&rollLeft);
	rigtRawADC = HAL_ADC_GetValue(&rollRigt);
	pitchRawADC = HAL_ADC_GetValue(&pitch);
	leftRawDistance = a_Linear *leftRawADC + b_Linear;
	rigtRawDistance = a_Linear *rigtRawADC + b_Linear;
	pitchRawDistance = a_Linear *pitchRawADC + b_Linear;
}

void filterADC(void)
{
	_rollLeftFilterWindow[0] = rollRawValue[left];
	_rollRigtFilterWindow[0] = rollRawValue[rigt];
	_pitchFilterWindow[0] = pitchRawValue[0];
	int _sumRollRawLeft;
	int _sumRollRawRigt;
	int _sumPitchRaw;
	for(int i = 0; i < 9; ++i)
	{
		_sumRollRawLeft += _rollLeftFilterWindow[i];
		_sumRollRawRigt += _rollRigtFilterWindow[i];
		_sumPitchRaw += _pitchFilterWindow[i];
	}
	_rollLeftFinalFilter = _sumRollRawLeft/10;
	_rollRigtFinalFilter = _sumRollRawRigt/10;
	_pitchFinalFilter = _sumPitchRaw/10;
	for(int i = 9; i > 0; --i)
	{
		_rollLeftFilterWindow[i] = _rollLeftFilterWindow[i-1];
		_rollRigtFilterWindow[i] = _rollRigtFilterWindow[i-1];
		_pitchFilterWindow[i] = _pitchFilterWindow[i-1];
	}
}
