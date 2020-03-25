//include sau UART.h
#define left 0
#define rigt 1
#define roll hadc1
#define pitch hadc3

int16_t leftRawDistance;
int16_t rigtRawDistance;
int16_t pitchRawDistance;
double a_Linear = 0.09007;
double b_Linear = 10.04528;

void readADC(void);
void filterADC(void);

uint16_t rollRawValue[2];
uint16_t pitchRawValue[1];
uint16_t _rollLeftFilterWindow[10] = {0,0,0,0,0,0,0,0,0,0};
uint16_t _rollLeftFinalFilter;
uint16_t _rollRigtFilterWindow[10] = {0,0,0,0,0,0,0,0,0,0};
uint16_t _rollRigtFinalFilter;
uint16_t _pitchFilterWindow[10] = {0,0,0,0,0,0,0,0,0,0};
uint16_t _pitchFinalFilter;


void readADC(void)
{
	HAL_ADC_Start_DMA(&roll, (uint32_t*)rollRawValue, 2);
	HAL_ADC_Start_DMA(&pitch, (uint32_t*)pitchRawValue, 1);
	rigtRawDistance = a_Linear *rollRawValue[rigt] + b_Linear;
	leftRawDistance = a_Linear *rollRawValue[left] + b_Linear;
	pitchRawDistance = a_Linear *pitchRawValue[0] + b_Linear;
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
