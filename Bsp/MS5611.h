#ifndef _MS5611_H_
#define _MS5611_H_


#include "stm32f4xx_hal.h"

void ReadTempPres(float* temp,float* pres);
void cmd_reset(void);
#endif