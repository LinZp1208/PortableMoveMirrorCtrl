#ifndef __DA_H_
#define __DA_H_
#include "stm32f4xx_hal.h"


HAL_StatusTypeDef Write_AD_5791(uint32_t j);
void Output_gate(long int j);
void ad5791delay(uint32_t i);
void DAC_init(void);
void Output_triangle(long int j);
void output_steep(long int j);

#endif
