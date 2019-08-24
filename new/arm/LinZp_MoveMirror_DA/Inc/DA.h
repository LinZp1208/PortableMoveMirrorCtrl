#ifndef __DA_H_
#define __DA_H_
#include "stm32f4xx_hal.h"

#define CS_0 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_RESET)
#define CS_1 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, GPIO_PIN_SET)

#define LDAC_LOW HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, GPIO_PIN_RESET)  
#define LDAC_HIGH  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, GPIO_PIN_SET)   //set LDAC as low infinitly

#define RESET_LOW HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, GPIO_PIN_RESET)  
#define RESET_HIGH HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, GPIO_PIN_SET)

#define CLR_LOW HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_RESET)
#define CLR_HIGH HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_SET)


HAL_StatusTypeDef Write_AD_5791(uint32_t j);
void Output_gate(long int j);
void ad5791delay(uint32_t i);
void DAC_init(void);
void Output_triangle(long int j);
void output_steep(long int j);

#endif
