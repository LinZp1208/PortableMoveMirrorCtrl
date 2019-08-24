/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define InfraredLight_Ctrl_Pin GPIO_PIN_8
#define InfraredLight_Ctrl_GPIO_Port GPIOF
#define Sys_LED_Pin GPIO_PIN_1
#define Sys_LED_GPIO_Port GPIOC
#define Zero_LED_Pin GPIO_PIN_2
#define Zero_LED_GPIO_Port GPIOC
#define Light_Voltage_Pin GPIO_PIN_0
#define Light_Voltage_GPIO_Port GPIOA
#define Light_Current_Pin GPIO_PIN_1
#define Light_Current_GPIO_Port GPIOA
#define Light_IDLE_Pin GPIO_PIN_6
#define Light_IDLE_GPIO_Port GPIOG
#define DIROUT_Pin GPIO_PIN_7
#define DIROUT_GPIO_Port GPIOG
#define Pulse_INT_Pin GPIO_PIN_8
#define Pulse_INT_GPIO_Port GPIOG
#define Pulse_INT_EXTI_IRQn EXTI9_5_IRQn
#define DA_LDAC_Pin GPIO_PIN_6
#define DA_LDAC_GPIO_Port GPIOC
#define RESET_Pin GPIO_PIN_7
#define RESET_GPIO_Port GPIOC
#define DA_CLR_Pin GPIO_PIN_8
#define DA_CLR_GPIO_Port GPIOC
#define Clk_Cpld_Pin GPIO_PIN_8
#define Clk_Cpld_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_6
#define SCK_GPIO_Port GPIOB
#define DATA_Pin GPIO_PIN_7
#define DATA_GPIO_Port GPIOB
#define DA_CS_Pin GPIO_PIN_8
#define DA_CS_GPIO_Port GPIOB
#define Motor_Idle_Pin GPIO_PIN_0
#define Motor_Idle_GPIO_Port GPIOE
#define Light_Switch_Pin GPIO_PIN_1
#define Light_Switch_GPIO_Port GPIOE
#define Light_Switch_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
