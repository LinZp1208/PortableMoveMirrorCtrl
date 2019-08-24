#ifndef __SYS_H__
#define __SYS_H__

#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_adc_ex.h"
#include "PID.h"
#include "shtxx.h"
#include "math.h"
#include "DA.h"

//系统宏定义

#define ResetFlag                    0                 //复位标志位
#define StartFlag                    1                 //起动标志位
#define PIDCalFlag                   2           		   //PID结算控制
#define StopFlag                     3         				 //减速标志位
#define preStartFlag                 4                 //加速准备
#define PIDCtrlFlag                  5
#define DebugFlag                    6
//#define StartFlagt                 7                 //起动标志位
//#define PIDCalFlagt                8           		   //PID结算控制
//#define StopFlagt                  9         				 //减速标志位
//#define preStartFlagt              10                //加速准备
//#define PIDCtrlFlagt               11
//#define DebugFlagt                 12
#define WaitFlag                     13                //等待
#define ReStratFlag                  14                //重新启动

#define MoveGoToSplitter             1      					 // 朝向分束器方向
#define MoveAwaySplitter             2     						 // 远离分束器方向



#define DIR0                         HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET)//???
#define DIR1                         HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET)
#define DIROUT_0                     HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET)
#define DIROUT_1                     HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET)


#define Sys_LED_ON                   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define Sys_LED_OFF                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)
#define Sys_LED_Toggle							 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1)
#define Zero_LED_ON                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)
#define Zero_LED_OFF                 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)
#define Zero_LED_Toggle							 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2)


#define CS_0 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2, GPIO_PIN_RESET)
#define CS_1 HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2, GPIO_PIN_SET)

#define LDAC_LOW HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, GPIO_PIN_RESET)  
#define LDAC_HIGH  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, GPIO_PIN_SET)   //set LDAC as low infinitly

#define CLR_LOW HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_RESET)  
#define CLR_HIGH HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_SET)  

#define RESET_LOW HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_RESET)  
#define RESET_HIGH HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8, GPIO_PIN_SET)

#define InfraredLight_ON    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_8, GPIO_PIN_RESET)
#define InfraredLight_OFF   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_8, GPIO_PIN_SET)

void HAL_Delay_us(__IO uint32_t delay_us);
void HAL_Delay_ms(__IO uint32_t delay_ms);
void SetOutput(int compare);
void SystemReset(void);
void DirectionToggle(void);
void GetLightStatus(void);
void AD5245_Write(uint8_t data);
int MakeMessage(char*Msg, void*data, unsigned char sign, int type);


extern SPI_HandleTypeDef hspi1;
extern ADC_HandleTypeDef hadc3;
extern I2C_HandleTypeDef hi2c1;
extern uint32_t ResetCount;                                   //复位延时计数标志位
extern uint8_t ProcessFlag;																	  //Process标志位
extern unsigned int MirrorSpeed;          									  //动镜速度
extern int Error;
extern int StartSpeedTarget;
extern uint8_t ProcessFlag;																	  //Process标志位
extern uint8_t MoveDirection;                                 //动镜运动方向标志位
extern int PassedDistance;			                              //当前位置距离零光程点，激光脉冲触发中断次数，也即脉冲数
extern int NotPassedZeroTimes;															  //没有经过零点的时间，以激光脉冲数为单位
extern int SpeedTarget;															          //设定的扫描速度值
extern short ZeroOffset;                                      //零点偏移
extern unsigned char IsNewPulse;                              //是否有新脉冲进来
extern unsigned char IsNewSpeed;
extern unsigned int NoPulseTimes;                             //无脉冲次数
extern int PIDValue;                                          //输出的占空比
extern int GotoLimit;
extern int AwayLimit;
extern int ReadSpeed[3];                                      //记录两次数据
extern int StartStopCount;
extern uint8_t	Uart1_Data_buffer[6];
extern uint8_t	Uart1_Data;
extern uint8_t Uart1_Trans_buffer[20];
extern uint8_t Uart1_Data_length;
extern uint8_t	Uart3_Data_buffer[6];
extern uint8_t	Uart3_Data;
extern uint8_t Uart3_Trans_buffer[20];
extern uint8_t Uart3_Data_length;



extern int UARTCount;
extern int UartSendFlag;
extern int StopSpeedTarget;
extern int SpeedClass;
extern int ResolutionClass;
extern int OverSpeedCount;
extern int Output;
extern unsigned char IsNewSpeedForUart;
extern int OutputMax;
extern int PasseddistanceRevise;
extern int PasseddistanceRevisedFlag;
extern int LimitCompensation;
extern int ADC_Value4;
extern int ADC_Value5;
extern int ADCMeasureCount;
extern int ZeroArrived;
extern int ZeroChecked;
extern int speedtestbuffer[2000];
extern int outtestbuffer[2000];
extern int outxbuffer[2000];
extern int outybuffer[2000];
extern int newtr;
extern int newtest;
extern int stopcount;
extern int PIDDoor;
extern int GatherDirection;
extern int GotoAccRate;
extern int GotoStopRate;
extern int AwayAccRate;
extern int AwayStopRate;
extern int OverSpeedCheck;
extern int OverSpeedDoor;
extern int NoPulseTimesDoor;
extern int NotPassedZeroDoor;
extern float MagneticRevise;
extern int Zero;
extern float humi[4];
extern float temp[4];
extern int LightIntensity;
extern float LightVoltage;
extern float LightCurrent;
extern char Checkflag;
extern int Motor_Idle;
extern int Light_Idle;
extern int watch;
extern int SleepFlag;
extern int ZeroCheckTimes;
extern int ZeroOutput;
extern int DistanceCompensate;
extern int ShtError;
extern int CmdNum;
extern char CmdBuf[10];
extern int CmdPos;
extern int ShtWatchFlag;

#endif
