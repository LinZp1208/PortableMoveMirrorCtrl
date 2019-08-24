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

//ϵͳ�궨��

#define ResetFlag                    0                 //��λ��־λ
#define StartFlag                    1                 //�𶯱�־λ
#define PIDCalFlag                   2           		   //PID�������
#define StopFlag                     3         				 //���ٱ�־λ
#define preStartFlag                 4                 //����׼��
#define PIDCtrlFlag                  5
#define DebugFlag                    6
//#define StartFlagt                 7                 //�𶯱�־λ
//#define PIDCalFlagt                8           		   //PID�������
//#define StopFlagt                  9         				 //���ٱ�־λ
//#define preStartFlagt              10                //����׼��
//#define PIDCtrlFlagt               11
//#define DebugFlagt                 12
#define WaitFlag                     13                //�ȴ�
#define ReStratFlag                  14                //��������

#define MoveGoToSplitter             1      					 // �������������
#define MoveAwaySplitter             2     						 // Զ�����������



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
extern uint32_t ResetCount;                                   //��λ��ʱ������־λ
extern uint8_t ProcessFlag;																	  //Process��־λ
extern unsigned int MirrorSpeed;          									  //�����ٶ�
extern int Error;
extern int StartSpeedTarget;
extern uint8_t ProcessFlag;																	  //Process��־λ
extern uint8_t MoveDirection;                                 //�����˶������־λ
extern int PassedDistance;			                              //��ǰλ�þ������̵㣬�������崥���жϴ�����Ҳ��������
extern int NotPassedZeroTimes;															  //û�о�������ʱ�䣬�Լ���������Ϊ��λ
extern int SpeedTarget;															          //�趨��ɨ���ٶ�ֵ
extern short ZeroOffset;                                      //���ƫ��
extern unsigned char IsNewPulse;                              //�Ƿ������������
extern unsigned char IsNewSpeed;
extern unsigned int NoPulseTimes;                             //���������
extern int PIDValue;                                          //�����ռ�ձ�
extern int GotoLimit;
extern int AwayLimit;
extern int ReadSpeed[3];                                      //��¼��������
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
