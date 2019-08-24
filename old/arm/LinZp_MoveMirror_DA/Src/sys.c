#include "sys.h"

//系统全局变量
/////////////////////////////////////////////////////设置量///////////////////////////////////////////////////////
int SpeedPreset=7500;																  //速度
int ResolutionPreset=80;                          		//分辨率
int LightIntensity=8;																//设置光源电压53是6.4v；8是10.6V

int SpeedClass;																  //分辨率
int ResolutionClass;                          		//速度
int GatherDirection=0;                                //设置快速扫描的方向
int DistanceCompensate=256;																//动镜的行程补偿



float MagneticRevise=1;                               //磁力系数,现已全设为1不起作用，以后可能会用到
int SpeedTarget;															        //设定的扫描速度值
int StartSpeedTarget;
int StopSpeedTarget;
short ZeroOffset;                                     //零点偏移
int GotoLimit;                                        //靠近分束器的行程
int AwayLimit;                                        //远离分束器的行程
int OutputMax;
uint16_t StartValue;
int LimitCompensation=0;
int PIDDoor;
int GotoAccRate;
int GotoStopRate;
int AwayAccRate;
int AwayStopRate;
///////////////////////////////////////////////////测量量//////////////////////////////////////////////////////////
unsigned int MirrorSpeed;          									   //动镜速度
int PassedDistance;			                            	 //当前位置距离零光程点，激光脉冲触发中断次数，也即脉冲数

unsigned char IsNewPulse;                              //是否有新脉冲进来
unsigned char IsNewSpeed;                               //速度是否更新

uint8_t MoveDirection;                                 //动镜运动方向标志位
int ReadSpeed[3];                                      //记录两次数据

int Motor_Idle;
int Light_Idle;

unsigned char IsNewSpeedForUart = 0;
int PasseddistanceRevise=0;
int PasseddistanceRevisedFlag=0;
int ZeroArrived=0;
int ZeroChecked=0;
int Zero=0;                                            //记录零点位置
int ZeroOutput=0;																		   //记录零点输出

float humi[4];																					 //记录温湿度
float temp[4];

float LightVoltage=0;																	 //光源电压
float LightCurrent=0;																	 //光源电流
int ZeroCheckTimes=0;                                  //每个行程更新几次零点
int ShtError=0;
int CmdNum=0;
char CmdBuf[10];
int CmdPos=0;
int ShtWatchFlag=0;

////////////////////////////////////////////////////////控制量//////////////////////////////////////////////////////////////////
uint8_t ProcessFlag;																	 //Process标志位
uint32_t ResetCount;                                   //复位延时计数标志位
int PIDValue;                                          //输出的占空比
int StartStopCount;

int SleepFlag=0;
int UARTCount;
char Checkflag='0';
int UartSendFlag;
//uart1
uint8_t	Uart1_Data_buffer[6];
uint8_t	Uart1_Data;                                    //串口实行单字符接收
uint8_t Uart1_Trans_buffer[20];
uint8_t Uart1_Data_length=0;
//uart3
uint8_t	Uart3_Data_buffer[6];
uint8_t	Uart3_Data;                                    //串口实行单字符接收
uint8_t Uart3_Trans_buffer[20];
uint8_t Uart3_Data_length=0;

int ADCMeasureCount=0;
///////////////////////////////////////////////////////////软件复位/////////////////////////////////////////////////////////////////////
int OverSpeedCount=0;
int NotPassedZeroTimes;															   //没有经过零点的时间，以激光脉冲数为单位
unsigned int NoPulseTimes;                             //无脉冲次数

int OverSpeedCheck;
int OverSpeedDoor;
int NoPulseTimesDoor;
int NotPassedZeroDoor;
///////////////////////////////////////////////////////////调试专用/////////////////////////////////////////////////////////////////////
int speedtestbuffer[2000];
int outtestbuffer[2000];
int outxbuffer[2000];
int outybuffer[2000];
int newtr=0;
int newtest=0;
int stopcount=0;
int watch = 0;


/******************************************************************************/
//                               Hal库systic定时器标准延时函数
/*******************************************************************************/

//void delay_init()
//{
//	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
//	fac_us=SystemCoreClock/8000000;	      //为系统时钟的1/8，实际上也就是在计算1usSysTick的VAL减的数目
//	fac_ms=(u16)fac_us*1000;		//代表每个ms需要的systick时钟数，即每毫秒SysTick的VAL减的数目   
//}
void HAL_Delay_us(__IO uint32_t delay_us)
{  
	uint32_t first_value = 0;              
  uint32_t current_value = 0;         
  uint32_t reload = SysTick ->LOAD;   	
      
  uint32_t nus_number = delay_us * ((reload + 1) / 1000);  
  uint32_t change_number = 0;  
  
  first_value = SysTick ->VAL;  
  while(1)  
  {
		current_value = SysTick ->VAL;  
    if(current_value != first_value)  
    {
			if(current_value < first_value)
			{  
          change_number += first_value - current_value;  
          //change_number = first_value - current_value + change_number;
			}       
      else  
      {
				change_number += reload - current_value + first_value;  
      }
			first_value = current_value;    
      if(change_number >= nus_number)  
      {  
        break;  
      }  
    }  
  }  
}
void HAL_Delay_ms(__IO uint32_t delay_ms)
{
	for(int i=0;i<1000;i++)
		HAL_Delay_us(delay_ms);
}
/******************************************************************************/
//                               动镜控制da输出
/*******************************************************************************/
void SetOutput(int compare)                     //void Tim_SetTim8Compare1(uint32_t compare)
{
	Write_AD_5791(524287 - compare);

}
/*******************************************************************************/
//                               翻转方向信号
/*******************************************************************************/
void DirectionToggle(void)
{
//		HAL_Delay_ms(50);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
//		HAL_Delay_ms(50);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
}
/******************************************************************************/
//                               系统复位
/*******************************************************************************/
void SystemReset()
{
	SetOutput(0);
	
	MoveDirection=MoveGoToSplitter;    
	//MoveDirection=MoveAwaySplitter;
	PIDReset();
	MirrorSpeed                  = 0;
	PassedDistance               = 0;
	Zero												 = 0;
	LimitCompensation						 = 0;
	StartValue                   = 0;
	PIDValue                     = 0;
	
	UARTCount										 = 0;
	UartSendFlag                 = 0;
	PasseddistanceRevise				 = 0;
	PasseddistanceRevisedFlag    = 0;
	
	SpeedClass                  =SpeedPreset;
	ResolutionClass             =ResolutionPreset;
	
	NotPassedZeroTimes           = 0;
	//IsNewPulse                   = 0;//
	NoPulseTimes                 = 0;
	
}

//获得ADC值
//ch: 通道值 0~16，取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
//返回值:转换结果
uint16_t Get_Adc(uint8_t ch)
{
   ADC_ChannelConfTypeDef ADC3_ChanConf;
	  
	 if(ch==4){
	 ADC3_ChanConf.Channel=ADC_CHANNEL_4;
	 }
	 
	  if(ch==5){
	 ADC3_ChanConf.Channel=ADC_CHANNEL_5;
	 }
    

//	  ADC1_ChanConf.Channel=ch;
    ADC3_ChanConf.Rank=1;                                       //第1个序列，序列1
    ADC3_ChanConf.SamplingTime=ADC_SAMPLETIME_480CYCLES;        //采样时间               
    //HAL_ADC_ConfigChannel(&ADC1_Handler,&ADC1_ChanConf);        //通道配置
//	 ADC1_ChanConf.Channel=ch; 
	while(HAL_ADC_ConfigChannel(&hadc3, &ADC3_ChanConf) != HAL_OK) {;}
	
    HAL_ADC_Start(&hadc3);                               //开启ADC
	
    HAL_ADC_PollForConversion(&hadc3,10);                //轮询转换
		
	 if(ch==4){
	 ADC3_ChanConf.Channel=ADC_CHANNEL_4;
	 }
	 
	  if(ch==5){
	 ADC3_ChanConf.Channel=ADC_CHANNEL_5;
	 HAL_Delay_ms(1);
			
	 }
	return (uint16_t)HAL_ADC_GetValue(&hadc3);	        //返回最近一次ADC1规则组的转换结果
}

void GetLightStatus(void)
{
	int Current_ad = Get_Adc(4);
	int Voltage_ad = Get_Adc(5);
	
	LightVoltage=Voltage_ad*3.3/4096*6;																	 //光源电压
	LightCurrent=Current_ad*3.3/4096/0.05/11;														//光源电流
	
}

/******************************************************************************/
//                   通过iic写设置ad5245
/*******************************************************************************/

void AD5245_Write(uint8_t data)
{
	uint8_t ADDR_AD5245_Write = 0x58;//01011000

	HAL_I2C_Mem_Write(&hi2c1, ADDR_AD5245_Write, 0, I2C_MEMADD_SIZE_8BIT,&data,1,0xff);
}


/******************************************************************************/
//                   制作返回信息
//para
//Msg输出数据
//data输入数据
//sign标志
//type类型：0无符号整数；1有符号整数；2无符号float;3有符号整数
//
/*******************************************************************************/

int MakeMessage(char*Msg, void*data, unsigned char sign, int type)
{
	char dataBuf[50];
	char checkbuf[10];
	int outlength=0;
	unsigned char check = 0;
	//float voltage = 6.8345;

	if (type == 0)//无符号整数
	{
		int Data = *(int*)(data);
		sprintf(dataBuf, "%c%d", sign, Data);
	}
	if (type == 1)//有符号整数
	{
		int Data = *(int*)(data);
		if(Data >= 0)
			sprintf(dataBuf, "%c+%d", sign, Data);
		if (Data < 0)
			sprintf(dataBuf, "%c-%d", sign, -Data);
	}
	if (type == 2)//无符号float
	{
		float Data=*(float*)(data);
		sprintf(dataBuf, "%c%f", sign, Data);
	}
	if (type == 3)//有符号float
	{
		float Data = *(float*)(data);
		if (Data >= 0)
			sprintf(dataBuf, "%c+%f", sign, Data);
		if (Data < 0)
			sprintf(dataBuf, "%c-%f", sign, -Data);
	}

	for (int i = 0; i < strlen(dataBuf); i++)
	{
		check += dataBuf[i];
	}
	sprintf(checkbuf, "%02X", check);

	for (int i = 0; i < strlen(dataBuf) + 1; i++)
	{
		if (i == 0)
			Msg[i] = '?';
		else
			Msg[i] = dataBuf[i - 1];
	}
	Msg[strlen(dataBuf) + 1] = checkbuf[1];
	Msg[strlen(dataBuf) + 2] = '#';
	outlength = strlen(dataBuf) + 3;
	return outlength;
}




















