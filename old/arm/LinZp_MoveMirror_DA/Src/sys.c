#include "sys.h"

//ϵͳȫ�ֱ���
/////////////////////////////////////////////////////������///////////////////////////////////////////////////////
int SpeedPreset=7500;																  //�ٶ�
int ResolutionPreset=80;                          		//�ֱ���
int LightIntensity=8;																//���ù�Դ��ѹ53��6.4v��8��10.6V

int SpeedClass;																  //�ֱ���
int ResolutionClass;                          		//�ٶ�
int GatherDirection=0;                                //���ÿ���ɨ��ķ���
int DistanceCompensate=256;																//�������г̲���



float MagneticRevise=1;                               //����ϵ��,����ȫ��Ϊ1�������ã��Ժ���ܻ��õ�
int SpeedTarget;															        //�趨��ɨ���ٶ�ֵ
int StartSpeedTarget;
int StopSpeedTarget;
short ZeroOffset;                                     //���ƫ��
int GotoLimit;                                        //�������������г�
int AwayLimit;                                        //Զ����������г�
int OutputMax;
uint16_t StartValue;
int LimitCompensation=0;
int PIDDoor;
int GotoAccRate;
int GotoStopRate;
int AwayAccRate;
int AwayStopRate;
///////////////////////////////////////////////////������//////////////////////////////////////////////////////////
unsigned int MirrorSpeed;          									   //�����ٶ�
int PassedDistance;			                            	 //��ǰλ�þ������̵㣬�������崥���жϴ�����Ҳ��������

unsigned char IsNewPulse;                              //�Ƿ������������
unsigned char IsNewSpeed;                               //�ٶ��Ƿ����

uint8_t MoveDirection;                                 //�����˶������־λ
int ReadSpeed[3];                                      //��¼��������

int Motor_Idle;
int Light_Idle;

unsigned char IsNewSpeedForUart = 0;
int PasseddistanceRevise=0;
int PasseddistanceRevisedFlag=0;
int ZeroArrived=0;
int ZeroChecked=0;
int Zero=0;                                            //��¼���λ��
int ZeroOutput=0;																		   //��¼������

float humi[4];																					 //��¼��ʪ��
float temp[4];

float LightVoltage=0;																	 //��Դ��ѹ
float LightCurrent=0;																	 //��Դ����
int ZeroCheckTimes=0;                                  //ÿ���г̸��¼������
int ShtError=0;
int CmdNum=0;
char CmdBuf[10];
int CmdPos=0;
int ShtWatchFlag=0;

////////////////////////////////////////////////////////������//////////////////////////////////////////////////////////////////
uint8_t ProcessFlag;																	 //Process��־λ
uint32_t ResetCount;                                   //��λ��ʱ������־λ
int PIDValue;                                          //�����ռ�ձ�
int StartStopCount;

int SleepFlag=0;
int UARTCount;
char Checkflag='0';
int UartSendFlag;
//uart1
uint8_t	Uart1_Data_buffer[6];
uint8_t	Uart1_Data;                                    //����ʵ�е��ַ�����
uint8_t Uart1_Trans_buffer[20];
uint8_t Uart1_Data_length=0;
//uart3
uint8_t	Uart3_Data_buffer[6];
uint8_t	Uart3_Data;                                    //����ʵ�е��ַ�����
uint8_t Uart3_Trans_buffer[20];
uint8_t Uart3_Data_length=0;

int ADCMeasureCount=0;
///////////////////////////////////////////////////////////�����λ/////////////////////////////////////////////////////////////////////
int OverSpeedCount=0;
int NotPassedZeroTimes;															   //û�о�������ʱ�䣬�Լ���������Ϊ��λ
unsigned int NoPulseTimes;                             //���������

int OverSpeedCheck;
int OverSpeedDoor;
int NoPulseTimesDoor;
int NotPassedZeroDoor;
///////////////////////////////////////////////////////////����ר��/////////////////////////////////////////////////////////////////////
int speedtestbuffer[2000];
int outtestbuffer[2000];
int outxbuffer[2000];
int outybuffer[2000];
int newtr=0;
int newtest=0;
int stopcount=0;
int watch = 0;


/******************************************************************************/
//                               Hal��systic��ʱ����׼��ʱ����
/*******************************************************************************/

//void delay_init()
//{
//	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
//	fac_us=SystemCoreClock/8000000;	      //Ϊϵͳʱ�ӵ�1/8��ʵ����Ҳ�����ڼ���1usSysTick��VAL������Ŀ
//	fac_ms=(u16)fac_us*1000;		//����ÿ��ms��Ҫ��systickʱ��������ÿ����SysTick��VAL������Ŀ   
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
//                               ��������da���
/*******************************************************************************/
void SetOutput(int compare)                     //void Tim_SetTim8Compare1(uint32_t compare)
{
	Write_AD_5791(524287 - compare);

}
/*******************************************************************************/
//                               ��ת�����ź�
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
//                               ϵͳ��λ
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

//���ADCֵ
//ch: ͨ��ֵ 0~16��ȡֵ��ΧΪ��ADC_CHANNEL_0~ADC_CHANNEL_16
//����ֵ:ת�����
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
    ADC3_ChanConf.Rank=1;                                       //��1�����У�����1
    ADC3_ChanConf.SamplingTime=ADC_SAMPLETIME_480CYCLES;        //����ʱ��               
    //HAL_ADC_ConfigChannel(&ADC1_Handler,&ADC1_ChanConf);        //ͨ������
//	 ADC1_ChanConf.Channel=ch; 
	while(HAL_ADC_ConfigChannel(&hadc3, &ADC3_ChanConf) != HAL_OK) {;}
	
    HAL_ADC_Start(&hadc3);                               //����ADC
	
    HAL_ADC_PollForConversion(&hadc3,10);                //��ѯת��
		
	 if(ch==4){
	 ADC3_ChanConf.Channel=ADC_CHANNEL_4;
	 }
	 
	  if(ch==5){
	 ADC3_ChanConf.Channel=ADC_CHANNEL_5;
	 HAL_Delay_ms(1);
			
	 }
	return (uint16_t)HAL_ADC_GetValue(&hadc3);	        //�������һ��ADC1�������ת�����
}

void GetLightStatus(void)
{
	int Current_ad = Get_Adc(4);
	int Voltage_ad = Get_Adc(5);
	
	LightVoltage=Voltage_ad*3.3/4096*6;																	 //��Դ��ѹ
	LightCurrent=Current_ad*3.3/4096/0.05/11;														//��Դ����
	
}

/******************************************************************************/
//                   ͨ��iicд����ad5245
/*******************************************************************************/

void AD5245_Write(uint8_t data)
{
	uint8_t ADDR_AD5245_Write = 0x58;//01011000

	HAL_I2C_Mem_Write(&hi2c1, ADDR_AD5245_Write, 0, I2C_MEMADD_SIZE_8BIT,&data,1,0xff);
}


/******************************************************************************/
//                   ����������Ϣ
//para
//Msg�������
//data��������
//sign��־
//type���ͣ�0�޷���������1�з���������2�޷���float;3�з�������
//
/*******************************************************************************/

int MakeMessage(char*Msg, void*data, unsigned char sign, int type)
{
	char dataBuf[50];
	char checkbuf[10];
	int outlength=0;
	unsigned char check = 0;
	//float voltage = 6.8345;

	if (type == 0)//�޷�������
	{
		int Data = *(int*)(data);
		sprintf(dataBuf, "%c%d", sign, Data);
	}
	if (type == 1)//�з�������
	{
		int Data = *(int*)(data);
		if(Data >= 0)
			sprintf(dataBuf, "%c+%d", sign, Data);
		if (Data < 0)
			sprintf(dataBuf, "%c-%d", sign, -Data);
	}
	if (type == 2)//�޷���float
	{
		float Data=*(float*)(data);
		sprintf(dataBuf, "%c%f", sign, Data);
	}
	if (type == 3)//�з���float
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




















