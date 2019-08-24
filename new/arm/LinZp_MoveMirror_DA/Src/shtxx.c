#include "sys.h"
//void delay_us(unsigned int tim)    //延时微秒
//{
//	int j = 0;
//	int i = 0;
//	for(i=tim; i>0; i--) 
//	{
//		for(j=42; j>0; j--) ;
//	}	
//}
//void delay_ms(unsigned int tim)    //延时毫秒
//{
//	int j = 0;
//	int i = 0;
//	for(i=tim; i>0; i--) 
//	{
//		for(j=42418; j>0; j--) ;
//	}
//}
void SetDataOut()//设置信号pe6为输出模式
{
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin =SHT_DAT;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_PP
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void SetDataIn()//设置信号pe6为输入模式
{
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin =SHT_DAT;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//GPIO_MODE_OUTPUT_PP
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

//SHT传输起始
//------------------------------------------------------------------------------
// generates a transmission start 
// _____ ________
// DAT: |_______|
// ___ ___
// SCK : ___| |___| |______
void SHT_Tra_Sta(void)
{
	SHT_DAT_OUT;
	SHT_DAT_1;
	HAL_Delay_us(10);
	SHT_SCK_0;
	HAL_Delay_us(10);
	SHT_SCK_1;
	HAL_Delay_us(10);
	SHT_DAT_0;
	HAL_Delay_us(10);
	SHT_SCK_0;
	HAL_Delay_us(10);
	SHT_SCK_1;
	HAL_Delay_us(10);
	SHT_DAT_1;
	HAL_Delay_us(10);
	SHT_SCK_0; 
	HAL_Delay_us(10);//经过亲自校验时序没有问题
}

//SHT传输重启
//------------------------------------------------------------------------------
// communication reset: DAT_line=1 and at least 9 SCK cycles followed by transstart
// _____________________________________________________ ________
// DAT: |_______|
// _ _ _ _ _ _ _ _ _ ___ ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______| |___| |______
void SHT_Tra_Res(void)
{
	uchar temp; 

	SHT_DAT_OUT;
	SHT_DAT_1;
	HAL_Delay_us(10);
	SHT_SCK_0;
	HAL_Delay_us(10);
	for(temp=0;temp<9;temp++)
	{
		SHT_SCK_1;
		HAL_Delay_us(10);
		SHT_SCK_0;
		HAL_Delay_us(10);
	}
	SHT_Tra_Sta();//经过亲自校验时序没有问题
}
//向SHT写入一个字节
//------------------------------------------------------------------------------
// writes a byte on the Sensibus and checks the acknowledge 
uchar SHT_Wri_Byte(uchar value)
{
	uchar temp,error=0;

	SHT_DAT_OUT;
	for(temp=0x80;temp>0;temp/=2)//nice！
	{
		if(temp&value) SHT_DAT_1;
		else SHT_DAT_0;
		HAL_Delay_us(10);
		SHT_SCK_1;
		HAL_Delay_us(10);
		SHT_SCK_0;
		HAL_Delay_us(10);
	}
	//等待相应
	//SHT_DAT_OUT;
	SHT_DAT_1;
	SHT_DAT_IN;
	HAL_Delay_us(10);
	SHT_SCK_1;
	HAL_Delay_us(10);
	error=Read_SHT_Dat;
	//watch=error;
	SHT_SCK_0;
	HAL_Delay_us(10);
	//error=1时无响应
	return error;
}

//从SHT读取一个字节
//------------------------------------------------------------------------------
// reads a byte form the Sensibus and gives an acknowledge in case of "ack=1"
uchar SHT_Rea_Byte(uchar ack)
{
		uchar temp,val=0,read_temp;

		//释放数据线
		SHT_DAT_OUT;
		SHT_DAT_1;
		HAL_Delay_us(10);

		SHT_DAT_IN;
		for(temp=0x80;temp>0;temp/=2)
		{
			SHT_SCK_1;
			HAL_Delay_us(10);
			read_temp=Read_SHT_Dat;
			if(read_temp) val|=temp;
			SHT_SCK_0;
			HAL_Delay_us(10);
		}

		//是否停止传输
		SHT_DAT_OUT;
		//继续
		if(ack) SHT_DAT_0;
		//停止
		else SHT_DAT_1; 
		SHT_SCK_1;
		HAL_Delay_us(10);
		SHT_SCK_0;
		HAL_Delay_us(10); 
		SHT_DAT_1;
		HAL_Delay_us(10);
		return val;
}
//SHT软件复位
//------------------------------------------------------------------------------
// resets the sensor by a softreset
uchar SHT_Sof_Res(void)
{ 
	uchar success=0; 

	SHT_Tra_Res(); //reset communication
	success+=SHT_Wri_Byte(RESET); //send RESET-command to sensor
	return success; //success=0 in case of no response form the sensor
}


//读状态寄存器ps这个函数以后再说
//------------------------------------------------------------------------------
// reads the status register with checksum (8-bit)
//uchar SHT_Rea_Sta(uchar *p_value,uchar *p_checksum)
//{ 
//	uchar error=0;

//	SHT_Tra_Sta(); //transmission start
//	error=SHT_Wri_Byte(STATUS_REG_R); //send command to sensor
//	*p_value=SHT_Rea_Byte(SHT_ACK); //read status register (8-bit)
//	*p_checksum=SHT_Rea_Byte(SHT_noACK); //read checksum (8-bit) 
//	return error; //error=1 in case of no response form the sensor
//}

////写状态寄存器
////------------------------------------------------------------------------------
//// writes the status register with checksum (8-bit)
//uchar SHT_Wri_Sta(uchar *p_value)
//{ 
//uchar error=0;

//SHT_Tra_Sta(); //transmission start
//error+=SHT_Wri_Byte(STATUS_REG_W); //send command to sensor
//error+=SHT_Wri_Byte(*p_value); //send value of status register
//return error; //error>=1 in case of no response form the sensor
//}

//开始测量
//------------------------------------------------------------------------------
// makes a measurement (humidity/temperature) with checksum
uchar SHT_Mea_TH(uchar *p_value,uchar *p_checksum,uchar mode)
{ 
uchar error=0,read_temp;
ulong temp;

//transmission start
SHT_Tra_Sta(); 
//send command to sensor
switch(mode)
{
	case 0:error+=SHT_Wri_Byte(MEASURE_TEMP);break;
	case 1:error+=SHT_Wri_Byte(MEASURE_HUMI);break;
//default:break; 
}

/*准备接收*/ 
//数据线为输出
SHT_DAT_OUT;
SHT_DAT_1;
//数据线为输入
SHT_DAT_IN; 
/*wait until sensor has finished the measurement or timeout (~2 sec.) is reached*/
for(temp=0;temp<160000;temp++)//325900//16294967//4294967295
{ 
	read_temp=Read_SHT_Dat;
	if(read_temp==0) break;
}
	read_temp=Read_SHT_Dat;
	if(read_temp) error+=1;
// watch=read_temp;
//read the first byte (MSB)
	*(p_value+1)=SHT_Rea_Byte(SHT_ACK);
//read the second byte (LSB)
	*(p_value)=SHT_Rea_Byte(SHT_ACK);
//read checksum
	*p_checksum=SHT_Rea_Byte(SHT_noACK);
	return error;
}

//计算温湿度值
//------------------------------------------------------------------------------
// calculates temperature [C] and humidity [%RH] 
// input : humi [Ticks] (12 bit) 
// temp [Ticks] (14 bit)
// output: humi [%RH]
// temp [C]
void SHT_Cal_TH(float *p_humidity,float *p_temperature)
{
//for 12 Bit
static float C1=-4.0;
//for 12 Bit
static float C2=+0.0405;
//for 12 Bit
static float C3=-0.0000028;
static float T1=+0.01;
static float T2=+0.00008;
//rh:Humidity [Ticks] 12 Bit 
float rh=*p_humidity;
//t:Temperature [Ticks] 14 Bit
float t=*p_temperature;
//rh_lin:Humidity linear
float rh_lin;
//rh_true:Temperature compensated humidity
float rh_true;
//t_C:Temperature [?]
float t_C;

//calc. temperature from ticks to [c]
t_C=t*0.01-39.64; 
//calc. humidity from ticks to [%RH] 
rh_lin=C3*rh*rh+C2*rh+C1;
//calc. temperature compensated humidity [%RH]
rh_true=(t_C-25)*(T1+T2*rh)+rh_lin;
/*cut if the value is outside of the physical possible range*/
if(rh_true>100) rh_true=100;
if(rh_true<0.1) rh_true=0.1;

//return temperature [C]
*p_temperature=t_C;
//return humidity[%RH]
*p_humidity=rh_true;
}


////计算露点
////------------------------------------------------------------------------------
//// calculates dew point
//// input: humidity [%RH], temperature [?]
//// output: dew point [?]
//float SHT_Cal_DP(float h,float t)
//{
//float logEx;
//float dew_point;
//logEx=0.66077+7.5*t/(237.3+t)+(log10(h)-2);
//dew_point=(logEx-0.66077)*237.3/(0.66077+7.5-logEx);
//return dew_point;
//}
int ShtMeasure(float *p_temp,float *p_humi)
{
	int error=0;
	uchar temperature[2],humi[2],checksum[2];
  float Temp,Humi,*p_temperature,*p_humidity;
	p_temperature=&Temp;
	p_humidity=&Humi;
	error+=SHT_Mea_TH(temperature,checksum,TEMP);
	error+=SHT_Mea_TH(humi,checksum,HUMI);
  Temp=temperature[0]+temperature[1]*256;
	Humi=humi[0]+humi[1]*256;
	SHT_Cal_TH(p_humidity,p_temperature);
	*p_temp=Temp;
	*p_humi=Humi;
	return error;
}




