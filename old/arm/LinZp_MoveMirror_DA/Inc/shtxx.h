#include "stm32f4xx_hal.h"

typedef  unsigned char uchar;
typedef unsigned long ulong;

void SetDataOut(void);
void SetDataIn(void);
int ShtMeasure(float *p_temp,float *p_humi);
//SHT传输起始
void SHT_Tra_Sta(void);
//SHT传输重启
void SHT_Tra_Res(void);
//向SHT写入一个字节
uchar SHT_Wri_Byte(uchar value); 
//从SHT读出一个字节
uchar SHT_Rea_Byte(uchar ack); 
//SHT软件复位
uchar SHT_Sof_Res(void); 
//读状态寄存器
uchar SHT_Rea_Sta(uchar *p_value, uchar *p_checksum);
//写状态寄存器
uchar SHT_Wri_Sta(uchar *p_value); 
//开始测量
uchar SHT_Mea_TH(uchar *p_value,uchar *p_checksum,uchar mode);
//计算温湿度
void SHT_Cal_TH(float *p_humidity,float *p_temperature);
//计算露点
float SHT_Cal_DP(float h,float t); 

#define SHT_SCK GPIO_PIN_5
#define SHT_DAT GPIO_PIN_6

#define SHT_SCK_1 HAL_GPIO_WritePin(GPIOE, SHT_SCK, GPIO_PIN_SET) //SHT_SCK=1
#define SHT_SCK_0 HAL_GPIO_WritePin(GPIOE, SHT_SCK, GPIO_PIN_RESET)//SHT_SCK=0

#define SHT_DAT_OUT SetDataOut() //数据输出
#define SHT_DAT_IN SetDataIn() //数据输入
#define SHT_DAT_1 HAL_GPIO_WritePin(GPIOE,SHT_DAT, GPIO_PIN_SET) //SHT_DAT=1
#define SHT_DAT_0 HAL_GPIO_WritePin(GPIOE,SHT_DAT, GPIO_PIN_RESET) //SHT_DAT=0

#define Read_SHT_Dat HAL_GPIO_ReadPin(GPIOE,SHT_DAT)

#define MEASURE_TEMP 0x03
#define MEASURE_HUMI 0x05

#define TEMP 0
#define HUMI 1
#define SHT_ACK 1
#define SHT_noACK 0














