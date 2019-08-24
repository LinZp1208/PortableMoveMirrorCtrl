#include "sys.h"

HAL_StatusTypeDef Write_AD_5791(uint32_t j){
	unsigned char str0[3];
	HAL_StatusTypeDef status;  //j== 1111 0000 1101 1011 0001 1000 0110 0001

	j=j&0x0fffff;//取后20位   //j== 0000 0000 0000 1011 0001 1000 0110 0001

	j=j|0x100000;//加上控制字节. 选.写模式 //j== 0000 0000 0001 1011 0001 1000 0110 0001
	//write the dac registry 
	str0[2]=(unsigned char)j;        //str[2]==  0110 0001   
	str0[1]=(unsigned char)(j>>8);   //str[1]==  0001 1000
	str0[0]=(unsigned char)(j>>16);  //str[0]==  0001 1011
																
	CS_0;

	status=HAL_SPI_Transmit(&hspi1, str0, 3, 0xffff);// 0001 1011 0001 1000 0110 0001 
	// 	HAL_SPI_Transmit(&hspi1, &str[0], 1, 0xffff);
	// 	HAL_SPI_Transmit(&hspi1, &str[1], 1, 0xffff);
	// 	HAL_SPI_Transmit(&hspi1, &str[2], 1, 0xffff);
	CS_1;
	return status;
}

void DAC_preInit(void)
{
	unsigned char str0[3];
	uint32_t j;
	j=0x200032;//控制字    010 00000 0000 0000 0011 0010  //

	str0[2]=(unsigned char)j; 
	str0[1]=(unsigned char)(j>>8);
	str0[0]=(unsigned char)(j>>16);

	CS_0;
	HAL_SPI_Transmit(&hspi1, str0, 3, 0xffff);
	//HAL_SPI_Transmit(&hspi1, str0, 3, 0x7fff);
	// 	HAL_SPI_Transmit(&hspi1, &str[0], 1, 0xffff);
	// 	HAL_SPI_Transmit(&hspi1, &str[1], 1, 0xffff);
	// 	HAL_SPI_Transmit(&hspi1, &str[2], 1, 0xffff);
	CS_1;
}


void Output_gate(long int j){
	Write_AD_5791(j);
	HAL_Delay_us(25);
	Write_AD_5791(0);
	HAL_Delay_us(25);
}

void Output_triangle(long int j)
{ 
	long int i;
// 	   Write_AD_5791(j);
	for(i=0;i<j;i+=200) Write_AD_5791(i);	
	if(i>=j) 
	{
		for(i=j;i>0;i-=200) Write_AD_5791(i);
  }
}

void output_steep(long int j)
{
	long int i;
	Write_AD_5791(j);
	for(i=j;i>0;i-=1000)  Write_AD_5791(i); 
	if(i<=0)  i=j;
}
void DAC_init(void)
{
	unsigned int j=0;	
	CS_1;
	HAL_Delay_ms(100);
	LDAC_LOW;
	HAL_Delay_ms(100);
	CLR_HIGH;
	HAL_Delay_ms(100);
	RESET_HIGH;
	HAL_Delay_ms(100);
	RESET_LOW;
	HAL_Delay_ms(100);
	RESET_HIGH;
	HAL_Delay_ms(100);
	for (j=0;j<10;j++) DAC_preInit();	
	//HAL_Delay_ms(100);
	SetOutput(0);
}




