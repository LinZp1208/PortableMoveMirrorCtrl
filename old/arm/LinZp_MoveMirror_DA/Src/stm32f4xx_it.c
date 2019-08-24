/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "sys.h"

//float temp,humi,*p_temp,*p_humi;


float y=0,yf=0,yff=0;
float p=3.141;
float f2ilter_deal(float f0,float fs,float x)
{
	y=4*p*p*f0*f0*x/(fs*fs+6*p*f0*fs+4*f0*f0*p*p)+(2*fs*fs+6*p*f0*fs)*yf/(fs*fs+6*p*f0*fs+4*f0*f0*p*p)-fs*fs*yff/(fs*fs+6*p*f0*fs+4*f0*f0*p*p);
	yf=y;
	yff=yf;
	return y;
}
uint8_t SpeedTransmit=0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	//�����λ�����ж�
		
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */
	ResolutionInit();
	ZeroCheckTimes++;//��鵥���г�ͨ����翪�ش���
	if(Zero == 0)
	{
		Zero = PassedDistance;
	}
	ZeroOutput=Output;
	//PassedDistance=Zero;
	PassedDistance=0;                                         //������
	NotPassedZeroTimes=0;
	
  /* USER CODE END EXTI3_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream0 global interrupt.
*/
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream6 global interrupt.
*/
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	//���������ж�
	IsNewPulse=1;
	IsNewSpeed++;	
	newtest		=1;

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
	unsigned int PulsePeriod=0;
	PulsePeriod=*((uint32_t *)(0x60000000));

	MirrorSpeed=(27000000/PulsePeriod);
	//MirrorSpeed=f2ilter_deal(10,5,MirrorSpeed);
	
	//IsNewSpeedForUart=1;
	Error = SpeedTarget-MirrorSpeed;
	

	if (MoveDirection==MoveGoToSplitter) 
		PassedDistance+=1;
	else 
		PassedDistance-=1;
////////////////////////////////////////////////////////////////////////////
	NoPulseTimes=0;
	if((ProcessFlag==PIDCtrlFlag)&&(IsNewSpeed>=PIDDoor))                                      //�ջ�������
	{
		IsNewSpeed = 0;
		PIDSpeedCtrl();
		//���� ��̲����ͻ�е���

		if((MoveDirection==MoveGoToSplitter)&&(PassedDistance>(LimitCompensation + (GotoLimit+DistanceCompensate))))
		{
			ProcessFlag=StopFlag;
			NotPassedZeroTimes++;
			ZeroCheckTimes=0;
			ShtWatchFlag=1;//ÿ���������һ����ʪ��
			//ProcessFlag=ResetFlag;
			//IsNewSpeedForUart=1;
			//watch = GotoLimit;
			
			if(SleepFlag==1)SleepFlag=0;//ʹ����һ�����ص�ʱ�������δ�յ�˯��ָ��������ָ�����
		}
		if((MoveDirection==MoveAwaySplitter)&&(PassedDistance<(LimitCompensation - (AwayLimit+DistanceCompensate))))
		{
			ProcessFlag=StopFlag;
			NotPassedZeroTimes++;
			//ProcessFlag=ResetFlag;
			//PIDValue=PIDValue/2;
			//IsNewSpeedForUart=1;
			ZeroCheckTimes=0;
			ShtWatchFlag=1;//ÿ���������һ����ʪ��
			//watch = AwayLimit;
		}
	}
	
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	if(ProcessFlag==DebugFlag)
	{
		SetOutput(0);
		if(newtr<1000)
		//if(stopcount>50)
		{
			newtr=2000;
			char str[25];
			for(int i=0;i<stopcount;i++)
			{
				sprintf(str,"%d,",speedtestbuffer[i]);
				HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),1000);
			}
			sprintf(str,"\n\n\n\n\n");
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),1000);
			for(int i=0;i<stopcount;i++)
			{
				sprintf(str,"%d,",outtestbuffer[i]);
				HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),1000);
			}
			sprintf(str,"\n\n\n\n\n");
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),1000);
			for(int i=0;i<stopcount;i++)
			{
				sprintf(str,"%d,",outxbuffer[i]);
				HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),1000);
			}
			sprintf(str,"\n\n\n\n\n");
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),1000);
			for(int i=0;i<stopcount;i++)
			{
				sprintf(str,"%d,",outybuffer[i]);
				HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),1000);
			}
		 }
	}
	if(ProcessFlag==ResetFlag)
	{
		SystemReset();
		PIDInit();
		ProcessFlag=ReStratFlag;
	}
	if(ProcessFlag==ReStratFlag)
	{
		SetOutput(0);												//
		MoveDirection=MoveGoToSplitter;    //
		//MoveDirection=MoveAwaySplitter;
		PIDReset();													//
		MirrorSpeed                  = 0;		//
		PassedDistance               = 0;		//
		Zero												 = 0;   //
		PIDValue                     = 0;   //
		ZeroArrived 								 = 0;		//
		NotPassedZeroTimes           = 0;		//
		NoPulseTimes                 = 0;		//
		GotoLimit=4396+256;
		AwayLimit=4396+256;
		
		ProcessFlag=WaitFlag;
		ShtWatchFlag=1;//ÿ�������λ����һ����ʪ��
	}
	if(ProcessFlag==WaitFlag)
	{
		ResetCount += 1;
		if(ResetCount==1000000)
		{
			ResetCount=0;
			ProcessFlag = preStartFlag;
		}
		PassedDistance=0;
	}
	if(ProcessFlag==preStartFlag)
	{
		preStartMove();
		ProcessFlag = StartFlag;
	}
		//if((ProcessFlag==StartFlag)&&(IsNewSpeed==1))                                       //����
	if(ProcessFlag==StartFlag) 
	{	
		IsNewSpeed = 0;
		StartStopCount++;
		//if(StartStopCount==40)
		if(StartStopCount==2)
		{
			StartStopCount=0;
				
			MovingMirrorSpeedUp();
				
		}
			////////////////////////////////////////////////////////////////////////////
		if((ReadSpeed[0]>=StartSpeedTarget)&&(ReadSpeed[1]>=StartSpeedTarget)&&(ReadSpeed[1]<StartSpeedTarget+7500))
		{
			UartSendFlag=1;
			if(MoveDirection==MoveGoToSplitter)
			{
				DIR1;		//ָʾ����//PG3
				DIROUT_1;//���ɼ��巽���źţ��ߵ�ƽ//PG7
			}
			if(MoveDirection==MoveAwaySplitter)
			{
				DIR0; //ָʾ����
				DIROUT_0;//���ɼ��巽���źţ��͵�ƽ
			}
				
			ProcessFlag=PIDCtrlFlag;
			//ProcessFlag=ResetFlag;
		}
	}

		//if((ProcessFlag==StopFlag)&&(IsNewSpeed==1))                                //����������
	if(ProcessFlag==StopFlag)												                               //����������
	{
		//PIDValue=PIDValue/10*8;
		IsNewSpeed = 0;
		UartSendFlag=0;
			
		StartStopCount++;
			
		if(StartStopCount>=10)
		{
			StartStopCount=0;
			if(MoveDirection==MoveGoToSplitter)
			{
				PIDValue-=GotoStopRate;//40		 
				OutputDrive();
			}
			else if(MoveDirection==MoveAwaySplitter)
			{
				//IsNewSpeedForUart=1;
				PIDValue+=AwayStopRate;
				//PIDValue=PIDValue/2;
				OutputDrive();
			}
		}
		if(MirrorSpeed<StopSpeedTarget)//3500
		{
				//�����־Ӧ���ı�
			if(MoveDirection==MoveGoToSplitter)
				MoveDirection=MoveAwaySplitter;
			else
				MoveDirection=MoveGoToSplitter;
			ProcessFlag=preStartFlag;
			//ProcessFlag=ResetFlag;
			//	IsNewSpeedForUart=0;
		}
	}
	
	
	if((abs(PassedDistance)<1000)&&(ProcessFlag!=ResetFlag))
	//if((PassedDistance<-3096)&&(ProcessFlag!=ResetFlag))
	{
		if(newtr<1990)
		{
			if(newtest==1)
			{
				newtest=0;
				speedtestbuffer[newtr]=MirrorSpeed;
				outtestbuffer[newtr]=Output;
				//outxbuffer[newtr]=PIDValue;//ProcessFlag*100000;
				outxbuffer[newtr]=ProcessFlag*1000;
				outybuffer[newtr]=PassedDistance*10;

				newtr++;
			}
		}
	}
	else
	{
		if(newtr!=0)
			stopcount=newtr;
		
		if(ProcessFlag!=DebugFlag)
			newtr = 0;

	}
	
  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	////////////////////////******���ڿ�������λ�����ٶ�*****///////////////////////////
	if(SpeedTransmit==1)
	{
		//////////////***************************************????????????????????
			char str[25];
			//sprintf(str,"%d,%d,%d,\r\n",MirrorSpeed,PassedDistance,Output);
			//sprintf(str,"%d,%d,\r\n",PassedDistance,Output);
			//sprintf(str,"%d,",PassedDistance-PasseddistanceRevise);
			sprintf(str,"%d,",MirrorSpeed);
			//sprintf(str,"%d,%d,\r\n",PassedDistance,MirrorSpeed);
			//sprintf(str,"%d,",PassedDistance);
			//if(UartSendFlag==1)
			{
				HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),1000);
			}
		
		//////////////***************************************????????????????????
	}

	////////////////////////******��δ������ٶȲ�����/������λ��*****///////////////////////////
	
	if(ProcessFlag!=WaitFlag) //�����������tim3���ж������Ż�
	{
		NoPulseTimes++;

	
		if(SpeedClass<20000)
		{
			if(MirrorSpeed>SpeedTarget+OverSpeedCheck)
				OverSpeedCount++;
			else
				OverSpeedCount = 0;
		}

		if(SpeedClass>40000)
		{
			if(MirrorSpeed>SpeedTarget+OverSpeedCheck)
				OverSpeedCount++;
			else
				OverSpeedCount = 0;
		}
		if(OverSpeedCount>OverSpeedDoor)//50
		{
			SetOutput(0);
			char str[25];
			
			sprintf(str,"?22#");
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),1000);
			sprintf(str,"?speedERROR*%d*#\r\n",MirrorSpeed);
///////////////////////////			HAL_UART_Transmit(&huart3,(uint8_t*)str,strlen(str),1000);
//			int MsgLength = MakeMessage(str, &MirrorSpeed, 'P', 0);
//			HAL_UART_Transmit(&huart1,(uint8_t*)str,MsgLength,1000);
//			HAL_UART_Transmit(&huart3,(uint8_t*)str,strlen(str),1000);
		
			OverSpeedCount=0;
			ProcessFlag = ReStratFlag;
		}

		if(NoPulseTimes>NoPulseTimesDoor)//1500
		{
			SetOutput(0);
			char str[25];
			sprintf(str,"?33#");
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),1000);
			sprintf(str,"?pulseERROR*#\r\n");
////////////////////////////////			HAL_UART_Transmit(&huart3,(uint8_t*)str,strlen(str),1000);
//			int MsgLength = MakeMessage(str, &MirrorSpeed, 'Q', 0);
//			HAL_UART_Transmit(&huart1,(uint8_t*)str,NoPulseTimes,1000);
//			HAL_UART_Transmit(&huart3,(uint8_t*)str,strlen(str),1000);
			NoPulseTimes=0;
			ProcessFlag = ReStratFlag;
		}
	
		if (NotPassedZeroTimes>NotPassedZeroDoor) //ƫһ�ߣ�δ�����
		{
			SetOutput(0);
			NotPassedZeroTimes = 0;
			char str[25];
			sprintf(str,"?44#");
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),1000);
		  sprintf(str,"?zeroERROR*#\r\n");
/////////////////////////////			HAL_UART_Transmit(&huart3,(uint8_t*)str,strlen(str),1000);
			ProcessFlag=ReStratFlag;
		}
		if (ZeroCheckTimes>=2) //�����г̶�δ�����翪��
		{
			SetOutput(0);
			ZeroCheckTimes = 0;
			char str[25];
			sprintf(str,"?55#");
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),1000);
		  sprintf(str,"?zero_check_too_much,please check the device!!*#\r\n");
/////////////////////////////			HAL_UART_Transmit(&huart3,(uint8_t*)str,strlen(str),1000);
			ProcessFlag=ReStratFlag;
		}
	}
		if((ShtWatchFlag==1)&&(SpeedTransmit!=1)&&(ShtError<3))//ÿ���������һ����ʪ��
	//if((ShtWatchFlag==1)&&(ShtError<3))//ÿ���������һ����ʪ��
	{
		float get_temp,get_humi;
		float temp_buffer[4],humi_buffer[4];
		ShtError=ShtMeasure(&get_temp,&get_humi);
		if(ShtError==0)
		{
			temp_buffer[0]=temp[0]=temp[1];
			temp_buffer[1]=temp[1]=temp[2];
			temp_buffer[2]=temp[2]=get_temp;
			
			humi_buffer[0]=humi[0]=humi[1];
			humi_buffer[1]=humi[1]=humi[2];
			humi_buffer[2]=humi[2]=get_humi;
			for(int i=0;i<2;i++)
			{
				for(int j=0;j<2;j++)
				{
					if(temp_buffer[i]>temp_buffer[i+1])
					{
						temp_buffer[3]=temp_buffer[i];
						temp_buffer[i]=temp_buffer[i+1];
						temp_buffer[i+1]=temp_buffer[3];
					}
				}
			}

			for(int i=0;i<2;i++)
			{
				for(int j=0;j<2;j++)
				{
					if(humi_buffer[i]>humi_buffer[i+1])
					{
						humi_buffer[3]=humi_buffer[i];
						humi_buffer[i]=humi_buffer[i+1];
						humi_buffer[i+1]=humi_buffer[3];
					}
				}
			}
		}
		temp[3]=temp_buffer[1];
		humi[3]=humi_buffer[1];
		
		ShtWatchFlag=0;
	}
///////////////////////////////////////////////////////////////////////////////////////////////////
	char Msg[20];
	int MsgLength;
	//int error=0;
	float get_temp,get_humi;
	
	if(CmdNum!=0)
	{
		switch(CmdBuf[CmdPos])
		{
			case 'B'://�ֱ���
				MsgLength = MakeMessage(Msg, &ResolutionClass, 'B', 0);
				HAL_UART_Transmit(&huart1,(uint8_t*)Msg,MsgLength,1000);
////////////////////////////				HAL_UART_Transmit(&huart3,(uint8_t*)Msg,MsgLength,1000);
				CmdPos++;
				break;
			case 'C'://��Դ��ѹ
				GetLightStatus();
				MsgLength = MakeMessage(Msg, &LightVoltage, 'C', 2);
				HAL_UART_Transmit(&huart1,(uint8_t*)Msg,MsgLength,1000);
/////////////////////////				HAL_UART_Transmit(&huart3,(uint8_t*)Msg,MsgLength,1000);
				CmdPos++;
				break;
			case 'D'://��Դ����
				GetLightStatus();
				MsgLength = MakeMessage(Msg, &LightCurrent, 'D', 2);
				HAL_UART_Transmit(&huart1,(uint8_t*)Msg,MsgLength,1000);
///////////////////////////////				HAL_UART_Transmit(&huart3,(uint8_t*)Msg,MsgLength,1000);
				CmdPos++;
				break;
			case 'E'://��ǰoffset
				MsgLength = MakeMessage(Msg, &LimitCompensation, 'E', 1);
				HAL_UART_Transmit(&huart1,(uint8_t*)Msg,MsgLength,1000);
/////////////////////////	/			HAL_UART_Transmit(&huart3,(uint8_t*)Msg,MsgLength,1000);
				CmdPos++;			
				break;
			case 'F'://�������¶�
				if(ShtError==0)
				{
					MsgLength = MakeMessage(Msg, temp+3, 'F', 3);
					HAL_UART_Transmit(&huart1,(uint8_t*)Msg,MsgLength,1000);
///////////////////////////////////////					HAL_UART_Transmit(&huart3,(uint8_t*)Msg,MsgLength,1000);
				}
				else
				{
					sprintf(Msg,"?66#\r\n");
					HAL_UART_Transmit(&huart1,(uint8_t*)Msg,strlen(Msg),1000);
//////////////////////////////////////					HAL_UART_Transmit(&huart3,(uint8_t*)Msg,strlen(Msg),1000);
				}
				CmdPos++;
				break;
			case 'H'://������ʪ��
				if(ShtError==0)
				{
					MsgLength = MakeMessage(Msg, humi+3, 'H', 3);
					HAL_UART_Transmit(&huart1,(uint8_t*)Msg,MsgLength,1000);
/////////////////////////////////					HAL_UART_Transmit(&huart3,(uint8_t*)Msg,MsgLength,1000);
				}else
				{
					sprintf(Msg,"?66#\r\n");
					HAL_UART_Transmit(&huart1,(uint8_t*)Msg,strlen(Msg),1000);
//////////////////////////					HAL_UART_Transmit(&huart3,(uint8_t*)Msg,strlen(Msg),1000);
				}
				CmdPos++;				
				break;
			case 'I'://��ǰ�ٶ�
				MsgLength = MakeMessage(Msg, &SpeedClass, 'I', 0);
				HAL_UART_Transmit(&huart1,(uint8_t*)Msg,MsgLength,1000);
////////////////////////////////				HAL_UART_Transmit(&huart3,(uint8_t*)Msg,MsgLength,1000);
				CmdPos++;			
				break;
			case 'M'://˯��
				if(SleepFlag==2)
				{
					sprintf(Msg,"?00#\r\n");
					HAL_UART_Transmit(&huart1,(uint8_t*)Msg,strlen(Msg),1000);
//////////////////////////////					HAL_UART_Transmit(&huart3,(uint8_t*)Msg,strlen(Msg),1000);
				}
				CmdPos++;				
				break;
			case 'N'://����
				sprintf(Msg,"?11#\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t*)Msg,strlen(Msg),1000);
///////////////////////////////				HAL_UART_Transmit(&huart3,(uint8_t*)Msg,strlen(Msg),1000);
				CmdPos++;			
				break;
			case 'Q'://ʵʱ��ʪ��
				ShtError = ShtMeasure(&get_temp,&get_humi);
				if(ShtError==0)
				{
					MsgLength = MakeMessage(Msg, &get_temp, 'F', 3);
					HAL_UART_Transmit(&huart1,(uint8_t*)Msg,MsgLength,1000);
///////////////////////////					HAL_UART_Transmit(&huart3,(uint8_t*)Msg,MsgLength,1000);
					
					MsgLength = MakeMessage(Msg, &get_humi, 'H', 3);
					HAL_UART_Transmit(&huart1,(uint8_t*)Msg,MsgLength,1000);
////////////////////////////					HAL_UART_Transmit(&huart3,(uint8_t*)Msg,MsgLength,1000);
				}else
				{
					sprintf(Msg,"?66#\r\n");
					HAL_UART_Transmit(&huart1,(uint8_t*)Msg,strlen(Msg),1000);
////////////////////////////					HAL_UART_Transmit(&huart3,(uint8_t*)Msg,strlen(Msg),1000);
				}
				
				CmdPos++;			
				break;
		}
		
		if(CmdNum==CmdPos)
		{
			CmdNum=0;
			CmdPos=0;
		}
		
	}	
	
	
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)
	{
		Uart1_Data_length++;
	
		if((Uart1_Data_length>1)&&((Uart1_Data>'9')||(Uart1_Data<'0')))
		{
			Uart1_Data_length=1;
		}
	
		Uart1_Data_buffer[Uart1_Data_length-1]=Uart1_Data;
	
		if(Uart1_Data_length==6)
		{
			Uart1_Data_length=0;
			int rev_data = (Uart1_Data_buffer[1] - 0x30)*10000 + (Uart1_Data_buffer[2] - 0x30)*1000+(Uart1_Data_buffer[3] - 0x30)*100+(Uart1_Data_buffer[4] -  0x30)*10+(Uart1_Data_buffer[5] -  0x30);
			if((Uart1_Data_buffer[0]!='M')&&(SleepFlag==1))	
				SleepFlag=0;//��仰ʹ��˯����������������η���
			switch(Uart1_Data_buffer[0])
			{
				/**************************����ָ��****************************/
				case 'K':
					ProcessFlag = ResetFlag;
					break;
				case 'S':
					//ProcessFlag = ResetFlag;
					SpeedClass=rev_data;
					break;
				case 'R':
//					ProcessFlag=ReStratFlag;
//					if(ResolutionClass==160)ProcessFlag = ReStratFlag;
					ResolutionClass = rev_data;
					break;
				case '+':
					ProcessFlag = ReStratFlag;
					LimitCompensation = rev_data;
					break;
				case '-':
					ProcessFlag = ReStratFlag;
					LimitCompensation = -rev_data;
					break;
				case 'A':
					LightIntensity= rev_data;
					AD5245_Write(LightIntensity);
					break;
				case 'L':
					if(Uart1_Data_buffer[5] == 0x30)InfraredLight_OFF;//L0xxxx
					if(Uart1_Data_buffer[5] == 0x31)InfraredLight_ON;//L1xxxx
					break;			
				case 'G':
					GatherDirection= rev_data;
					break;
				case 'J':
					ProcessFlag=DebugFlag;
					break;
				case 'P':
					DistanceCompensate=rev_data;
					break;
				
				/***************************************************************/
				
				
				/**************************��ѯָ��*****************************/
				case 'M':
					CmdBuf[CmdNum]='M';
					CmdNum++;
				
					SleepFlag++;
					break;	
				case 'N':
					CmdBuf[CmdNum]='N';
					CmdNum++;
				
					SleepFlag=-1;
					break;

				case 'B':
					CmdBuf[CmdNum]='B';
					CmdNum++;
				
					break;
				case 'C':
					CmdBuf[CmdNum]='C';
					CmdNum++;
					break;
				case 'D':
					CmdBuf[CmdNum]='D';
					CmdNum++;
					break;
				case 'E':
					CmdBuf[CmdNum]='E';
					CmdNum++;
					break;
				case 'F':
					CmdBuf[CmdNum]='F';
					CmdNum++;
					break;
				case 'H':
					CmdBuf[CmdNum]='H';
					CmdNum++;
					break;
				case 'I':
					CmdBuf[CmdNum]='I';//Q00000��ѯʵʱ��ʪ��
					CmdNum++;
					break;
				case 'Q':
					CmdBuf[CmdNum]='Q';
					CmdNum++;
					break;
				case 'O':
					SpeedTransmit	=1;
					break;	
				/******************************************************************/

			}
			//PIDInit();
		}
		HAL_UART_Receive_IT(&huart1,&Uart1_Data,1);
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	if(huart==&huart3)
//	{
//		Uart3_Data_length++;
//	
//		if((Uart3_Data_length>1)&&((Uart3_Data>'9')||(Uart3_Data<'0')))
//		{
//			Uart3_Data_length=1;
//		}
//	
//		Uart3_Data_buffer[Uart3_Data_length-1]=Uart3_Data;
//	
//		if(Uart3_Data_length==6)
//		{
//			Uart3_Data_length=0;
//			int rev_data = (Uart3_Data_buffer[1] - 0x30)*10000 + (Uart3_Data_buffer[2] - 0x30)*1000+(Uart3_Data_buffer[3] - 0x30)*100+(Uart3_Data_buffer[4] -  0x30)*10+(Uart3_Data_buffer[5] -  0x30);
//			if((Uart3_Data_buffer[0]!='M')&&(SleepFlag==1))	
//				SleepFlag=0;//��仰ʹ��˯����������������η���
//			switch(Uart3_Data_buffer[0])
//			{
//				/**************************����ָ��****************************/
//				case 'K':
//					ProcessFlag = ResetFlag;
//					break;
//				case 'S':
//					//ProcessFlag = ResetFlag;
//					SpeedClass=rev_data;
//					break;
//				case 'R':
//					ProcessFlag=ReStratFlag;
//					if(ResolutionClass==160)ProcessFlag = ReStratFlag;
//					ResolutionClass = rev_data;
//					break;
//				case '+':
//					ProcessFlag = ReStratFlag;
//					LimitCompensation = rev_data;
//					break;
//				case '-':
//					ProcessFlag = ReStratFlag;
//					LimitCompensation = -rev_data;
//					break;
//				case 'A':
//					LightIntensity= rev_data;
//					AD5245_Write(LightIntensity);
//					break;
//				case 'L':
//					if(Uart3_Data_buffer[5] == 0x30)InfraredLight_OFF;//L0xxxx
//					if(Uart3_Data_buffer[5] == 0x31)InfraredLight_ON;//L1xxxx
//					break;			
//				case 'G':
//					GatherDirection= rev_data;
//					break;
//				case 'J':
//					ProcessFlag=DebugFlag;
//					break;
//				case 'P':
//					DistanceCompensate=rev_data;
//					break;
//				/***************************************************************/
//				
//				
//				/**************************��ѯָ��*****************************/
//				case 'M':
//					CmdBuf[CmdNum]='M';
//					CmdNum++;
//					SleepFlag++;
//					break;	
//				case 'N':
//					CmdBuf[CmdNum]='N';
//					CmdNum++;
//					SleepFlag=-1;
//					break;
//				case 'B':
//					CmdBuf[CmdNum]='B';
//					CmdNum++;
//				
//					break;
//				case 'C':
//					CmdBuf[CmdNum]='C';
//					CmdNum++;
//					break;
//				case 'D':
//					CmdBuf[CmdNum]='D';
//					CmdNum++;
//					break;
//				case 'E':
//					CmdBuf[CmdNum]='E';
//					CmdNum++;
//					break;
//				case 'F':
//					CmdBuf[CmdNum]='F';
//					CmdNum++;
//					break;
//				case 'H':
//					CmdBuf[CmdNum]='H';
//					CmdNum++;
//					break;
//				case 'I':
//					CmdBuf[CmdNum]='I';
//					CmdNum++;
//					break;
//				case 'Q':
//					CmdBuf[CmdNum]='Q';
//					CmdNum++;
//					break;
//				case 'O':
//					SpeedTransmit	=1;
//					break;	
//				/******************************************************************/

//			}
//			//PIDInit();
//		}
//		HAL_UART_Receive_IT(&huart3,&Uart3_Data,1);
//	}
	
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
