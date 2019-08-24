#include "sys.h"

/* External variables --------------------------------------------------------*/
//PID�������˱����ڱ��0322Cǿ��������·ǿ����




/***************PID��������**************************************************/
int watch1=0,watch2=0,watch3=0,watch4=0;
int dError=0, Error=0, SubError=0,Total=0;

int	LastError   ;
int	PrevError   ;
int	SumError    ;
int	Std         ;

int	Proportion;
int	Integral  ;
int	Derivative;

int MechanicalZeroOffset;	           //��е��㣨ƽ��㣩�͹��ԭ��ƫ�� �������Զ�����
int Coefficient;                     //480���Զ��ɱ���ϵ��

int OutValueMax;			           //Maximum Current
int Output;
int PIDMax;

/***********************************************/
/*PID�����㷨�ܳ���� ��ʼ��               */
/***********************************************/
void PIDInit(void)
{
	OverSpeedCheck=1000;//1000
	OverSpeedDoor=50;//200
	
	NoPulseTimesDoor=500;//500
	
	NotPassedZeroDoor=2;
	
	MagneticRevise=0.38;

	switch(Speed_Class)
	{
		case 1:
			
		
//			SpeedTarget 								 = 80000;
//			GotoAccRate									 = 100;//30;
//			AwayAccRate									 = 100;//30
//			GotoStopRate								 = 1600;
//			AwayStopRate								 = 1600;
//			
//			PIDDoor                      = 6;//4
//			Proportion                   = 2000;//1820;//1520//1300;
//			Integral                     = 100;//50//18;
//			Derivative                   = 200;//600;
//			Coefficient                  = 260;//260;
//			StartSpeedTarget						 = 75000;
//			StopSpeedTarget              = 4000;
		////////////////////////////////75k//////////////////////////////////
//			SpeedTarget 								 = 75000;
//			GotoAccRate									 = 100;//30;
//			AwayAccRate									 = 100;//30
//			GotoStopRate								 = 1600;
//			AwayStopRate								 = 1600;
//			
//			PIDDoor                      = 6;//4
//			Proportion                   = 2000;//1820;//1520//1300;
//			Integral                     = 100;//50//18;
//			Derivative                   = 200;//600;
//			Coefficient                  = 260;//260;
//			StartSpeedTarget						 = 70000;
//			StopSpeedTarget              = 4000;
		////////////////////////////////70k//////////////////////////////////
			SpeedTarget 								 = 70000;
			GotoAccRate									 = 100;//30;
			GotoStopRate								 = 1500;
			AwayAccRate									 = 100;//30;
			AwayStopRate								 = 1500;
			
			PIDDoor                      = 5;//4
			Proportion                   = 2000;//1820;//1520//1300;
			Integral                     = 100;//50//18;
			Derivative                   = 200;//600;
			Coefficient                  = 260;//260;
			StartSpeedTarget						 = 70000;
			StopSpeedTarget              = 4000;
		////////////////////////////////65k//////////////////////////////////
//			SpeedTarget 								 = 65000;
//			GotoAccRate									 = 100;//30;
//			GotoStopRate								 = 1000;
//			AwayAccRate									 = 100;//30;
//			AwayStopRate								 = 1000;
//			
//			PIDDoor                      = 5;//4
//			Proportion                   = 2000;//1820;//1520//1300;
//			Integral                     = 100;//50//18;
//			Derivative                   = 200;//600;
//			Coefficient                  = 260;//260;
//			StartSpeedTarget						 = 65000;
//			StopSpeedTarget              = 4000;
		
			////////////////////////////////60k_new//////////////////////////////////
//			SpeedTarget 								 = 60000;
//			GotoAccRate									 = 100;//30;
//			GotoStopRate								 = 5000;
//			AwayAccRate									 = 100;//30;
//			AwayStopRate								 = 5000;
//			
//			PIDDoor                      = 5;//4
//			Proportion                   = 2000;//1820;//1520//1300;
//			Integral                     = 100;//50//18;
//			Derivative                   = 200;//600;
//			Coefficient                  = 260;//260;
//			StartSpeedTarget						 = 60000;
//			StopSpeedTarget              = 4000;
		////////////////////////////////60k//////////////////////////////////
//			SpeedTarget 								 = 60000;
//			GotoAccRate									 = 100;//30;
//			GotoStopRate								 = 900;
//			AwayAccRate									 = 100;//30;
//			AwayStopRate								 = 900;
//			
//			PIDDoor                      = 5;//4
//			Proportion                   = 2000;//1820;//1520//1300;
//			Integral                     = 100;//50//18;
//			Derivative                   = 200;//600;
//			Coefficient                  = 260;//260;
//			StartSpeedTarget						 = 60000;
//			StopSpeedTarget              = 4000;
		///////////////////////////////////50k//////////////////////////////////
//			SpeedTarget 								 = 50000;
//			GotoAccRate									 = 30;
//			GotoStopRate								 = 300;
//			AwayAccRate									 = 30;
//			AwayStopRate								 = 300;
//			
//			PIDDoor                      = 5;//4
//			Proportion                   = 1820;//1520//1300;
//			Integral                     = 100;//50//18;
//			Derivative                   = 200;//600;
//			Coefficient                  = 260;//260;
//			StartSpeedTarget						 = 50000;
//			StopSpeedTarget              = 4000;
		break;
		case 2:
			SpeedTarget 								 = 40000;
			GotoAccRate									 = 30;
			GotoStopRate								 = 300;
			AwayAccRate									 = 30;
			AwayStopRate								 = 300;
			
			PIDDoor                      = 5;//4
			Proportion                   = 1820;//1520//1300;
			Integral                     = 100;//50//18;
			Derivative                   = 200;//600;
			Coefficient                  = 260;//260;
			StartSpeedTarget						 = 40000;
			StopSpeedTarget              = 4000;
		break;
		case 3:
			SpeedTarget 								 = 20000;
			GotoAccRate									 = 2;
			GotoStopRate								 = 2;
			AwayAccRate									 = 2;
			AwayStopRate								 = 4;
			PIDDoor                      = 2;
			Proportion                   = 1300;//1300;//120;//900;//ֻ�ñ���ϵ�����ɵ�1200
			Integral                     = 50;//18;//30;//30;//17 60;//����ϵ��//��ʱ70���ٽ�ֵ
			Derivative                   = 160;//600;//50;//40/150;//΢��ϵ��
			Coefficient                  = 260;//260;//75;//86;//91;//65;//v//֮ǰ��21/��֮����15
			StartSpeedTarget						 = 30000;
			StopSpeedTarget              = 4000;
		break;
		case 4:
			SpeedTarget 								 = 10000;
			GotoAccRate									 = 2;
			GotoStopRate								 = 2;
			AwayAccRate								   	 = 2;
			AwayStopRate								 = 4;


			PIDDoor                      = 1;
			Proportion                   = 1300;//1300;//120;//900;//ֻ�ñ���ϵ�����ɵ�1200
			Integral                     = 50;//18;//30;//30;//17 60;//����ϵ��//��ʱ70���ٽ�ֵ
			Derivative                   = 160;//600;//50;//40/150;//΢��ϵ��
			Coefficient                  = 260;//260;//75;//86;//91;//65;//v//֮ǰ��21/��֮����15
			StartSpeedTarget						 = 10000;
			StopSpeedTarget              = 4000;
		break;
		case 5:
			SpeedTarget 								 = 7500;
			GotoAccRate									 = 2;
			GotoStopRate								 = 2;
			AwayAccRate									 = 2;
			AwayStopRate								 = 4;
		
			PIDDoor                      = 1;
			Proportion                   = 1300;//1300;//120;//900;//ֻ�ñ���ϵ�����ɵ�1200
			Integral                     = 50;//18;//30;//30;//17 60;//����ϵ��//��ʱ70���ٽ�ֵ
			Derivative                   = 160;//600;//50;//40/150;//΢��ϵ��
			Coefficient                  = 260;//260;//75;//86;//91;//65;//v//֮ǰ��21/��֮����15
			StartSpeedTarget             = 7400;//7500;//5000;
			StopSpeedTarget              = 3000;//3000;		
		break;
		case 6:
			SpeedTarget 								 = 5000;
			GotoAccRate									 = 2;
			GotoStopRate								 = 2;
			AwayAccRate									 = 2;
			AwayStopRate								 = 4;
		
			PIDDoor                      = 1;
			Proportion                   = 700;//1300;//120;//900;//ֻ�ñ���ϵ�����ɵ�1200
			Integral                     = 30;//18;//30;//30;//17 60;//����ϵ��//��ʱ70���ٽ�ֵ
			Derivative                   = 90;//600;//50;//40/150;//΢��ϵ��
			Coefficient                  = 260;//260;//75;//86;//91;//65;//v//֮ǰ��21/��֮����15
			StartSpeedTarget             = 4900;//7500;//5000;
			StopSpeedTarget              = 3000;//3000;		
		break;
		case 7:
			SpeedTarget 								 = 2500;
			GotoAccRate									 = 2;
			GotoStopRate								 = 2;
			AwayAccRate									 = 2;
			AwayStopRate								 = 4;
		
			PIDDoor                      = 1;
			Proportion                   = 400;//1300;//120;//900;//ֻ�ñ���ϵ�����ɵ�1200
			Integral                     = 20;//18;//30;//30;//17 60;//����ϵ��//��ʱ70���ٽ�ֵ
			Derivative                   = 50;//600;//50;//40/150;//΢��ϵ��
			Coefficient                  = 260;//260;//75;//86;//91;//65;//v//֮ǰ��21/��֮����15
			StartSpeedTarget             = 2400;//7500;//5000;
			StopSpeedTarget              = 1500;//3000;		
		break;
	}
	
	
	if (Resolution_Class == 5)
	{
		GotoLimit = 16384;
		AwayLimit = 32768;
	}
	else if (Resolution_Class == 10) {
		GotoLimit = 16384;
		AwayLimit = 16384;
	}
	else if (Resolution_Class == 20) {
		GotoLimit = 8192 + 256;
		AwayLimit = 8192 + 256;
	}
	else if (Resolution_Class == 40) {
		GotoLimit = 4096 + 256;
		AwayLimit = 4096 + 256;
	}
	else if (Resolution_Class == 80) {
		GotoLimit = 2048 + 256;
		AwayLimit = 2048 + 256;
	}
	else if (Resolution_Class == 160) {
		GotoLimit = 1024 + 256;
		AwayLimit = 1024 + 256;
	}
	else if (Resolution_Class == 320) {
		GotoLimit = 512 + 256;
		AwayLimit = 512 + 256;
	}

	
	if(Resolution_Class<=80)
	{
	
		if((MoveDirection==MoveAwaySplitter)&&(GatherDirection==1))
		{
			SpeedTarget 								 = 40000;
			GotoAccRate									 = 30;
			GotoStopRate								 = 300;
			AwayAccRate									 = 30;
			AwayStopRate								 = 300;
			
			PIDDoor                      = 5;//4
			Proportion                   = 1820;//1520//1300;
			Integral                     = 100;//50//18;
			Derivative                   = 200;//600;
			Coefficient                  = 260;//260;
			StartSpeedTarget						 = 40000;
			StopSpeedTarget              = 4000;
		}
	
		if((MoveDirection==MoveGoToSplitter)&&(GatherDirection==2))
		{
			SpeedTarget 								 = 40000;
			
			GotoAccRate									 = 30;
			GotoStopRate								 = 300;
			AwayAccRate									 = 30;
			AwayStopRate								 = 300;
			
			PIDDoor                      = 5;//4
			Proportion                   = 1820;//1520//1300;
			Integral                     = 50;//18;
			Derivative                   = 200;//600;
			Coefficient                  = 260;//260;
			StartSpeedTarget						 = 40000;
			StopSpeedTarget              = 4000;
		}
	}
	
}

void PIDReset(void)
{
	LastError                    = 0;  // Error[-1]
	PrevError                    = 0;  // Error[-2]
	SumError                     = 0;   // Sums of Errors	
	Std                          = 0;
	
	dError                       = 0; 
	Error                        = 0; 
	SubError                     = 0;
	Total                        = 0;
	
	StartStopCount               = 0;
//	PIDCount                     = 0;
	

	//DistanceLimit            		 = 8192;     //

	//SpeedTarget                  = 10000;//10000															 //�趨��ɨ���ٶ�ֵ
	//StartSpeedTarget             = 8200;//8200
	//StopSpeedTarget              = 3500;
	//SettedResolution             = 40;                               //�趨�ֱ���

	MechanicalZeroOffset         = 0;
	
	OutValueMax 	               = 524287;
	PIDMax                       = 524287;
	OutputMax                    = 524287;
	/////////////////////////****************usartctrl****************//////////////////////////
}
/***************************************/
/*PID�㷨���� 
���ܣ�����PID�ı������ ������� ΢�������ڽṹ����

�������:������,΢��,�������ṹ���ָ�룬
����ʽPID
*/
/***************************************/
void PIDCalc(void)		
{
	/*********************************************************************/
	SubError = Error - LastError;                         // Delta(e(k))=e(k)-e(k-1)������
	dError = Error - 2*LastError + PrevError;   // ΢����
	/*********************************************************************/	
	PrevError = LastError;	
	LastError = Error;
	/*********************************************************************/	
	SubError =SubError  *Proportion      *MagneticRevise;	                            // ������SubError
	Error    =Error     *Integral        *MagneticRevise;			                                // ������
	dError   =dError    *Derivative      *MagneticRevise; 	                            // ΢����
	Total    = (SubError+ Error + dError)/100;
////	watch2 = Total;
////	return Total;														                        // ������SubError// ������ // ΢��
////return (SubError+ Error + dError);
}

/****************************************************
��������:������������������ѹ
������� ��
������� ��
*****************************************************/
void OutputDrive(void)
{	
	Output=MechanicalZeroOffset+(Coefficient*PassedDistance *MagneticRevise)/10+PIDValue;
	
	//IsNewSpeedForUart=1;
	if (Output<(-OutputMax)) 
		Output=(-OutputMax);
	if (Output>OutputMax) 
		Output=OutputMax;			//PWM pulse is 1952
	SetOutput(Output);
}


/******************************************
�������� �������ٺ���
�������  ��
�������  ��
�����Ŵ�����м�̸��ţ���Դ����Ӱ�죬
*********************************************/
void preStartMove(void)
{
	ReadSpeed[0]=0;
	ReadSpeed[1]=0;
	PIDInit();
	PIDReset();
}

void MovingMirrorSpeedUp(void)
{	
	
	//if((MirrorSpeed>1000)&&(MirrorSpeed<10000))
  //if((MirrorSpeed>20)&&(MirrorSpeed<10000))
  //{
	if(MoveDirection==MoveGoToSplitter)
	{
		PIDValue+=GotoAccRate;//1;
		OutputDrive();
				//						SetOutput(50000);
	}else if(MoveDirection==MoveAwaySplitter)
	{
		PIDValue-=AwayAccRate;//1;
		OutputDrive();
	}
	if(IsNewPulse) 
	{
		//ReadSpeed[0]=ReadSpeed[1];
		ReadSpeed[0]=ReadSpeed[1];
		ReadSpeed[1]=MirrorSpeed;	
		IsNewPulse=0;			
	}	
//	}else
//	{
//		if(MoveDirection==MoveGoToSplitter)
//	{
//		PIDValue+=30;
//		OutputDrive();
//				//						SetOutput(50000);
//	}else if(MoveDirection==MoveAwaySplitter)
//	{
//		PIDValue-=30;		 
//		OutputDrive();
//	}
//	if(IsNewPulse) 
//	{
//		//ReadSpeed[0]=ReadSpeed[1];
//		ReadSpeed[0]=ReadSpeed[1];
//		ReadSpeed[1]=MirrorSpeed;	
//		IsNewPulse=0;			
//	}	
//	}
	
}
/**********************************************
		�������� �������ٺ���
		������� ��
		������� ��
***********************************************/
void StopMove(void)
{	


}
/********************************************************* 
�������� �ٶȵ����ӳ��� ����
������� ��
������� ��
*********************************************************/
int PIDSpeedCtrl(void)
{	
	int Temp;
	//*****************************************************
	PIDCalc();//Speed );	//PID
		
	if(Total>PIDMax) 	
		Total=PIDMax; 
	else if(Total<-PIDMax) 
		Total=-PIDMax;//��Сʱ���趨һ����Сֵ	
	if (MoveDirection==MoveGoToSplitter) 
		Temp=PIDValue + Total; 
	else 
		Temp=PIDValue - Total;   
	
	if(Temp<-OutValueMax) 
		Temp=-OutValueMax;
	else if(Temp>OutValueMax) 
	Temp=OutValueMax; //�趨Ŀǰ���ֵ
	
	PIDValue=Temp;
	OutputDrive();
	return 1;
}


