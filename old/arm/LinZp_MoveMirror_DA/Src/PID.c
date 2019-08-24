#include "sys.h"

/* External variables --------------------------------------------------------*/
//PID参数表，此表用于标号0322B动镜,强磁铁




/***************PID变量定义**************************************************/
int watch1=0,watch2=0,watch3=0,watch4=0;
int dError=0, Error=0, SubError=0,Total=0;

int	LastError   ;
int	PrevError   ;
int	SumError    ;
int	Std         ;

int	Proportion;
int	Integral  ;
int	Derivative;

int MechanicalZeroOffset;	           //机械零点（平衡点）和光电原点偏移 程序内自动测量
int Coefficient;                     //480弹性定律比例系数

int OutValueMax;			           //Maximum Current
int Output;
int PIDMax;

/***********************************************/
/*PID控制算法总程序的 初始化               */
/***********************************************/
void ResolutionInit(void)
{
	
	switch(ResolutionClass)
	{
		case 5:
			GotoLimit=16384;
			AwayLimit=32768;
			break;
		case 10:
			GotoLimit=16384;
			AwayLimit=16384;		
			break;
		case 20:
			GotoLimit=8192;
			AwayLimit=8192;		
			break;
		case 40:
			GotoLimit=4096;
			AwayLimit=4096;
			break;
		case 80:
			GotoLimit=2048;
			AwayLimit=2048;		
			break;
		case 160:
			GotoLimit=1024;
			AwayLimit=1024;
			break;
		case 320:
			GotoLimit=512;
			AwayLimit=512;
			break;
		default:
			ResolutionClass=40;
																
	}
	if(SleepFlag>=2)
	{
		GotoLimit=512;
		AwayLimit=512;
		
		//InfraredLight_OFF;
	}
	else if(SleepFlag<0)
	{
		SleepFlag = 0;
		//InfraredLight_ON;
		//AD5245_Write(LightIntensity);
	}

}


void PIDInit(void)
{
	OverSpeedCheck=2500;
	OverSpeedDoor=200;
	
	NoPulseTimesDoor=1375;
	
	NotPassedZeroDoor=2;
	
	MagneticRevise=1;//0.68

	switch(SpeedClass)
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
			Proportion                   = 1360;//1820;//1520//1300;
			Integral                     = 68;//50//18;
			Derivative                   = 136;//600;
			Coefficient                  = 250;//260;
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
		case 40000:
			SpeedTarget 								 = 40000;
			GotoAccRate									 = 50;
			GotoStopRate								 = 300;///300
			AwayAccRate									 = 50;
			AwayStopRate								 = 300;
		
			PIDDoor                      = 5;//4
			Proportion                   = 1238;//1520//1300;
			Integral                     = 68;//50//18;
			Derivative                   = 136;//600;
			Coefficient                  = 180;//250;
			StartSpeedTarget						 = 40000;
			StopSpeedTarget              = 4000;
		
		break;
		case 20000:
			SpeedTarget 								 = 20000;
			GotoAccRate									 = 5;
			GotoStopRate								 = 150;
			AwayAccRate									 = 5;
			AwayStopRate								 = 150;
		
			PIDDoor                      = 2;
			Proportion                   = 1100;//
			Integral                     = 34;//18;
			Derivative                   = 34;//600
			Coefficient                  = 250;//260
			StartSpeedTarget						 = 20000;
			StopSpeedTarget              = 4000;
		break;
		case 10000:
			SpeedTarget 								 = 10000;
		
			GotoAccRate									 = 4;
			GotoStopRate								 = 150;
			AwayAccRate									 = 4;
			AwayStopRate								 = 150;
		
			PIDDoor                      = 1;
			Proportion                   = 1100;//
			Integral                     = 34;//18;
			Derivative                   = 34;//600
			Coefficient                  = 250;//260
			StartSpeedTarget						 = 10000;
			StopSpeedTarget              = 4000;
		break;
		case 7500:
			SpeedTarget 								 = 7500;//4//25
			GotoAccRate									 = 4;
			GotoStopRate								 = 25;
			AwayAccRate									 = 4;
			AwayStopRate								 = 25;
			PIDDoor                      = 1;
			Proportion                   = 1100;//1156;//120;//900;//只用比例系数最大可到1200
			Integral                     = 34;//18;//30;//30;//17 60;//积分系数//暂时70是临界值
			Derivative                   = 34;//600;//50;//40/150;//微分系数
			Coefficient                  = 250;//260;//75;//86;//91;//65;//v//之前是21/在之后是15
			StartSpeedTarget             = 7400;//7500;//5000;
			StopSpeedTarget              = 3000;//3000;		
		
		break;
		case 5000:
			SpeedTarget 								 = 5000;
			GotoAccRate									 = 3;
			GotoStopRate								 = 150;
			AwayAccRate									 = 3;
			AwayStopRate								 = 150;
		
			PIDDoor                      = 1;
			Proportion                   = 590;//1300;//120;//900;//只用比例系数最大可到1200
			Integral                     = 20;//18;//30;//30;//17 60;//积分系数//暂时70是临界值
			Derivative                   = 20;//600;//50;//40/150;//微分系数
			Coefficient                  = 250;//260;//75;//86;//91;//65;//v//之前是21/在之后是15
			StartSpeedTarget             = 4900;//7500;//5000;
			StopSpeedTarget              = 3000;//3000;		
		break;
		case 2500://
			SpeedTarget 								 = 2500;
			GotoAccRate									 = 3;
			GotoStopRate								 = 250;
			AwayAccRate									 = 3;
			AwayStopRate								 = 250;
		
			PIDDoor                      = 1;
			Proportion                   = 590;//1300;//120;//900;//只用比例系数最大可到1200
			Integral                     = 14;//18;//30;//30;//17 60;//积分系数//暂时70是临界值
			Derivative                   = 14;//600;//50;//40/150;//微分系数
			Coefficient                  = 250;//260;//75;//86;//91;//65;//v//之前是21/在之后是15
			StartSpeedTarget             = 2400;//7500;//5000;
			StopSpeedTarget              = 1500;//3000;		
		break;
		default:SpeedClass=7500;
	}
	
	if(ResolutionClass<=80)
	{
		if((MoveDirection==MoveAwaySplitter)&&(GatherDirection==1))
		{
			SpeedTarget 								 = 40000;
			GotoAccRate									 = 30;
			GotoStopRate								 = 300;///300
			AwayAccRate									 = 30;
			AwayStopRate								 = 300;
		
			PIDDoor                      = 5;//4
			Proportion                   = 1238;//1520//1300;
			Integral                     = 68;//50//18;
			Derivative                   = 136;//600;
			Coefficient                  = 180;//250;
			StartSpeedTarget						 = 40000;
			StopSpeedTarget              = 4000;
		}
	
		if((MoveDirection==MoveGoToSplitter)&&(GatherDirection==2))
		{
			SpeedTarget 								 = 40000;
			GotoAccRate									 = 30;
			GotoStopRate								 = 300;///300
			AwayAccRate									 = 30;
			AwayStopRate								 = 300;
		
			PIDDoor                      = 5;//4
			Proportion                   = 1238;//1520//1300;
			Integral                     = 68;//50//18;
			Derivative                   = 136;//600;
			Coefficient                  = 180;//250;
			StartSpeedTarget						 = 40000;
			StopSpeedTarget              = 4000;
		}
	
//		if((MoveDirection==MoveAwaySplitter)&&(GatherDirection==1))
//		{
//			SpeedTarget 								 = 40000;
//			GotoAccRate									 = 30;
//			GotoStopRate								 = 300;
//			AwayAccRate									 = 30;
//			AwayStopRate								 = 300;
//			
//			PIDDoor                      = 5;//4
//			Proportion                   = 1238;//1520//1300;
//			Integral                     = 68;//50//18;
//			Derivative                   = 136;//600;
//			Coefficient                  = 250;//250;
//			StartSpeedTarget						 = 40000;
//			StopSpeedTarget              = 4000;
//		}
//	
//		if((MoveDirection==MoveGoToSplitter)&&(GatherDirection==2))
//		{
//			SpeedTarget 								 = 40000;
//			GotoAccRate									 = 30;
//			GotoStopRate								 = 300;
//			AwayAccRate									 = 30;
//			AwayStopRate								 = 300;
//			
//			PIDDoor                      = 5;//4
//			Proportion                   = 1238;//1520//1300;
//			Integral                     = 68;//50//18;
//			Derivative                   = 136;//600;
//			Coefficient                  = 250;//250;
//			StartSpeedTarget						 = 40000;
//			StopSpeedTarget              = 4000;
//		}
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

	//SpeedTarget                  = 10000;//10000															 //设定的扫描速度值
	//StartSpeedTarget             = 8200;//8200
	//StopSpeedTarget              = 3500;
	//SettedResolution             = 40;                               //设定分辨率

	MechanicalZeroOffset         = 0;
	
	OutValueMax 	               = 524287;
	PIDMax                       = 524287;
	OutputMax                    = 524287;
	/////////////////////////****************usartctrl****************//////////////////////////
}
/***************************************/
/*PID算法函数 
功能：计算PID的比例误差 积分误差 微分误差，存在结构体内

输入变量:含比例,微分,积分误差结构体的指针，
增量式PID
*/
/***************************************/
void PIDCalc(void)		
{
	/*********************************************************************/
	SubError = Error - LastError;                         // Delta(e(k))=e(k)-e(k-1)比例项
	dError = Error - 2*LastError + PrevError;   // 微分项
	/*********************************************************************/	
	PrevError = LastError;	
	LastError = Error;
	/*********************************************************************/	
	SubError =SubError  *Proportion      *MagneticRevise;	                            // 比例项SubError
	Error    =Error     *Integral        *MagneticRevise;			                                // 积分项
	dError   =dError    *Derivative      *MagneticRevise; 	                            // 微分项
	Total    = (SubError+ Error + dError)/100;
////	watch2 = Total;
////	return Total;														                        // 比例项SubError// 积分项 // 微分
////return (SubError+ Error + dError);
}
/***************************************/
/*输出补偿函数 
功能：根据修正的动镜模型，动镜位于不同位置时，相同电流提供的安培力是不同的。
此函数用于修正由此带来的误差
*/
/***************************************/
float CalcOutputK(void)
{	
	float K;
	float x;
	x=(float)abs(PassedDistance);
	if(PassedDistance>=0)
	{
		K=x/4000*0.12;
		K=K+1;
		K=sqrtf(K);

		return K;
	}
	else
	{
		K=x/4000*0.12;
		K=K+1;
		K=sqrtf(K);

		return 1/K;
	}
	
}

/****************************************************
函数功能:驱动函数，输出电机电压
输入变量 无
输出变量 无
*****************************************************/
void OutputDrive(void)
{	
	float K;
	Output=MechanicalZeroOffset+(Coefficient*(PassedDistance) *MagneticRevise)/10+PIDValue;
	
	K = CalcOutputK();
	Output=(Output-ZeroOutput)*K	+	ZeroOutput;

	if (Output<(-OutputMax)) 
		Output=(-OutputMax);
	if (Output>OutputMax) 
		Output=OutputMax;			//PWM pulse is 1952
	SetOutput(Output);
}


/******************************************
函数功能 动镜加速函数
输入变量  无
输出变量  无
二级放大输出有尖刺干扰，光源噪声影响，
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
		函数功能 动镜减速函数
		输入变量 无
		输出变量 无
***********************************************/
void StopMove(void)
{	


}
/********************************************************* 
函数功能 速度调整子程序 核心
输入变量 无
输出变量 无
*********************************************************/
int PIDSpeedCtrl(void)
{	
	int Temp;
	//*****************************************************
	PIDCalc();//Speed );	//PID
		
	if(Total>PIDMax) 	
		Total=PIDMax; 
	else if(Total<-PIDMax) 
		Total=-PIDMax;//过小时，设定一个最小值	
	if (MoveDirection==MoveGoToSplitter) 
		Temp=PIDValue + Total; 
	else 
		Temp=PIDValue - Total;   
	
	if(Temp<-OutValueMax) 
		Temp=-OutValueMax;
	else if(Temp>OutValueMax) 
	Temp=OutValueMax; //设定目前最大值
	
	PIDValue=Temp;
	OutputDrive();
	return 1;
}


