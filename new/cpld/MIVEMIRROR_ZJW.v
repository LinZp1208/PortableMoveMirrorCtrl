module MIVEMIRROR_ZJW(
						////////////////////系统//////////////
						input clk,		//时钟
						input reset_n,	//reset
						/////////////////-----------///////////
						
						////////////////////激光脉冲////////////
						input SAMPLE_IN,//由CPLD计数
						input ZeroOffset,//光电开关
						output ARM_ZeroOffset,	//光电开关
						output CPLD_PG8,	//输入arm激光脉冲
						/////////////////-----------///////////
						
						
						////////////////spi控制DA//////////////
						input SPI1_MOSI,
						output SDIN,
						input SPI1_SCK,
						output SCLK,
						input ARM_GPIO_PB8,
						output SYNC,
						input SDO,
						output SPI1_MISO,		
						
						input ARM_GPIO_PC6,
						output LDAC,
						input ARM_GPIO_PC7,
						output RESET,
						input ARM_GPIO_PC8,
						output CLR,
						/////////////////-----------///////////
						
						
						////////////////输出到主板//////////////
						input CPLD_PG7,//ARM发出方向信号
						output DirOUT,//输出方向信号
						output SamplePulseOut, //脉冲给数据采集卡
						input CPLD_PB9,//输出VALID信号
						output MOTOR_PULSE_VALID,
						
						input MOTOR_IDLE_CTRL,
						output Arm_Motor_Idle,
						
						input MOTOR_LIGHT_IDLE_CTRL,
						output Arm_Light_Idle,
						/////////////////-----------///////////
						
						
						////////////////////FSMC//////////////
						input FSMC_NE1,
						input FSMC_NOE,
						input FSMC_NWE,
						input [6:0]Address,
						output ARM_IsPowerLow,
						inout [15:0]readdata
						/////////////////-----------///////////
											);
//////////////////////*************************spi控制DA(AD5394或AD5384)*****************************///////////////////////////
assign SYNC=ARM_GPIO_PB8;
assign SCLK=SPI1_SCK;
assign SDIN=SPI1_MOSI;
assign SPI1_MISO=SDO;

assign LDAC=ARM_GPIO_PC6;
assign RESET=ARM_GPIO_PC7;
assign CLR=ARM_GPIO_PC8;
//////////////////////*************************-------------------------*****************************///////////////////////////

assign DirOUT=CPLD_PG7;	 //方向信号

assign MOTOR_PULSE_VALID = CPLD_PB9;

assign ARM_ZeroOffset=ZeroOffset;//光电开关

assign Arm_Motor_Idle = MOTOR_IDLE_CTRL;

assign Arm_Light_Idle = MOTOR_LIGHT_IDLE_CTRL;

/////////////////////////////////
wire [15:0]readdatatmp;//线型
wire [31:0]T1;

assign readdatatmp = (Address[1:0]==2'd0) ? T1[15:0] :T1[31:16] ;//三目运算符？
assign readdata = (FSMC_NE1==1'b0) ? readdatatmp : 16'hzz	;

//assign readdatatmp = (Address[1:0]==2'd0) ? 8'h34 :// 仅可32bit访问模式
//						   (Address[1:0]==2'd1) ? 8'h12://三目运算符？��
//						   (Address[1:0]==2'd2) ? 8'h78 : 8'h59;//三目运算符？							
//assign readdata = (({FSMC_NE1,FSMC_NOE}==2'b00)&&(Address[5:2]==4'd0)) ? readdatatmp : 16'hzzzz	;

//assign T1=32'h12345678;
CaptureT CaptureT_inst(
			.clk(clk),
			.reset_n(1'b1),
			.SamplePlusIn1(SAMPLE_IN),//把两个模块里面相同引脚不同名字衔接起��
			.T1(T1),
			.samplein_sycronous(SamplePulseOut),
			.INT(CPLD_PG8)
			);
endmodule	
