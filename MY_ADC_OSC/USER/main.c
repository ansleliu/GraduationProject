#include "led.h"
#include "delay.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "dac_TriangleWave.h"
#include "dac.h"
#include "key.h"
/////////////////////////////////////
// ADC转化实验
void GPIO_Configuration(void);
void RCC_Configuration(void);
void DMA_Configuration(void);
void ADC_Configuration(u8);
void NVIC_Configuration(void);
void RCC_Configuration(void);
void Time_Configuration(void);
void ADC_Restart(u8 cysj);
///////////////////////////////////////
ErrorStatus HSEStartUpStatus;
#define ADC1_DR_Address ((u32)0x4001244C)//ADC1外设地址 dma用
u32 ADC_ConvertedValue[4096];//ADC1采集数据缓存
u32 Lcd_Buff1[320];	//双lcd缓存用来实现lcd的快速擦写
u32 Lcd_Buff2[320];

DMA_InitTypeDef DMA_InitStructure;		  
u8 dma_flag=1;//记录双lcd缓存使用情况
u8 WK_flag=1;//记录位宽情况
u8 H_range=4,L_range=4;//调节波形显示的宽高
u32 min,max,average; //最大值最小值平均值
u16 QSW[2];//起始位				
u16 ZZW;//终止位

////////////////////////////////////
 int main(void)
 {	 
	u8 key;
    RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();

	delay_init();//延时函数初始化		  
	uart_init(9600);//串口初始化为
 	LED_Init();	 //LED端口初始化
	KEY_Init();	//键盘初始化
	//////////////////////////////////
	LCD_Init();	//LCD端口初始化
	LCD_Clear(BLACK);
	LCD_Display_Dir(1);
	POINT_COLOR = RED;

	Time_Configuration();
	DMA_Configuration();

	Dac_TriangleWave();
//	DAC_Mode_Init();
	ADC_Configuration(ADC_SampleTime_1Cycles5);

	while (1)
    {
		/////////////////////////
		//扩展与延时
	    key=KEY_Scan(0);
		if(key == 1)
		{
			if((4000-L_range*320)>=3*(QSW[1]-QSW[0])&&L_range!=13)//不能超过临界值
				L_range++;
		}
		else if(key == 2)
		{
			if(L_range > 1)
				L_range--;	
		}
		LED0 = !LED0;
	}
 }

/************************************************************
*配置GPIOC0为模拟输入
************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;  
	//PC3设置为模拟通道3                   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

//////////////////////////////////////////////////////////////////////////////
//时钟配置
void RCC_Configuration(void)
{   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);
  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);	
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 
    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);
    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
  ///////////////////////////////////////////////////
	//
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//使能DMA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);//使能ADC1,GPIOC时钟
}

/************************************************************
*配置DMA
************************************************************/
void DMA_Configuration(void)
{
	DMA_DeInit(DMA1_Channel1);//复位DMA通道1
 	DMA_InitStructure.DMA_PeripheralBaseAddr =ADC1_DR_Address;//定义 DMA通道外设基地址=ADC1_DR_Address
 	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue; //定义DMA通道存储器地址
 	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//指定外设为源地址
	DMA_InitStructure.DMA_BufferSize = 4096;//定义DMA缓冲区大小1
 	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//当前外设寄存器地址不变
 	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器地址递增
 	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;//定义外设数据宽度32位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; //定义存储器数据宽度32位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA通道操作模式位为循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA通道优先级高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//禁止DMA通道存储器到存储器传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);//初始化DMA通道1
	
	DMA_ITConfig( DMA1_Channel1,DMA_IT_TC, ENABLE);//时能dma中断
	DMA_Cmd(DMA1_Channel1, ENABLE); //使能DMA通道1
}
/************************************************************
*配置adc1
************************************************************/
void ADC_Configuration(u8 cysj)
{
	ADC_InitTypeDef ADC_InitStructure; 
    //以下是ADC1的寄存器配置
  	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//AD模式选为独立模式
  	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//自动扫描模式使能
  	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换模式使能
  	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//没有中断触发转换
  	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//数据向右对齐
  	ADC_InitStructure.ADC_NbrOfChannel = 1;//初始化ADC通道号数1
  	ADC_Init(ADC1, &ADC_InitStructure);//构建ADC1设备
  	//PA0对应ADC1通道是0
  	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, cysj);
	//使能ADC1模块DMA
	ADC_DMACmd(ADC1, ENABLE);//使能ADC1模块DMA
	//使能ADC1
  	ADC_Cmd(ADC1, ENABLE);
  	//复位ADC1的寄存器  
  	ADC_ResetCalibration(ADC1);
    //等待复位结束 	
  	while(ADC_GetResetCalibrationStatus(ADC1));
  	//开始ADC1校准
  	ADC_StartCalibration(ADC1);
  	//等待ADC1校准结束 	
  	while(ADC_GetCalibrationStatus(ADC1));
    //使能ADC1转换
  	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
void ADC_Restart(u8 cysj)
{
	//PA0对应ADC1通道是0
  	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, cysj);
	//使能ADC1模块DMA
	ADC_DMACmd(ADC1, ENABLE);//使能ADC1模块DMA
	//使能ADC1
  	ADC_Cmd(ADC1, ENABLE);
  	//复位ADC1的寄存器  
  	ADC_ResetCalibration(ADC1);
    //等待复位结束 	
  	while(ADC_GetResetCalibrationStatus(ADC1));
  	//开始ADC1校准
  	ADC_StartCalibration(ADC1);
  	//等待ADC1校准结束 	
  	while(ADC_GetCalibrationStatus(ADC1));
    //使能ADC1转换
  	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
/************************************************************
*配置中断优先级
************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);	 	 
	//配置优先级模式，详情请阅读原材料中的文章
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//配置优先级
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel=ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;//开TIM3中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}
/************************************************************
*配置定时器 50ms 
************************************************************/
void Time_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/*Time base configuration  */
	TIM_TimeBaseStructure.TIM_Period = 100;//50ms
	TIM_TimeBaseStructure.TIM_Prescaler = 35999; //设置预分频器分频系数36000
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//设置了时钟分割 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//选择向上计数

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx 
	TIM_ClearFlag(TIM3, TIM_FLAG_CC1);
	TIM_ARRPreloadConfig(TIM2, DISABLE);
  	/* TIM IT enable */
  	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);//使能定时器的中断

  	/* TIM3 enable counter */
  	TIM_Cmd(TIM3, ENABLE);//使能定时器
}
/************************************************************
*DMA的中断函数
*adc1数据缓冲区装满后 关闭dma中断关闭adc，等待lcd刷新
************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
	DMA_Cmd(DMA1_Channel1, DISABLE);
	ADC_DMACmd(ADC1, DISABLE);

	DMA_ITConfig( DMA1_Channel1,DMA_IT_TC, DISABLE);

	DMA_ClearFlag(DMA1_FLAG_TC1);
	DMA_ClearITPendingBit(DMA1_IT_TC1);
}
/************************************************************
*定时器中断函数
*处理adc缓冲区中的数据 刷新lcd间隔为50毫秒
************************************************************/
void TIM3_IRQHandler(void)
{
	u16 i;//数据位
	u16 qsw_n=0;
	u16 PL_flag=0;//频率标志
	u16 temp = 0;//中间变量
	double MAX_flag=0.0,MIN_flag=0.0;//最大值最小值
	QSW[0]=0;//起始位
	QSW[1]=0;//起始位
	ZZW=0; //转折位
    /////////////////////////////////////////
	//计算峰值
	max=ADC_ConvertedValue[0];
	min=ADC_ConvertedValue[0];
	LCD_Clear(BLACK);
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  	{
		//取得最大值最小值		
		for(i=0;i<4096;i++)
		{
			if(ADC_ConvertedValue[i]>max)
			{
				max=ADC_ConvertedValue[i];		
			}
			if(ADC_ConvertedValue[i]<min)
			{
				min=ADC_ConvertedValue[i];			
			}		
		}
		//////////////////////////////////////////////////////
		//显示最大值
		LCD_ShowString(150,10,200,16,16,"MAX_VAL:0.000v");
		MAX_flag = max*3.3/4096;
		temp = MAX_flag;
		LCD_ShowxNum(214,10,temp,1,16,0);
		MAX_flag-=temp;
		MAX_flag*=1000;
		LCD_ShowxNum(230,10,MAX_flag,3,16,0);
		
		///////////////////////////////////////////////////////
		//显示最小值
		LCD_ShowString(150,30,200,16,16,"MIN VAL:0.000v");
		MIN_flag = min*3.3/4096;
		temp = MIN_flag;
		LCD_ShowxNum(214,30,temp,1,16,0);
		MIN_flag-=temp;
		MIN_flag*=1000;
		LCD_ShowxNum(230,30,MIN_flag,3,16,0);

		average=((max+min)>>1)>>4;

		for(i=4;i<2048;i++)
		{
			//捕捉上升沿
			if((ADC_ConvertedValue[i-1]>>4)<average&&
				(ADC_ConvertedValue[i-4]>>4)<average&&
				(ADC_ConvertedValue[i+1]>>4)>average&&
				(ADC_ConvertedValue[i+4]>>4)>average)
			{
				//找到上升沿			
				if(qsw_n==0)
				{
					//第一个上升沿	
					QSW[0]=i;
					qsw_n++;
				}
				else if(qsw_n==1&&(i-QSW[0]>5))
				{
					//第二个上升沿
					QSW[1]=i;
					//LCD上显示频率
					if(WK_flag==1)//位宽标志为1
					{
						//11kHz以上
						LCD_ShowString(10,10,200,16,16,"WxVEFRE:000000kHz");
						PL_flag=2571428/(QSW[1]-QSW[0]);
												
						PL_flag=PL_flag/10;
						LCD_ShowxNum(114,10,PL_flag%10,1,16,0);

						PL_flag=PL_flag/10;
						LCD_ShowxNum(106,10,PL_flag%10,1,16,0);
						LCD_ShowChar(98,10,'.',16,0);

						PL_flag=PL_flag/10;
						LCD_ShowxNum(90,10,PL_flag%10,1,16,0);

						PL_flag=PL_flag/10;
						LCD_ShowxNum(82,10,PL_flag%10,1,16,0);

						PL_flag=PL_flag/10;
						LCD_ShowxNum(74,10,PL_flag%10,1,16,0);
					}
					if(WK_flag==2)
					{
						//3.4kHz~11kHz
						LCD_ShowString(10,10,200,16,16,"WAxEFRE:000000kHz");
						PL_flag=1384615/(QSW[1]-QSW[0]);

						LCD_ShowxNum(114,10,PL_flag%10,1,16,0);
						PL_flag=PL_flag/10;
						LCD_ShowxNum(106,10,PL_flag%10,1,16,0);
						PL_flag=PL_flag/10;
						LCD_ShowxNum(98,10,PL_flag%10,1,16,0);
						LCD_ShowChar(90,10,'.',16,0);
						PL_flag=PL_flag/10;
						LCD_ShowxNum(82,10,PL_flag%10,1,16,0);
						PL_flag=PL_flag/10;
						LCD_ShowxNum(74,10,PL_flag%10,1,16,0);
					}
					if(WK_flag==3)
					{
						//1kH~3.4kHz
						LCD_ShowString(10,10,200,16,16,"WAVxFRE:000000kHz");
						PL_flag=429642/(QSW[1]-QSW[0]);

						LCD_ShowxNum(114,10,PL_flag%10,1,16,0);
						PL_flag=PL_flag/10;
						LCD_ShowxNum(106,10,PL_flag%10,1,16,0);
						PL_flag=PL_flag/10;
						LCD_ShowxNum(98,10,PL_flag%10,1,16,0);
						LCD_ShowChar(90,10,'.',16,0);
						PL_flag=PL_flag/10;
						LCD_ShowxNum(82,10,PL_flag%10,1,16,0);
						PL_flag=PL_flag/10;
						LCD_ShowxNum(74,10,PL_flag%10,1,16,0);
					}
					if(WK_flag==4)
					{
						//小于1kHz
						LCD_ShowString(10,10,200,16,16,"WAVExRE:000000kHz");
						PL_flag=142857/(QSW[1]-QSW[0]);

						LCD_ShowxNum(114,10,PL_flag%10,1,16,0);
						PL_flag=PL_flag/10;
						LCD_ShowxNum(106,10,PL_flag%10,1,16,0);
						PL_flag=PL_flag/10;
						LCD_ShowxNum(98,10,PL_flag%10,1,16,0);
						LCD_ShowChar(90,10,'.',16,0);
						PL_flag=PL_flag/10;
						LCD_ShowxNum(82,10,PL_flag%10,1,16,0);
						PL_flag=PL_flag/10;
						LCD_ShowxNum(74,10,PL_flag%10,1,16,0);
					}
					break;
				}
				
			}
			//扑捉下降沿
			if((ADC_ConvertedValue[i-1]>>4)>average&&
				(ADC_ConvertedValue[i-4]>>4)>average&&
				(ADC_ConvertedValue[i+1]>>4)<average&&
				(ADC_ConvertedValue[i+4]>>4)<average)
			{
				if(QSW[0]!=0)
					ZZW=i;	
			}
			//捕捉超时
			if(i==2046)
			{
				QSW[0]=0;
				QSW[1]=0;
				ZZW=0;
				WK_flag=4;//检测不到频率，用最长检测周期
				break;	
			}
		
		}
		//装lcd缓存
		if(dma_flag==1)
		{
			for(i=0;i<319;i++)  		
				Lcd_Buff1[i]=240 - ADC_ConvertedValue[i*L_range+QSW[0]]/17.1;	//L_range调宽H_range调高
		}
		else
		{
			for(i=0;i<319;i++)  		
				Lcd_Buff2[i]=240 - ADC_ConvertedValue[i*L_range+QSW[0]]/17.1;	
		}	
		
	    //刷新lcd
		if(dma_flag==1)
		{
//			for(i=0;i<319;i++) 
//			{
//				POINT_COLOR = DARKBLUE;
//				LCD_DrawLine(i,Lcd_Buff2[i],i+1,Lcd_Buff2[i+1]);	
//			} 		
			for(i=0;i<319;i++)  
			{
				POINT_COLOR = RED;
				LCD_DrawLine(i,Lcd_Buff1[i],i+1,Lcd_Buff1[i+1]);
				printf("%i,",Lcd_Buff1[i]);
			}		
			dma_flag=2;
		}
		else
		{
//			for(i=0;i<319;i++)
//			{
//				POINT_COLOR = DARKBLUE;
//				LCD_DrawLine(i,Lcd_Buff1[i],i+1,Lcd_Buff1[i+1]);
//			}  		
			for(i=0;i<319;i++)
			{
				POINT_COLOR = RED;
				LCD_DrawLine(i,Lcd_Buff2[i],i+1,Lcd_Buff2[i+1]);
			}  		
			dma_flag=1;
		}		
		
		//自适应波形频率 频率差是为了不影响临界值的正常显示	
		//频率小于7.3KHz
		if(WK_flag==1)
		{
			if(QSW[1]-QSW[0]>350)
			{
				WK_flag++;
				ADC_Configuration(ADC_SampleTime_13Cycles5);
			}
		}
		
		if(WK_flag==2)
		{	
			//频率小于2.7k 采样时间延长
			if(QSW[1]-QSW[0]>512)
			{
				WK_flag++;
				ADC_Configuration(ADC_SampleTime_71Cycles5);
			}
			//频率大于11khz，减少采样时间
			if(QSW[1]-QSW[0]<126)
			{
				WK_flag--;
				ADC_Configuration(ADC_SampleTime_1Cycles5);
			}
		}
		
		if(WK_flag==3)
		{
			//频率小于860Hz采样时间延长
			if(QSW[1]-QSW[0]>498)
			{
				WK_flag++;
				ADC_Configuration(ADC_SampleTime_239Cycles5);
			}
			//频率大于3.4khz，减少采样时间
			if(QSW[1]-QSW[0]<127)
			{
				WK_flag--;
				ADC_Configuration(ADC_SampleTime_13Cycles5);
			}			
		}
		//频率大于1k，减少采样时间
		if(WK_flag==4)
		{
			if(QSW[1]-QSW[0]<142)
			{
				WK_flag--;
				ADC_Configuration(ADC_SampleTime_71Cycles5);
			}
		}
		//恢复dma adc
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	  	DMA_Cmd(DMA1_Channel1, ENABLE);
		DMA_ITConfig( DMA1_Channel1,DMA_IT_TC, ENABLE);	
		ADC_DMACmd(ADC1, ENABLE); 
	}
	else
	{
		
	}
}

