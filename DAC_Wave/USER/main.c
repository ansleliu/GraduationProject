#include "led.h"
#include "delay.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 	 
#include "dac.h"
/////////////////////////////////////////////////////////
//
/*
#define N 5
extern __IO uint16_t ADC_ConvertedValue;
extern vu16 ADC_RegularConvertedValueTab[32], ADC_InjectedConvertedValueTab[32]; 
u16 adcx;
int xpoint=-N;
int ypoint=0;
int oldy=0;
// Private variables ---------------------------------------------------------//
ErrorStatus HSEStartUpStatus;
///////////////////////////////////////////
//
void NVIC_Configuration(void);
void RCC_Configuration(void);
*/
#define N 5
vu16 ADC_RCVTab[400] ;	        //DMA传输目的地址
     vu16 ADC_RCVTab1[400];	       //暂存空间
   	 float a=10	;                 // 调节波形X疏密变量
	 int p=1;		             // 调节波形Y疏密变量
     uint32_t Capture = 0;	   	// 捕获值
     uint32_t TIM2Freq = 0;    // 频率
	 vu16 TIM2_RCVTab[2] ;	  //定时器DMA 传输目的地址用于测频率
	 uint16_t s0,s1;		 //暂存变量

 int main(void)
 {	 
 	//初始变量---------------------------------------------------------------------------/ 
    int x=-N,y,i=0,t=0,SUM=0,AVERAGE=0,H,L; //X,Y为坐标，i为标志量，t为中间量，SUM，AVERAGE,H,L为和平均值，最大值，最小值
    float AVERAGE1=0,H1=0,L1=0;		      //中间量，实际值
	char string[50] = ""; 				 //中间字符串

	//系统时钟初始化，外设时钟开启，GPIO配置，中断配置-----------------------------------/ 
    SysClock_Init();
	RCC_Configuration();              //外设时钟使能
	GPIO_Configuration();
	NVIC_Configuration();
	EXTI_Configuration() ;
	//外设配置，DMA,ADC,TIM,SPI,TOUCH----------------------------------------------------/
	DMA_USERConfiguration(); //DMA使能配置	 用于传输波形数据的采样数据，以及定时器的捕获值
	ADC_Configuration();    //ADC配置  采样400个数据
	TIM_Configuration()	;  //定时器配置，测频率

	SPI_Configuration();  // MCU通过SPI1与触摸屏通信，在此进行 SPI配置
	////////////////////////////////////////////////////////////////////
	//
	delay_init();	    	 //延时函数初始化
//	RCC_Configuration();
//	NVIC_Configuration();	  
//	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(9600);	 	//串口初始化为9600
 	LED_Init();			     //LED端口初始化
	LCD_Init();			 	 //LCD初始化
	LCD_Clear(BLACK);
	LCD_Display_Dir(1);

	POINT_COLOR=RED;
		
	//初始化DAC，开始DAC转换//
	DAC_Mode_Init();
//	Adc_Init();	//ADC初始化
	while(1)
	{
	//循环开始 允许ADC的DMA请求，等待400个数据传输完成，关闭传输，再画波形------------/
	    ADC_DMACmd(ADC1, ENABLE);      //允许ADC的DMA 请求
		t=ADC_RCVTab[399];
	  	while(t==ADC_RCVTab[399]);	 //等待传输完成
		ADC_DMACmd(ADC1, DISABLE); 	//关闭传输


        //画波形，-----------------------------------------------------------------------/


		for(i=0;i<390;i=i+1)	  
		{
			 x+=N;
			if(x >= 320)
			{
				x = 0;
				LCD_Clear(BLACK);
			}
			//显示
			printf("%i,",ADC_RCVTab[i]);
			y = 240 - (ADC_RCVTab[i]/17.1);
//			LCD_Fast_DrawPoint(x,y,BLUE);
			LCD_DrawLine(x,240 - (ADC_RCVTab1[i-1]/17.1),x+N,y);
			delay_us(100);
			ADC_RCVTab1[i]=y;//保存要清除的值
			//////////////////////////////
			//指示灯
			LED0=!LED0;
		}
/*
		     x=i*p+81;	                                //确定x
			 if(x<470)
			    {
			      y=160+(int)((a/10)*(-95+ADC_RCVTab[i]*160/2048));	 //确定y  98对应偏移电压为1V的情况
				  ypoint = 240 - (adcx/17.1);
		     	  LCD_Fast_DrawPoint(xpoint,ypoint,BLUE);
		          LCD_DrawLine(xpoint,oldy,xpoint+N,ypoint);
//		   	      SetPixel(x,y);					//画点
		   	      ADC_RCVTab1[i]=y;				   //保存要清除的值
//				  Delay(30000);

			    }
			 else i=390;

		    }


        //求最大值，最小值，平均值---------------------------------------------------------/


		for(i=0,SUM=0,H=0,L=0xffff;i<400;i++)
		  {
		   SUM=SUM+ADC_RCVTab[i];            //求和
		  
		   if(ADC_RCVTab[i]>H)			   //求最大值
		      H=ADC_RCVTab[i];
										 //求最小值
			if(ADC_RCVTab[i]<L)
			L=ADC_RCVTab[i];

		  }


		   
		   AVERAGE=SUM/400;
		   AVERAGE1=((AVERAGE*3.27/4096*1000+60)-1050)*3;              //求平均值实际值
		   H1=(((H)*3.27/4096*1000+100)-1030)*3;			          //求最大值实际值
		   L1=(((L)*3.27/4096*1000-20)-1000)*3;		                 //求最小值实际值

		   sprintf(string, "  %6.1f", AVERAGE1);                     //写平均值
//		   PutsXY(10,105,string);			 

		   sprintf(string, "  %6.1f", H1);		                //写最大值		
//		   PutsXY(10,80,"MAX:    ");
//		   PutsXY(10,55,string);

		   sprintf(string, " %6.1f", L1);	                 //写最小值
//		   PutsXY(10,30,"MIN:    ");
//		   PutsXY(10,5,string);

		   sprintf(string,"%6.1f", (H1-L1));	         //写峰峰值
//		   PutsXY(10,205,string);


		   
           //等待DMA从TIM获取的值全部更新，计算频率-------------------------------------------/


		   s0=TIM2_RCVTab[0];
		   s1=TIM2_RCVTab[1] ;
           while(s1==TIM2_RCVTab[1]) ;                                 //全部更新
                
		   if(TIM2_RCVTab[0]<TIM2_RCVTab[1])				    
             Capture = (TIM2_RCVTab[1] - TIM2_RCVTab[0]); 	         //在一个定时器周期内
				  
		   else
		     Capture = ((0xFFFF- TIM2_RCVTab[0]) + TIM2_RCVTab[1]);//处於两个周期边界 
				   		   
		   TIM2Freq = (uint32_t)7200000 / Capture;                // 计算频率
		   sprintf(string, "%6d", TIM2Freq);
//		   PutsXY(10,255,string);	


		   //延时保持波形，再擦出波形，回到循环初始，重新画波----------------------------------/

		  // Delay(800000); //延时保持波形
		   
		   	for(i=0;i<390;i=i+1)	 //消除波形
		  {
		     
		     x=i*p+81;
			 if(x<470)
			 {
			 y=ADC_RCVTab1[i];
//		  	 ErasePixel(x,y) ;
//			 Draw_horizontal_xuxian(80,470,160);
			  }
			else  i=390;
	    } 
*/		    
	}  
	////////////////////////////////////////////////////
	//
/*
  	while(1)
	{
		xpoint+=N;
		if(xpoint >= 320)
		{
			xpoint = 0;
			LCD_Clear(BLACK);
		}
		adcx=ADC_ConvertedValue;
		//显示
		ypoint = 240 - (adcx/17.1);
//		LCD_Fast_DrawPoint(xpoint,ypoint,BLUE);
		LCD_DrawLine(xpoint,oldy,xpoint+N,ypoint);
		delay_us(1.17);
		//////////////////////////////
		//指示灯
		LED0=!LED0;
		oldy = ypoint;
	}
*/		   	      		
 }
/*
void ADC1_2_IRQHandler()
{
  int i	=0 ;
  ADC_ITConfig(ADC1,ADC_IT_JEOC,DISABLE);

  for(;i<32;i++)
  {
  		xpoint+=N;
		if(xpoint >= 320)
		{
			xpoint = 0;
			LCD_Clear(BLACK);
		}
		adcx=ADC_RegularConvertedValueTab[i];
		//显示
		ypoint = 240 - (adcx/17.1);
//		LCD_Fast_DrawPoint(xpoint,ypoint,BLUE);
		LCD_DrawLine(xpoint,oldy,xpoint+N,ypoint);
		//////////////////////////////
		//指示灯
		LED0=!LED0;
		oldy = ypoint;
  }
  ADC_ITConfig(ADC1,ADC_IT_JEOC,ENABLE);
}
/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
void RCC_Configuration(void)
{   
  /* RCC system reset(for debug purpose) //
  RCC_DeInit();

  /* Enable HSE //
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready //
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer //
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state //
    FLASH_SetLatency(FLASH_Latency_2);
	
    /* HCLK = SYSCLK//
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK //
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 //
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* ADCCLK = PCLK2/4 //
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
  
    /* PLLCLK = 8MHz * 9 = 56 MHz//
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);

    /* Enable PLL // 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready//
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source //
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source //
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}
/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures NVIC and Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM  
  // Set the Vector Table base location at 0x20000000// 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else // VECT_TAB_FLASH //
  // Set the Vector Table base location at 0x08000000 //
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif

  //Enable ADC1_2 IRQChannel //
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
 */
