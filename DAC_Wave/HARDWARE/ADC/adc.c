 #include "adc.h"
 #include "dac.h"
/*
 /////////////////////////////////////////////////////////
 //
 #define ADC1_DR_Address    ((u32)0x4001244C)
//////////////////////////////////////////////////////
//
ADC_InitTypeDef           ADC_InitStructure;
DMA_InitTypeDef           DMA_InitStructure;
TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
TIM_OCInitTypeDef         TIM_OCInitStructure;
vu16 ADC_RegularConvertedValueTab[32], ADC_InjectedConvertedValueTab[32];
//ErrorStatus HSEStartUpStatus;

////////////////////////////////////////////////////
//
void Adc_Init(void)
{
///* Enable GPIOA, GPIOC, ADC1 and TIM1 clock//
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC |
                         RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1, ENABLE);
//  /* System clocks configuration ---------------------------------------------//
//  RCC_Configuration();
  /* NVIC configuration ------------------------------------------------------//
//  NVIC_Configuration();
//  /* GPIO configuration -----------------------------------------------------//
  GPIO_Configuration();

//  /* TIM1 configuration -----------------------------------------------------// 
//  /* Time Base configuration //
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 0xFF;          
  TIM_TimeBaseStructure.TIM_Prescaler = 0x4;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  /* TIM1 channel1 configuration in PWM mode //
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
  TIM_OCInitStructure.TIM_Pulse = 0x7F; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;         
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);


  /// Enable DMA1 clock//
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  // DMA1 Channel1 Configuration ----------------------------------------------//
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_RegularConvertedValueTab;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 32;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  ///* Enable DMA1 channel1 //
  DMA_Cmd(DMA1_Channel1, ENABLE);

  // ADC1 configuration ------------------------------------------------------//
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ///ADC1 regular channel14 configuration // 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_13Cycles5);

  /// Set injected sequencer length //
  ADC_InjectedSequencerLengthConfig(ADC1, 1);
  // ADC1 injected channel Configuration // 
  ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_71Cycles5);
  // ADC1 injected external trigger configuration //
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);

  //Enable automatic injected conversion start after regular one //
  ADC_AutoInjectedConvCmd(ADC1, ENABLE);

  //Enable ADC1 DMA //
  ADC_DMACmd(ADC1, ENABLE);

  //Enable ADC1 external trigger // 
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);

  //Enable JEOC interupt //
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);

  // Enable ADC1 //
  ADC_Cmd(ADC1, ENABLE);

  // Enable ADC1 reset calibaration register //   
  ADC_ResetCalibration(ADC1);
  // Check the end of ADC1 reset calibration register //
  while(ADC_GetResetCalibrationStatus(ADC1));

  //Start ADC1 calibaration ///
  ADC_StartCalibration(ADC1);
  /Check the end of ADC1 calibration //
  while(ADC_GetCalibrationStatus(ADC1));

  //TIM1 counter enable //
  TIM_Cmd(TIM1, ENABLE);
  //TIM1 main Output Enable ///
  TIM_CtrlPWMOutputs(TIM1, ENABLE);  

  ///Test on channel1 transfer complete flag //
  while(!DMA_GetFlagStatus(DMA1_FLAG_TC1));
  //Clear channel1 transfer complete flag //
  DMA_ClearFlag(DMA1_FLAG_TC1); 

  //TIM1 counter disable //
  TIM_Cmd(TIM1, DISABLE);
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
  // RCC system reset(for debug purpose) 
  RCC_DeInit();

  // Enable HSE 
  RCC_HSEConfig(RCC_HSE_ON);

  //* Wait till HSE is ready //
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    //* Enable Prefetch Buffer //
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    //* Flash 2 wait state //
    FLASH_SetLatency(FLASH_Latency_2);
	
    //* HCLK = SYSCLK //
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    //* PCLK2 = HCLK //
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    //* PCLK1 = HCLK/2 //
    RCC_PCLK1Config(RCC_HCLK_Div2);

   ///* ADCCLK = PCLK2/4 //
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
  
    //* PLLCLK = 8MHz * 7 = 56 MHz///
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);

    //* Enable PLL //
    RCC_PLLCmd(ENABLE);

    //* Wait till PLL is ready//
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    //* Select PLL as system clock source //
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    //* Wait till PLL is used as system clock source //
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
  
///* Enable peripheral clocks -------------------------------------------------//
  ///* Enable DMA1 clock//
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  ///* Enable GPIOA, GPIOC, ADC1 and TIM1 clock//
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC |
                         RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1, ENABLE);
}
*/
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure TIM1_CH1 (PA8) as alternate function push-pull //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure PC.06 as output push-pull //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // Configure PA.00 and PA.01 (ADC Channel0 and Channel1) as analog input //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*
//*******************************************************************************
//* Function Name  : NVIC_Configuration
//* Description    : Configures NVIC and Vector Table base location.
//* Input          : None
//* Output         : None
//* Return         : None
//////////////////////////////////////////////////////////////////////////////////
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
/*******************************************************************************

        初始化时钟晶振 72MHZ

*******************************************************************************/


void SysClock_Init(void)
{
	ErrorStatus HSEStartUpStatus;     
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus == SUCCESS)
	{
    	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_Latency_2); 
	    RCC_HCLKConfig(RCC_SYSCLK_Div1); 	  
	    RCC_PCLK2Config(RCC_HCLK_Div1); 	
	    RCC_PCLK1Config(RCC_HCLK_Div2);	
		RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_2);	
	    RCC_PLLCmd(ENABLE);	
	    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
	    	;
		}	
	    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);	
	    while(RCC_GetSYSCLKSource() != 0x08)
		{
	    	;
		}
	}
}


/***********************************************************************

	 外设时钟使能


************************************************************************/


void RCC_Configuration(void)
{	
  	/*  使能外设时钟 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO  | RCC_APB2Periph_ADC1  | 
						   RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | 
						   RCC_APB2Periph_GPIOD  | RCC_APB2Periph_GPIOE | RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}



/*******************************************************************************

        全部中断在此配置

*******************************************************************************/



void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    //-- Configure the NVIC Preemption Priority Bits-----//  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
    // -----------------Enable the EXTI4 Interrupt------------ //
   
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	 
   
}


/*******************************************************************************

        外部中断在此配置

*******************************************************************************/
void EXTI_Configuration(void)
{
    EXTI_InitTypeDef EXTI_InitStructure; //定义结构体


	
   //-------------------------------触控屏按下中断-------------------------------//
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);	  // S4中断	  端口映射PA8
	EXTI_InitStructure.EXTI_Line = EXTI_Line4;	 //选择中断线8
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	  //选择中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;	//使能
    EXTI_Init(&EXTI_InitStructure);
   	   

}

/*******************************************************************************
       
        全部用到的引脚将在在配置

*******************************************************************************/


void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	
	//-----------PB0， 作为模拟通道8输入引脚  AD采样-----------------//                     
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		
	   GPIO_Init(GPIOC, &GPIO_InitStructure);	

	


    //---------------TIM2  CH2		 测频率------------------------//
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
       GPIO_Init(GPIOA, &GPIO_InitStructure);  	

   
	//----------------使用于触控-----------------------------------------------------//
  	 
               //Configure SPI1 pins: SCK, MISO and MOSI 	   SPI1 
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5  | GPIO_Pin_7 | GPIO_Pin_6; //5 主设备 时钟 6 输入 7 输出
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//必须设为此模式具体原因我也不知  6号脚输入亦可
       GPIO_Init(GPIOA, &GPIO_InitStructure);


               //  触控中断 PC4
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOC,&GPIO_InitStructure);

              //  触控片选 PC6
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
   	
}
 //定义源地址或目的地址
#define   ADC1_DR_Address    ((u32)0x4001244C)
#define   Timer2_CC2_DR_Base 0x40000038
#define   SPI1_DR_Address    0x4001300C	//SPI1 数据寄存器地址
/****************************************************************************
* 名    称：DMA_USERConfiguration(vu16*tab,vu16 s) 
* 功    能：DMA初始化
* 入口参数：vu16*tab,vu16 s
* 出口参数：无
* 说    明：tab 是目的地址或源地址
* 调用方法：
*****************************************************************************/
extern vu16 TIM2_RCVTab[2] ; 
extern vu16 ADC_RCVTab[400] ;//示波器为400
 
/////////////////////////////////////ADC_DMA///////////////////////////////////////////////////////////
void DMA_USERConfiguration(void)
{	

    DMA_InitTypeDef DMA_InitStructure;

    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	                          	//源地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_RCVTab;			                       //目的地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	                                  //单向
    DMA_InitStructure.DMA_BufferSize =400;	//示波器为400，频谱仪用1024                  //发送s个数据
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                    //设备自增禁止
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                            //目的地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	                                //连续模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                            //高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);


     
    DMA_Cmd(DMA1_Channel1, ENABLE);	               //通道使能	 
   // ADC_DMACmd(ADC1, ENABLE);                     //允许外设DMA请求


/////////////////////////////////////TIM_DMA///////////////////////////////////////////////////////////

    DMA_DeInit(DMA1_Channel7); //预初始化	通道数要通过查用户手册来确定
	 
    DMA_InitStructure.DMA_PeripheralBaseAddr =Timer2_CC2_DR_Base;                //DMA源地址，相应外设寄存器地址，可以通过查用户手册查找寄存器映射（外设地址+偏移地址）
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)TIM2_RCVTab;                    //DMA传送目的地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                         //传输方向 单向
    DMA_InitStructure.DMA_BufferSize = 2;                                     //传送的数 依次放在 目的地址开始的两个寄存器
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;         //外设增模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	                //目的地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//半字传送
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	      //半字传送
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	                         //循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                    //优先级高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);
  
    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel7, ENABLE);  //通道使能

}

 /****************************************************************************
* 名    称：ADC_Configuration(void)
* 功    能：ADC初始化
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：
*****************************************************************************/

void ADC_Configuration(void)
{
  ADC_InitTypeDef ADC_InitStructure;

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                    //独立模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;                        // 通道扫描及多通道模式
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;		          //连续模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//没有外部触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;			//右对齐
  ADC_InitStructure.ADC_NbrOfChannel = 1;						   //通道数
  ADC_Init(ADC1, &ADC_InitStructure);

    
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_1Cycles5); //开通道0顺序为1

   
  ADC_Cmd(ADC1, ENABLE);   //使能ADC


  
  
  ADC_ResetCalibration(ADC1); // Enable ADC1 reset calibaration register //    
  while(ADC_GetResetCalibrationStatus(ADC1));   // Check the end of ADC1 reset calibration register // 
  ADC_StartCalibration(ADC1);  // Start ADC1 calibaration //  
  while(ADC_GetCalibrationStatus(ADC1)); // Check the end of ADC1 calibration //     
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);  //软件开启ADC1   // Start ADC1 Software Conversion // 


}



/*****************************************************************

         定时器在此配置		输入捕获模式


*****************************************************************/

void TIM_Configuration(void)
{
   TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
   TIM_ICInitTypeDef          TIM_ICInitStructure;


   TIM_DeInit(TIM2);//复位TIM2定时器


 //----------------下面是基本的定时器2的配置函数-------------------------------//
      
   TIM_TimeBaseStructure.TIM_Period =0xFFFF; //  记满
   TIM_TimeBaseStructure.TIM_Prescaler = 9;  //  定时器预分频		 定时时间=（0xffff+1）/72000000*(9+1)
   TIM_TimeBaseStructure.TIM_ClockDivision = 0;//TIM_CKD_DIV1;时钟分割 定时器时钟除以分割数再除相应的分频数作为滤波器 时钟详见http://www.doc88.com/p-684602058911.html 
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //计数方向向上计数
   TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);							
   
   
 
 //------------------------接下来是关键的捕获输入的配置函数--------------------//
  
   TIM_ICStructInit(&TIM_ICInitStructure);//设置为缺省值  捕获   1通道   上升沿   直接方向
   TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;		 //选择通道2
   TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;	//	下降沿
   TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;// 直接方向
   TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;  //每捕获一个下降沿，执行一次中断
   TIM_ICInitStructure.TIM_ICFilter = 0x0; //滤波器采样N个同样的有效电平输出该电平，滤波器输出的电平直接作为捕获的源，此处N为1
   TIM_ICInit(TIM2, &TIM_ICInitStructure);// 初始化
  
  
   TIM_Cmd(TIM2, ENABLE);	//定时器使能
   TIM_DMACmd(TIM2,TIM_DMA_CC2, ENABLE);//定时器请求DMA使能
  
  // TIM_ITConfig(TIM2, TIM_IT_CC2,  ENABLE);//中断使能

   

}


/*******************************************************************************
       
        SPI配置

*******************************************************************************/
 
 void SPI_Configuration(void)
 {
    
        SPI_InitTypeDef SPI_InitStructure;  

        //------------------------ SPI1 configuration ------------------------//
        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI_Direction_1Line_Tx;
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
        SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;/* 72M/64=1.125M */
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_InitStructure.SPI_CRCPolynomial = 7;

        SPI_I2S_DeInit(SPI1);
        SPI_Init(SPI1, &SPI_InitStructure);

        // Enable SPI_MASTER //
        SPI_Cmd(SPI1, ENABLE);
        SPI_CalculateCRC(SPI1, DISABLE);
    
 
 }