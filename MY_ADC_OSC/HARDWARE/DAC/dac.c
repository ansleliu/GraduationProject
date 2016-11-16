#include "dac.h"
//////////////////////////////////////////////////////////////////////////////////	 								  
//DAC通道1输出初始化
#define DAC_DHR12RD_Address      ((u32)0x40007420)
/* 波形数据 ---------------------------------------------------------*/
uc16 Sine12bit[32] = {2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056,
                      3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909, 
                      599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647};

u32 DualSine12bit[32];

/**
  * @brief  使能DAC的时钟，初始化GPIO
  * @param  无
  * @retval 无
  */
static void DAC_Config(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );	  //使能PORTA通道时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );	  //使能DAC通道时钟 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;;	// 端口配置
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 		 //模拟输入
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
					
	DAC_InitType.DAC_Trigger = DAC_Trigger_T2_TRGO;	//使用TIM2作为触发源
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//屏蔽、幅值设置
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;//DAC1输出缓存关闭 BOFF1=1
    DAC_Init(DAC_Channel_1,&DAC_InitType);//初始化DAC通道1
    DAC_Init(DAC_Channel_2, &DAC_InitType);//初始化DAC通道2
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值
	DAC_SetChannel2Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值
 
    DAC_Cmd(DAC_Channel_1, ENABLE);//使能通道1 由PA4输出
    DAC_Cmd(DAC_Channel_2, ENABLE);//使能通道2 由PA5输出
    
    DAC_DMACmd(DAC_Channel_2, ENABLE);//使能DAC的DMA请求
	DAC_DMACmd(DAC_Channel_1, ENABLE);
}

//设置通道1输出电压
//vol:0~3300,代表0~3.3V
void Dac1_Set_Vol(u16 vol)
{
	float temp=vol;
	temp/=1000;
	temp=temp*4096/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12位右对齐数据格式设置DAC值
}

static void DAC_TIM_Config(void)
{
	
  TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	
  // 使能TIM2时钟，TIM2CLK 为72M//
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
  // TIM2基本定时器配置/
  // TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 1;//定时周期 
  TIM_TimeBaseStructure.TIM_Prescaler = 7199; //预分频，不分频 72M / (0+1) = 72M
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; //时钟分频系数
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* 配置TIM2触发源 */
  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

	/* 使能TIM2 */
  TIM_Cmd(TIM2, ENABLE);

}

/**
  * @brief  配置DMA
  * @param  无
  * @retval 无
  */
static void DAC_DMA_Config(void)
{	
	DMA_InitTypeDef  DMA_InitStructure;

	/* 使能DMA2时钟 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	// DMA channel1 configuration
	DMA_DeInit(DMA1_Channel4);
	/* 配置DMA2 */
  	DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12RD_Address;	//外设数据地址
  	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&DualSine12bit ;//内存数据地址 DualSine12bit
  	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//数据传输方向内存至外设
  	DMA_InitStructure.DMA_BufferSize = 32;//缓存大小为32字节	
  	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设数据地址固定	
  	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//内存数据地址自增
  	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;	//外设数据以字为单位
  	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;//内存数据以字为单位	
  	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	//循环模式
  	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//高DMA通道优先级
  	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//非内存至内存模式	

  	DMA_Init(DMA2_Channel4, &DMA_InitStructure);
	
    /* 使能DMA2-14通道 */
    DMA_Cmd(DMA2_Channel4, ENABLE);
}


/**
  * @brief  DAC初始化函数
  * @param  无
  * @retval 无
  */
void DAC_Mode_Init(void)
{
	u8 Idx = 0;  
	RCC_Configuration();
	DAC_TIM_Config();
	DAC_Config();
	DAC_DMA_Config();
	
	/* 填充正弦波形数据，双通道右对齐*/
  for (Idx = 0; Idx < 32; Idx++)
  {
    DualSine12bit[Idx] = (Sine12bit[Idx] << 16) + (Sine12bit[Idx]);
  }
}
