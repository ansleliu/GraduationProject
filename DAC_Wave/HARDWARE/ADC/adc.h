#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//ADC 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/7
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//void Adc_Init(void);
//void RCC_Configuration(void);
//void GPIO_Configuration(void);
//void NVIC_Configuration(void);
void DMA_USERConfiguration(void);
void ADC_Configuration(void);
void TIM_Configuration(void);
void SysClock_Init(void); //系统时钟配置72M
void RCC_Configuration(void);//外设时钟使能
void NVIC_Configuration(void);//中断配置
void GPIO_Configuration(void); //GPIO 配置
void EXTI_Configuration(void);//外部中断 
#endif 
