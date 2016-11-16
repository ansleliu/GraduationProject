#include "led.h"
#include "delay.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "adc.h"
/////////////////////////////////////
// ADC转化实验
// 采集电压并转换
// 2014.05.14
/////////////////////////////////////
// ADC1转换的电压值通过MDA方式传到SRAM
extern __IO uint16_t ADC_ConvertedValue;
// 局部变量，用于保存转换计算后的电压值 	 
//float ADC_ConvertedValueLocal;
 int main(void)
 {	 
  	u16 adcx;
	float temp;
	delay_init();	    	 //延时函数初始化	  
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200
 	LED_Init();			     //LED端口初始化
	LCD_Init();			 	 //LCD端口初始化

	POINT_COLOR=RED;//设置字体为红色 
	LCD_ShowString(60,50,200,16,16,"STM32ZET6");	
	LCD_ShowString(60,70,200,16,16,"ADC TEST");	
	LCD_ShowString(60,90,200,16,16,"Ansleliu");
	LCD_ShowString(60,110,200,16,16,"2014/05/12");	
	//显示提示信息
	POINT_COLOR=BLUE;//设置字体为蓝色
	LCD_ShowString(60,130,200,16,16,"ADC_CH0_VAL:");	      
	LCD_ShowString(60,150,200,16,16,"ADC_CH0_VOL:0.000V");

	Adc_Init();		  		//ADC初始化

	while (1)
	{
		adcx=ADC_ConvertedValue;
//		ADC_ConvertedValueLocal =(float) adcx/4096*3.3; // 读取转换的AD值
		/////////////////////////////////////////////////
		//显示
		//串口传输
		printf("%i,",adcx);
		//显示ADC的值
		LCD_ShowxNum(156,130,adcx,4,16,0);
		//显示电压值
		temp=(float)adcx*(3.3/4096);
		adcx=temp;
		LCD_ShowxNum(156,150,adcx,1,16,0);
		temp-=adcx;
		temp*=1000;
		LCD_ShowxNum(172,150,temp,3,16,0X80);
		//////////////////////////////
		//指示灯
		LED0=!LED0;
		delay_ms(2);
	}
 }

