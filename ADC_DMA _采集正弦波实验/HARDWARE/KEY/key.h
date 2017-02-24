#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
/////////////////////////////////////////////////////////////////
#define KEY0  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)//读取按键0
#define KEY1  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)//读取按键1
#define KEY2  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)//读取按键2 
#define KEY3  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)//读取按键3


#define KEY_UP 		1
#define KEY_DOWN	2
#define KEY_ADD	    3
#define KEY_MINUS	4
void KEY_Init(void);//IO初始化
u8 KEY_Scan(u8);  	//按键扫描函数
					    
#endif
