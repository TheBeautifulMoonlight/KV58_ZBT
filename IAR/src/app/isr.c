/*
* isr.c
*
*  Created on: Feb 18, 2018
*      Author: ZBT
*/

#include "include.h"
#include "isr.h"
void Error_handler(unsigned char* Log)  //错误处理，ASSERT和HARDFAULT都会掉入此函数
{
  DisableInterrupts;
  //紧急处理
  
  while(1);//终止程序运行
}

void HardFault_Handler(void)
{
  Error_handler("HardFault");
}

//中断定义区

void VSYNC_IRQ(void)
{
  u32 flag = VSYNC_ISFR;
  VSYNC_ISFR=~0;//清中断
  if(flag & (1<<2))
  {
    ov7725_get_img();
  }
}