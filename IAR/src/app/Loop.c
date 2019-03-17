#include "include.h"
#include "String.h"
#include "fsl_debug_console.h"
void Setup(void)
{
  DisableInterrupts;
  Ov7725_exti_Init();
  GPIO_Init(motor_left_DIR, GPO, HIGH);
  GPIO_Init(motor_right_DIR, GPO, HIGH);
  GPIO_Init(mode_1, GPI, HIGH);
  GPIO_Init(mode_2, GPI, HIGH);
  GPIO_Init(mode_3, GPI, HIGH);
  GPIO_Init(mode_4, GPI, HIGH);
  FTM_PWM_init(motor_left_FTM,1000,10000);
  FTM_PWM_init(motor_right_FTM,1000,10000);
  FTM_PWM_init(steer_FTM,steer_FTM_middle,1900);
  LCD_PORT_init();
  EnableInterrupts;
}

void Loop(void)
{
  
}