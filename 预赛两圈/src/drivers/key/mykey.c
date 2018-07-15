#include "mykey.h"

void myKEY_Init(void)
{
  gpio_init(PORTC,8,GPI,HIGH);
  port_init_NoALT(PTC8,PULLUP);
  gpio_init(PORTC,6,GPI,HIGH);
  port_init_NoALT(PTC6,PULLUP);
}

void myKEY_Exti_Init(void)
{
   exti_init(PORTB,1,falling_up);
   exti_init(PORTA,17,falling_up);
   exti_init(PORTA,15,falling_up);
}

u8 KEY_Scan(void)
{
  static u8 key_up=1;//按键按松开标志	  
  if(key_up&&(KEY1==0||KEY2==0))
  {
    delayms(10);//去抖动 
    key_up=0;
    if(KEY1==0)                  return KEY1_PRES;
    else if(KEY2==0)            return KEY2_PRES;
  }
  else if(KEY1==1&&KEY2==1)
  {
    key_up=1; 	    
  }
  return 0;// 无按键按下
}
