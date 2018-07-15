#include "led.h"

void myLED_Init(void)
{
  gpio_init(PORTD,0,GPO,HIGH);
  gpio_init(PORTD,2,GPO,HIGH);
}


void LED_Open_once(void)
{
  LED1=1;
  delayms(10);
  LED1=0;
}
