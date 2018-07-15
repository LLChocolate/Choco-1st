#include "dial_switch.h"
#include "port.h"

void dial_switch_Init(void)
{
  gpio_init(PORTB,0,GPI,LOW);
  gpio_init(PORTB,1,GPI,LOW);
  gpio_init(PORTB,2,GPI,LOW);
  gpio_init(PORTB,3,GPI,LOW);
  gpio_init(PORTB,4,GPI,LOW);
  gpio_init(PORTB,5,GPI,LOW);
  gpio_init(PORTB,6,GPI,LOW);
  gpio_init(PORTB,7,GPI,LOW);

}

u8 dial_switch_Scan(void)
{
  u8 status=0,i;
  i=SW1;
  status|=i<<0;
  i=SW2;
  status|=i<<1;
  i=SW3;
  status|=i<<2;
  i=SW4;
  status|=i<<3;
  i=SW5;
  status|=i<<4;
  i=SW6;
  status|=i<<5;
  i=SW7;
  status|=i<<6;
  i=SW8;
  status|=i<<7;
  return status;
}
