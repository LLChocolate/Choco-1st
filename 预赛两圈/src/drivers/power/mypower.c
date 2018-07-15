#include "mypower.h"

u8 myPower_Init(void)
{
  adc_init(MYADC_3);
  return 0;
}

u8 myPower_Test(void)
{
  static u8 Beep_cnt = 0;
  Real_AD = ad_mid(MYADC_3,ADC_12bit)*3.3/4096;
  Run_.Volts_now = Real_AD*3.75;
  if(Run_.Volts_now > 5
     &&Beep_cnt==0)
  {
    Beep_Once(&Test_100ms);
    Beep_cnt++;
  }
  
  if(Run_.Volts_now > Run_.Run_Volts)
    return 1;
  else 
    return 0;
}
