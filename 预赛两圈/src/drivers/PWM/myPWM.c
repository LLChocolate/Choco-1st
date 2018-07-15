#include "myPWM.h"
void Motot1_Init(void)
{
  gpio_init(PORTA,13,GPO,HIGH);//DIR
  FTM_PWM_init(MOTOR_1,1000,50000);//设置频率为10k
}
void Motot2_Init(void)
{
  gpio_init(PORTE,26,GPO,HIGH);//DIR
  FTM_PWM_init(MOTOR_2,1000,65530);//设置频率为10k
}

void Servo_Init(void)
{
  FTM_PWM_init(STEER_,50,SERVO_MIDDLE);//初始0.5ms高电平旋转0度
}

void PWM_DISENABLE(FTMn ftmn, CHn ch)
{
    FTM_PWM_Duty(ftmn,ch,0);//关闭PWM
}

