/*
* �жϴ������
*/
#include "common.h"
#include "include.h"
#include "define.h"
#include "ISR_FUN.h"
extern signed int Speed_goal1_TEMP;//���ת��Ŀ��ֵ
extern signed int Speed_goal2_TEMP;//���ת��Ŀ��ֵ
u8 TimeCnt_Start_Reduct_Flag = 1 ;
/*************************************************************************
*                             Ұ��Ƕ��ʽ����������
*
*  �������ƣ�HardFault_Handler
*  ����˵����Ӳ���Ϸ��жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-2-4    �Ѳ���
*  ��    ע��������LED��˸��ָʾ������Ӳ���Ϸ�
*************************************************************************/

/*************************************************************************
*  �������ƣ�VSYNC_IRQ
*  ����˵����PORTD�˿��жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-1-25    �Ѳ���
*  ��    ע�����ź���Ҫ�Լ���ʼ�������
*************************************************************************/
void VSYNC_IRQ(void)
{
    static u32 flag;
    //Clear Interrupt flag
    flag = PORTD_ISFR;
    PORTD_ISFR = flag;
//    Image_Flag=0;//����������������0����
    if(img_flag == IMG_START)	//��Ҫ��ʼ�ɼ�ͼ��
    {
      //����洢��ѭ��ʹ��
      //���֮ǰͼ����ʹ�õĴ洢����2����DMA�����λ����1�����ʱ�Ѵ���λ�ø�Ϊ2
      if(Memory_use_Flag==2)
      {
        Memory_use_Flag=1;
        DMA_PORTx2BUFF_Init(CAMERA_DMA_CH, (void *)&PTB_BYTE2_IN, (void *)Image_fire_Memory2, PTA26, DMA_BYTE1, CAMERA_SIZE , DMA_falling);
        DMA_DADDR(CAMERA_DMA_CH) = (u32)Image_fire_Memory2; //�ָ���ַ
      }
      //���֮ǰͼ����ʹ�õĴ洢����1����DMA�����λ����2�����ʱ�Ѵ���λ�ø�Ϊ1
      else if(Memory_use_Flag==1)
      {
        Memory_use_Flag=2;
        DMA_PORTx2BUFF_Init(CAMERA_DMA_CH, (void *)&PTB_BYTE2_IN, (void *)Image_fire_Memory1, PTA26, DMA_BYTE1, CAMERA_SIZE , DMA_falling);
        DMA_DADDR(CAMERA_DMA_CH) = (u32)Image_fire_Memory1; //�ָ���ַ
      }
        DMA_EN(CAMERA_DMA_CH);            		//ʹ��ͨ��CHn Ӳ������
        disable_irq(90);  
        img_flag=IMG_GATHER;
    }

    else					//ͼ��ɼ�����
    {
        img_flag = IMG_START;	//��ʼ�ɼ�ͼ��
        PORTD_ISFR=~0;		//д1���жϱ�־λ(����ģ���Ȼ�ص���һ���жϾ����ϴ����ж�)
        enable_irq(90); 
    }

}

/*************************************************************************
*  �������ƣ�DMA0_IRQHandler
*  ����˵����DMA0
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-1-25    �Ѳ���
*  ��    ע�����ź���Ҫ�����Լ���ʼ�����޸�
*************************************************************************/
void DMA0_IRQHandler()
{
    DMA_DIS(CAMERA_DMA_CH);            	//�ر�ͨ��CHn Ӳ������
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);           //���ͨ�������жϱ�־λ
    img_flag = IMG_FINISH ; 
    Image_Flag=1;
}

void PendSV_Handler(void)
{
  
}
void HardFault_Handler(void)
{

}
void SysTick_Handler(void)
{
  
}
void USART1_IRQHandler()
{
  static u8 b_cnt=0;
  const u8 i=0;
  
  u8 j;
  int Sum=0;
  char c;
  char Res_Temp[10]={0};
  static char Res[5];
  DisableInterrupts;
  uart_pendstr(UART5,Res_Temp);
//  uart_sendStr(UART5,Res_Temp);
  c=Res_Temp[0];
  switch(c)
  {
//  case 'n':
//    {
//      for(j=0;j<i;j++)
//      {
//        Res[j]=0;
//      }
//      i=0;              //�������
//      break;
//    }
  case 'm':
    {
      LED2=!LED2;
      break;
    }
  case 's':
    {
      break;
    }
  case 'p':
    {
      break;
    }
  case 'i':
    {

      break;
    }
  case 'd':
    {

      break;
    }
  case 'b':
    {
      if(b_cnt==0)
      {
        Blue_Start_Flag=1;
//        Speed_Start_Flag=1;
        b_cnt++;
      }
      else
      {
        Speed_goal1=0;
        Speed_goal2=0;
        Speed_stand=0;
        stand_p=0;
        stand_d=0;
        Motor_enable_Flag=0;
      }
      break;
    }
  case 'a'://ͣ��
    {
      Speed_goal1_TEMP=Speed_goal1;
      Speed_goal2_TEMP=Speed_goal2;
      Speed_goal1=0;
      Speed_goal2=0;
      Speed_stand=0;
      stand_p=0;
      stand_d=0;
      Motor_enable_Flag=0;
      
//      disable()
      break;
//      FTM_PWM_Duty(MOTOR_1,0);
//      FTM_PWM_Duty(MOTOR_2,0);
    }
  case '1':
    {
      Speed_goal1+=100;
      break;
    }
  case '2':
    {
      Speed_goal1-=100;
      break;
    }
  case '3':
    {
      Speed_goal2+=100;
      break;
    }
  case '4':
    {
      Speed_goal2-=100;
      break;
    }
    
  default:
  {
    if(c>=48&&c<=57)//��������֣�����Ч
    {
//      i=strlen(Res_Temp);
      for(j=0;j<i;j++)
      {
        Res[j]=Res_Temp[j]-48;
      }
//      printf("\r\n%d\r\n",i);
    }
    
    break;
  }
  }
  
  for(j=0;j<i;j++)
  {
    uart_putchar(UART5,Res[j]+48);
  }
  printf("\r\n");
  LED1=!LED1;
  EnableInterrupts;
}

void USART2_IRQHandler()
{

  
}
  

void USART3_IRQHandler()
{

}
void USART5_IRQHandler()
{

}


void PIT0_IRQHandler()
{
  static unsigned char TimeCnt_20ms = 0,TimeCnt_5ms=0;	  //5ms,20msʱ�������    
  static unsigned int  TimeCnt_2000ms = 0;
  static unsigned int  TimeCnt_Key_Start_ms = 0;
  static unsigned int  TimeCnt_Key__ms = 0;
  static unsigned int  TimeCnt_Start_Reduct_ms = 0;
  static unsigned int  _1000ms_cnt = 0;
  u8 i;
  //ʱ���߸��� 
  Time_1ms++; 
  TimeCnt_5ms++;
  TimeCnt_20ms++;
  _1000ms_cnt ++;
  if(Reduct_Flag==1&&Blue_Start_Flag==1)
    TimeCnt_2000ms++;
  if(Key_Start_Flag==1)
  {
    TimeCnt_Key_Start_ms++;
    if(TimeCnt_Key_Start_ms==2500)
    {
      Key_Start_Flag = 0;
      Blue_Start_Flag = 1;
    }
  }

  if(Blue_Start_Flag==1&&TimeCnt_Start_Reduct_ms<1002)
  {
    TimeCnt_Start_Reduct_ms++;
    if(TimeCnt_Start_Reduct_ms==1000)
    TimeCnt_Start_Reduct_Flag=0;
  }
  if(TimeCnt_5ms >= 5)
    TimeCnt_5ms = 0;
  if(TimeCnt_20ms >= 20)
    TimeCnt_20ms = 0;
  if(_1000ms_cnt >=1000)
  {
    _1000ms_cnt = 0;
    b = a;
    a = 0;
    if(b<50)
    {
      Blue_Start_Flag = 0;
      if(LCD_DISPLAY_FLAG==0)
      {
//        Beep_Once(&Diaozhen_2000ms);      
      }
    }
  }
  
//**************************��������ʱ���־******************************
  if(Run_.Run_flag==0)
  {
    Run_.Run_flag = myPower_Test();
    if(Run_.Run_flag)
    {
      Beep_Once(&Test_100ms);
      Blue_Start_Flag = 1;
    }
  }
  
  
  
//*******************************End*********************************
  
//*************************����ʱ���־***********************
  if(Island.Next_Island_flag==1)
  {
    if(Island.Next_Island_flag_delay==0)
    {
      Island.State = NoIsland;//���־
      Island.In_Center = 0;
      Island.Half_longer_cnt = 0;
      Island.Stay_Center = 0;
      Island.Out_Center_ = 0;
      Island.Out_Center  = 0;
      Island.Stay_Island_center_cnt = 0;
      Island.Out_Allow_flag = 0;
      Island.doubleAD_Allow_flag = 0;
      Island.Stay2Out_cnt = 0;
      Island.Next_Island_flag = 0;
      Island._2Next_Island_cnt = 0;
      Island.In2Stay_cnt = 0;
//      Beep_Once(&Image_Island_Test_Beep);
    }
    Island.Next_Island_flag_delay--;
  }
  if(Island.Stay2Out_flag==1)
  {
    if(Island.Stay2Out_flag_delay==0)
    {
      Island.Out_Allow_flag = 1;
      Island.Stay2Out_flag = 0;
    }
    Island.Stay2Out_flag_delay--;
  }
  if(Island.Out2doubleAD_flag==1)
  {
    if(Island.Out2doubleAD_flag_delay==0)
    {
      Island.doubleAD_Allow_flag = 1;
      Island.Out2doubleAD_flag = 0;
    }
    Island.Out2doubleAD_flag_delay--;
  }
  
//*****************************END****************************
  
//**********************************������ʱ���־*****************************
  if(Blue_Start_Flag==1
     &&Start_line.test_allow_flag==0)
  {
    if(Start_line._2Over_cnt==0)
    {
      Start_line.test_allow_flag = 1;
    }
    Start_line._2Over_cnt--;
  }
  
  if(Start_line.Elec_delay_flag==1)
  {
    if(Start_line.Elec_delay==0)
    {
      Start_line.Elec_delay_flag = 0;
    }
    Start_line.Elec_delay--;
  }

//***********************************END**********************************  
  
//***********************Beepʱ���־*******************
  Beep_ISR(&Image_Island_Test_Beep);
  Beep_ISR(&Test_100ms);
  Beep_ISR(&Diaozhen_2000ms);
  Beep_ISR(&Test_10000ms);
//***************************END******************
  
//��������
//    if(button_timeout<16)
//    {
//      button_timeout--;
//    }
//  if(button_timeout==0)//��ʱ10ms��ⰴ��״̬
//  {
//      if(KEY1==0)
//      {
//        Reduct_Flag=1;
//        Key_status=KEY1_PRES;
////        BEEP=1;
//        LED1=0;
//      }
//      else if(KEY2==0)
//      {
//        Key_Start_Flag=1;
//        Key_status=KEY2_PRES;
//        LED2=0;
//      }
//  }

//����������������
//*********************************************ʮ����ʱ******************************************
  if(Cross.Cross_delay_flag==1)
  {
    if(Cross.Cross_delay_cnt==0)
    {
      Cross.Cross_delay_flag = 0;//���
    }
    Cross.Cross_delay_cnt--;
  }
//*********************************************END********************************************
  
//*********************************************�µ���ʱ***************************************
  if(Podao.Stay_flag==1)
  {
    if(Podao.Stay_delay==0)
    {
//      Beep_Once(&Test_100ms);
      Podao.Stay_flag = 0;
    }
    Podao.Stay_delay--;
  }
  if(Podao._2Next_flag==1)
  {
    if(Podao._2Next_delay==0)
    {
      Podao._2Next_flag = 0;
    }
    Podao._2Next_delay--;
  }
  if(Podao.Verify_flag==1)
  {
    if(Podao.Verify_delay==0)
    {
      Podao.Verify_flag = 0;
    }
    Podao.Verify_delay--;
  }

//*********************************************END*****************************************  
  
//***************************************************���ٺ���
  if(TimeCnt_5ms==1)
  {
  }
  
//****************************************************���ٺ���
  if(TimeCnt_5ms==2&&Blue_Start_Flag==1)//20msִ��һ��
  {
    Get_speed1(&Motor2);
    Speed_output();
    if(L_AD_Sum<300&&R_AD_Sum<300)
    {
      Blue_Start_Flag = 0;
//      Beep_Once(&Image_Island_Test_Beep);
    }
  }
  if(TimeCnt_5ms==3)
  {
    AD_new();
    if(ad_once(MYADC_3,ADC_12bit)*3.3/4096*3.75>10)
    {
      Beep_Once(&Diaozhen_2000ms);
    }
  }
  
  if(TimeCnt_20ms==2&&Blue_Start_Flag==1)
  {
    Servo_Diff_PID();
  }
    //*****************************************************adc���
  if(Time_1ms == 1000)
  {
    stand_AD_L = L_AD_Ave;
    stand_AD_R = R_AD_Ave;
    if(stand_AD_L>10000||stand_AD_R>10000)
    {
      
    }
    stand_AD = (stand_AD_L+stand_AD_R)/2;
  }
//���ٺ�������
  PIT_Flag_Clear(PIT0);
}
void PIT3_IRQHandler()
{
//  Delay_Mark=1;                 //������ʱѭ����־λ��1
//  disable_irq(71);             //�жϴ�һ�κ󱻹ر�
  PIT_Flag_Clear(PIT3);
}

void PIT2_IRQHandler()
{
//  TEMP_OF_MARK=1;
  PIT_Flag_Clear(PIT2);
}
void PORTC_IRQHandler()
{
  u8 n=0;
  n=18;
  if(PORTC_ISFR & (1<<n))
  {
//�����ж�1���жϷ���
//    Beep_Once(&Image_Island_Test_Beep);
    if(Start_line.Elec_delay_flag==1)
    {
      Start_line.test_allow_flag = 0;//��ձ�־
      Start_line._2Over_cnt = Start_line._2Over_cnt_const;//���¸�ֵ
      Start_line.Start_Line_cnt++;
    }
    if(Start_line.Start_Line_cnt==1)
    {
      Blue_Start_Flag = 0;//�رյ��
    }

    PORTC_ISFR |=(1<<n);//д��1����жϱ�־λ
  }
}
void PORTA_IRQHandler()
{
  u8 n=0;
  n=17;
  if(PORTA_ISFR & (1<<n))
  {
//�����ж�1���жϷ���
    button_timeout=15;//��־��15
    PORTA_ISFR |=(1<<n);//д��1����жϱ�־λ
  }
  n=15;
  if(PORTA_ISFR & (1<<n))
  {
//�����ж�1���жϷ���
    button_timeout=15;//��־��15
    PORTA_ISFR |=(1<<n);//д��1����жϱ�־λ
  }
}

void PORTB_IRQHandler()
{
  u8 n=0;
  n=1;
  if(PORTB_ISFR & (1<<n))
  {
//�����ж�1���жϷ���
    button_timeout=15;//��־��15
    PORTB_ISFR |=(1<<n);//д��1����жϱ�־λ
  }
}

void PORTE_IRQHandler()
{

}
void FTM0_IRQHandler()
{
  
}
void FTM1_IRQHandler()
{
  
}

void FTM2_IRQHandler()
{
  
}
