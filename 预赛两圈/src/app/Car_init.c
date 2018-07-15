#include  "include.h"


//void PID_Init(PID_Struct* pid,double P_,double I_,double D_,u32 Result,double Target)
void Parameters_init_CAR(void);
void Start_line_Init(void);
void System_Init()
{
  DisableInterrupts;
  Motot2_Init();
  myBEEP_Init();
  myPower_Init();
  pit_init_ms(PIT0,1);
  dial_switch_Init();
  MY_adc_init();
  getspeed1_init();
  myLED_Init();
  Servo_Init();
//  FTM_PWM_init(FTM0,CH2,10000,30000);
  myKEY_Init();
  Start_line_Init();
//  myKEY_Init();
  if(LCD_DISPLAY_FLAG==1)
  {
    LCD_init(FALSE);
    Disp_single_colour(White);
  }
  uart_init(UART1,115200);
//  UART_IRQ_EN(UART1);
//  MPU_Init();					//3?¡§o???£¤MPU6050
  Brush_Color=Red;			//¡§|¡§¡§???¨¢?¡§???ao¡§?¡§|? 
  Back_Color=White;
  Ov7725_Init();
  Parameters_init_CAR();
  PID_Init(&Motor2_PID,200,50,0,40000,50,65500,0);
//  PID_Init(&Diff_PID,0.85,0,0.25,0,0,DIFF_UP,DIFF_DOWN);//30:0.08,40:0.09,50:0.13  limit:20
  //                                                  60:0.32                          limit:35
  //                                                  70:0.4 limit:50
  //                                                  80:0.6  0.04  limit:80
  //                                                  90:1  0.07  limit:85
  //                                                    90:0.3
  //                                                    110:0.5 0.1  90
  //                                                    120:0.7 0.14  100
  //                                                    130:0.6 0.15    110
  //                                                    140:1.25 0.25 118
  //                                                    145:0.8 0.23
  //                                                    150:0.85 0.25
  //                                                    153:0.96 0.25
  //                                                    155:1   0.28  110
  stand_p=Servo_PID.P;
  stand_d=Servo_PID.D;
  EnableInterrupts;
  return ;
}
void Parameters_init_CAR(void)
{
  u8 i;
  Switch_Status=dial_switch_Scan();
  for(i=Far_Point;i<200;i++)
  {
    Image_hang.center[i] = 160;
    Image_hang.halfwidth[i] = 108;
    Image_hang.halfwidth_const[i] = (i - 85)*(108 - 73)/(105 - 85) + 73;
  }
  if((Switch_Status&3)==0)
  {
    Speed_stand=190;
    Start_Point = 94;
    PID_Init(&Servo_PID,3.4,0,1.5,0,0,492,-492);//Î´Ä¥ÂÖ×Ó
    Island.Image_Start_hang = 67;
    Island.Next_Island_flag_delay_const = 1450;
    Island.R_P = 3.5;
    Island.R_D = 0.6;
    Island.L_P = 3.5;
    Island.L_D = 0.6;
    Island.Speed = 190;
    Podao.Speed = 210;
  }
  else if((Switch_Status&3)==1)
  {
    Speed_stand=190;
    Start_Point = 94;
    PID_Init(&Servo_PID,4.5,0,8.5,0,0,492,-492);//Î´Ä¥ÂÖ×Ó
    Island.Image_Start_hang = 67;
    Island.Next_Island_flag_delay_const = 1450;
    Island.R_P = 3.7;
    Island.R_D = 5;
    Island.L_P = 5;
    Island.L_D = 10;
    Island.Speed = 190;
    Podao.Speed = 210;
  }
  else if((Switch_Status&3)==2)
  {
    Speed_stand=230;
    Start_Point = 94;
    PID_Init(&Servo_PID,5,0,10,0,0,492,-492);//Î´Ä¥ÂÖ×Ó
    Island.Image_Start_hang = 67;
    Island.Next_Island_flag_delay_const = 1600;
    Island.R_P = 3.7;
    Island.R_D = 5;
    Island.L_P = 5;
    Island.L_D = 10;
    Island.Speed = 190;
    Podao.Speed = 230;
  }
  else if((Switch_Status&3)==3)
  {
    Speed_stand=251;
    Start_Point = 93;
    PID_Init(&Servo_PID,5,0,10,0,0,492,-492);//Î´Ä¥ÂÖ×Ó
    Island.Image_Start_hang = 67;
    Island.Next_Island_flag_delay_const = 1600;
    Island.R_P = 3.7;
    Island.R_D = 5;
    Island.L_P = 5;
    Island.L_D = 10;
    Island.Speed = 190;
    Podao.Speed = 230;
  }

  if(((Switch_Status>>2)&3)==0)
  {
    ov7727_reg[45].Value = 75;
  }
  else if(((Switch_Status>>2)&7)==1)
  {
    ov7727_reg[45].Value = 80;
  }
  else if(((Switch_Status>>2)&7)==2)
  {
    ov7727_reg[45].Value = 85;
  }
  else if(((Switch_Status>>2)&7)==3)
  {
    ov7727_reg[45].Value = 90;
  }
  
  
  if(((Switch_Status>>4)&3)==0)
  {
    Run_.Run_Volts = 8.2;
  }
  else if(((Switch_Status>>4)&3)==1)
  {
    Run_.Run_Volts = 8.6;
  }
  else if(((Switch_Status>>4)&3)==2)
  {
    Run_.Run_Volts = 8.9;
  }
  else if(((Switch_Status>>4)&3)==2)
  {
    Run_.Run_Volts = 9.4;
  }
  
//  else if(Switch_Status==2)
//  {
//
//  }
}

void Start_line_Init(void)
{
  exti_init(PORTC,18,falling_up);
}
