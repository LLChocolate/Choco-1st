#include "include.h"
#include "stdlib.h"
//u8 B_f=0;

u32 L_AD_Ave = 0;
u32 R_AD_Ave = 0;
u32 L_AD_Sum = 0;
u32 R_AD_Sum = 0;
Motor_Status Motor1;//电机状态结构体
Motor_Status Motor2;//电机状态结构体
int Speed_L_sum=0;
int Speed_R_sum=0;
s32 Duty_Motor1;//电机占空比
s32 Duty_Motor2;//电机占空比
PID_Struct Motor1_PID;//pid结构体
PID_Struct Motor2_PID;//pid结构体
PID_Struct Diff_PID;//差速PID
PID_Struct Diff_Straight;//直线PID
float Speed_goal1=25;//电机转速目标值
float Speed_goal2=25;//电机转速目标值
u16 Diff_goal=0;
int Diff_error=0;
u16 Master_Speed=25;
u16 ADC_Value;//ADC值
u16 Speed_stand;
int Weight_mean=0;
//***************************定时器标志位*************************
u8 button_timeout=255;//按键时间标志
long int Time_1ms=0;//时间轴
//****************************************************************
u8 Road_Status_Flag=0;
u16 Switch_Status;//拨码状态
u8 Key_status;//按键状态
u8 LCD_DISPLAY_FLAG=0;
u8 Motor_enable_Flag=1;
u8 Slow_Flag=0;
u8 Reduct_Flag=0;
u8 Blue_Start_Flag=0;//蓝牙开启
u8 Key_Start_Flag=0;
//***********************调试用临时全局变量**************************
float stand_p;
float stand_d;
int P_TEMP1=550;//201
int I_TEMP1=70;
int D_TEMP1=0;
int P_TEMP2=200;//90
int I_TEMP2=50;
int D_TEMP2=0;
signed int Speed_goal1_TEMP=80;//电机转速目标值
signed int Speed_goal2_TEMP=80;//电机转速目标值
u8 Image_Flag=1;
u8 LED_timeout=50;
u8 Dir_temp=1;
u16 hang=0;
int DIFF_UP=110;
int DIFF_DOWN=-110;
long int  temp_cnt=0;
u8 DIFF_PID_CHANGE_FLAG=0;
u16 temp_CNT_WATCH=0;
u8 Speed_max_to_min_diff;
u8 Acc_Limit=40;
float Acc_K=1;
u16 stand_AD_L = 0xffff;
u16 stand_AD_R = 0xffff;
u16 stand_AD   = 0Xffff;
PID_Struct Servo_PID;
int center_temp_test = 160;
u16 duty_temp=32767;
Cap_Run_Str Run_={.Run_Volts = 8.6};
Start_line_Str Start_line = {._2Over_cnt = 10000,
                            ._2Over_cnt_const = 10000,
                            .Elec_delay_const = 1000,
                            .Stop_delay_const = 1000};
u16 Center_for_debug;
int a,b;
float Real_AD;
int Ave = 0;
u8 Liner_threshold = 4;
u16 SERVO_MIDDLE = 4750;
u8 LED_flag = 0;
u8 Beep_flag = 0;
//******************************************************************
void main()
{
  u8 Key;
  u16 i=0,j;
  System_Init();
  Brush_Color=Black;
  Motor1_PID.target=Speed_goal1;
  Motor2_PID.target=Speed_goal2;
  Duty_Motor1=10000;
  Duty_Motor2=62000;
  
while(1)
{
  Motor2_PID.P=P_TEMP2;
  Motor2_PID.I=I_TEMP2;
  Key = KEY_Scan();
  if(Key == KEY1_PRES)
  {
    LCD_init(FALSE);
    Disp_single_colour(White);
    LCD_DISPLAY_FLAG = 1;
//    Beep_Once(&Image_Island_Test_Beep);
  }
  else if(Key == KEY2_PRES)
  {
    LCD_DISPLAY_FLAG = 0;
    Key_Start_Flag = 1;
    Beep_Once(&Image_Island_Test_Beep);
//    Beep_Once(&Test_10000ms);
  }
//摄像头采集一次
//图像处理  
    if(Image_Flag==1)
  {
    Image_Flag=0;
    ov7725_get_img();//转存结束后立刻允许接收场中断 
    image_process();
    if(LCD_DISPLAY_FLAG==1)
    {
      Send_Image_to_LCD(Image_fire);
      Send_Image_to_LCD(Image_fire);
      LCD_Draw_Line(Image_lie.Three_Lie[0],Image_lie.Three_lie_end[0],Image_lie.Three_Lie[0],160);  
      LCD_Draw_Line(Image_lie.Three_Lie[1],Image_lie.Three_lie_end[1],Image_lie.Three_Lie[1],160);  
      LCD_Draw_Line(Image_lie.Three_Lie[2],Image_lie.Three_lie_end[2],Image_lie.Three_Lie[2],160);
      LCD_Draw_Line(0,Island.Image_Start_hang,319,Island.Image_Start_hang);
//        LCD_Draw_Line(0,Image_hang.hang_use,319,Image_hang.hang_use);
      LCD_Draw_Line(0,Start_Point,319,Start_Point);
      LCD_Draw_Line(0,Island.Correct_hang,319,Island.Correct_hang);
      LCD_Put_Int(100,100,"",L_AD_Ave,Red,White);
      LCD_Put_Int(100,120,"",R_AD_Ave,Red,White);
    }
    if(Memory_use_Flag==1)//改变缓冲区
    {
      Image_fire=&Image_fire_Memory2[0];
    }
    else if(Memory_use_Flag==2)
    {
      Image_fire=&Image_fire_Memory1[0];
    }
    a++;
  }
  SCI_Send_Datas(UART1);
  if(Blue_Start_Flag==0)
  {
    FTM_PWM_Duty(STEER_,SERVO_MIDDLE);
    FTM_PWM_Duty(MOTOR_2,0xfff0);
  }
  
}
while(1);
}