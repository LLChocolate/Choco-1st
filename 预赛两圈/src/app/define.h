#ifndef _DEFINE_H
#define _DEFINE_H

#include  "common.h"
/*************************************************************************
*  模块名称：define模块
*  功能说明：Include 用户自定义的宏
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14
*************************************************************************/

//#define LCD_DISPLAY

//道路状态标志
typedef enum
{
  Nstart,//未开始状态
  Straight,//走直线
  Left_turn,//左转
  Right_turn,//右转
//  Cross,//十字
  Uphill,//坡道
  
}Road_Status;


#define Speed_filter_Period       5//均值滤波周期

//电机舵机
#define MOTOR_1                         FTM0,CH4//电机接口
#define MOTOR_2                         FTM1,CH0//电机接口
#define STEER_                          FTM0,CH6//舵机接口
#define MOTOR1_DIR                       PTA13_OUT//电机方向控制
#define MOTOR2_DIR                       PTE26_OUT//电机方向控制
typedef struct{
  u8 Dir;
  s16 Speed;
}Motor_Status;          //电机状态结构体

//#define 

typedef struct
{
  u16 Middle;
  u16 Now_duty;
}
Servo_Str;


//ADC

#define MYADC_1                          ADC1,AD4a
#define MYADC_2                          ADC1,AD5a
#define MYADC_3                          ADC1,AD6a
#define MYADC_4                          ADC1,AD7a
//拨码
#define SW1              PTB0_IN
#define SW2              PTB1_IN
#define SW3              PTB2_IN
#define SW4              PTB3_IN
#define SW5              PTB4_IN
#define SW6              PTB5_IN
#define SW7              PTB6_IN
#define SW8              PTB7_IN

//按键
#define KEY1             PTC8_IN
#define KEY1_PRES        1
#define KEY2             PTC6_IN
#define KEY2_PRES        2

//蜂鸣器
#define BEEP PTE24_OUT

//流水灯
#define LED1             PTD0_OUT
#define LED2             PTD2_OUT


//PID结构体
typedef struct                    //结构体，存放PID相关变量
{
  float P;                        //参数P
  float I;                        //参数I
  float D;                        //参数D
  float error[3];                 //误差存储数组
  float delat;                    //每次的调节结果
  float derr;                     //一阶误差
  float dderr;                    //二阶误差
  float result;                      //PID的输出，由于是增量式的所以初值请设为导轨平衡时的输出值，重要！不能是0要不就炸了
  
  float target;                   //PID调节的目标值     
  float feedback;
  float UP_Limit;
  float LOW_Limit;
}PID_Struct;

typedef struct 
{
  float x_mid;
  float x_now;
  float p_mid ;
  float p_now;
  float kg;
  float ProcessNoise_Q;
  float MeasureNoise_R;
  float x_last1;
  float p_last1;
}Kalman_Date;

typedef struct
{
  float m_filter;
  float ResrcData_mem[2];
  float output_mem[2];
}Filter_1st_Str;

////系统状态结构体
//typedef struct
//{
//  long int Time_1ms;
//  
//}System_Status

#define     pit_delay_ms(PITn,ms)          pit_delay(PITn,ms * bus_clk_khz);        //PIT延时 ms
#define     pit_delay_us(PITn,us)          pit_delay(PITn,us * bus_clk_khz/1000);   //PIT延时 us

typedef enum{
  Unlock,
  Left_Lock,
  Right_Lock
}Black_View_Run_Status;


#define ALL_LINE            (240)
typedef struct
{
  int center[ALL_LINE];
  u8 Center_Black_flag[ALL_LINE];
  int halfwidth[ALL_LINE];
  int halfwidth_const[ALL_LINE];
  int black_L[ALL_LINE];
  int black_R[ALL_LINE];
  u8 getLeft_flag[ALL_LINE];
  u8 getRight_flag[ALL_LINE];
  u8 hang_use;
}Image_hangData;


typedef struct
{
  u8 Three_Lie[3];
  u8 Three_lie_end[3];
  int Black_L;
  int Black_R;
  u8 Far_30[30];
  int Far_end;
  int Far_center;
}Image_lieData;


enum ISLAND_STATE
{
  NoIsland,
  Left_Island_pre,
  Right_Island_pre,
  Left_Island_in,
  Right_Island_in,
  Left_Island_out,
  Right_Island_out,
  Left_Wait_Next,
  Right_Wait_Next
};

#define Island_Center_Period_Const  (25)
typedef struct
{
  enum ISLAND_STATE State;
  
  float L_P;
  float L_D;
  float R_P;
  float R_D;
  u16   Speed;
  
  u16 black_L[40];
  u16 black_R[40];
  
  u8 Image_Start_hang;
  int Image_Start_hang_half;
  u8 Half_longer_cnt;
  u8  Correct_hang;     //用来补线的行
  int In_Center;
  
  int Stay_Center;
  
  int Out_Center;//用作出环岛的标准
  
  u8  Stay_hang_use;
  u8  Stay2Out_flag;
  u16 Stay2Out_flag_delay;
  u16 Stay2Out_flag_delay_const;
  u8  Out_Allow_flag;
  
  u8  Out2doubleAD_flag;
  u16 Out2doubleAD_flag_delay;
  u16 Out2doubleAD_flag_delay_const;
  u8  doubleAD_Allow_flag;
  
  u8  In2Stay_cnt;
  u8  Stay2Out_cnt;
  
  u32 Out_Center_;
  u16 Stay_Island_center_cnt;
  
  u8  Next_Island_flag;         //和下一个环岛之间的时间间隔
  u16 Next_Island_flag_delay;
  u16 Next_Island_flag_delay_const;
  
  u8  _2Next_Island_cnt;
}Island_Data;

typedef struct
{
  u8 Flag;
  u16 Delay;
  u16 Delay_const;
}Beep_Str;

enum Cross_STATE
{
  NoCross,//没有十字
  R2Cross_Pre, //可能在向右斜入十字
  L2Cross_Pre, //可能在向左斜入十字
  R2Cross_True,//真的在向右斜入十字
  L2Cross_True,//真的在向左斜入十字
  Str2Cross//直入十字

};

typedef struct
{
  enum Cross_STATE State;
  int In_center;
  u8  Test_hang;
  u16  Cross_delay_cnt;
  u16  Cross_delay_cnt_const;
  u8   Cross_delay_flag;
}Cross_Data;


typedef struct
{
  float Volts_now;
  float Run_Volts;
  u8 Run_flag;//起跑标志
}Cap_Run_Str;


typedef struct
{
  u16 _2Over_cnt;
  u16 _2Over_cnt_const;
  u8 test_allow_flag;
  
  u8 Elec_delay_flag;
  u16 Elec_delay;
  u16 Elec_delay_const;
  
  u8 Start_Line_cnt;
  
  u8 Square_doublt_flag;
  u8 Square_flag;
  
  u8 Stop_delay_flag;
  u16 Stop_delay;
  u16 Stop_delay_const;
  u8 Stop_flag;
}Start_line_Str;

typedef struct
{
  u8 Stay_flag;
  u16 Stay_delay;
  u16 Stay_delay_const;
  
  u8 _2Next_flag;
  u16 _2Next_delay;
  u16 _2Next_delay_const;
  
  u8 Verify_flag;
  u16 Verify_delay;
  u16 Verify_delay_const;
  
  u16 Speed;
}Podao_Str;

#endif //_DEFINE_H