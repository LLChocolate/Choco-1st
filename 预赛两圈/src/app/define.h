#ifndef _DEFINE_H
#define _DEFINE_H

#include  "common.h"
/*************************************************************************
*  ģ�����ƣ�defineģ��
*  ����˵����Include �û��Զ���ĺ�
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-2-14
*************************************************************************/

//#define LCD_DISPLAY

//��·״̬��־
typedef enum
{
  Nstart,//δ��ʼ״̬
  Straight,//��ֱ��
  Left_turn,//��ת
  Right_turn,//��ת
//  Cross,//ʮ��
  Uphill,//�µ�
  
}Road_Status;


#define Speed_filter_Period       5//��ֵ�˲�����

//������
#define MOTOR_1                         FTM0,CH4//����ӿ�
#define MOTOR_2                         FTM1,CH0//����ӿ�
#define STEER_                          FTM0,CH6//����ӿ�
#define MOTOR1_DIR                       PTA13_OUT//����������
#define MOTOR2_DIR                       PTE26_OUT//����������
typedef struct{
  u8 Dir;
  s16 Speed;
}Motor_Status;          //���״̬�ṹ��

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
//����
#define SW1              PTB0_IN
#define SW2              PTB1_IN
#define SW3              PTB2_IN
#define SW4              PTB3_IN
#define SW5              PTB4_IN
#define SW6              PTB5_IN
#define SW7              PTB6_IN
#define SW8              PTB7_IN

//����
#define KEY1             PTC8_IN
#define KEY1_PRES        1
#define KEY2             PTC6_IN
#define KEY2_PRES        2

//������
#define BEEP PTE24_OUT

//��ˮ��
#define LED1             PTD0_OUT
#define LED2             PTD2_OUT


//PID�ṹ��
typedef struct                    //�ṹ�壬���PID��ر���
{
  float P;                        //����P
  float I;                        //����I
  float D;                        //����D
  float error[3];                 //���洢����
  float delat;                    //ÿ�εĵ��ڽ��
  float derr;                     //һ�����
  float dderr;                    //�������
  float result;                      //PID�����������������ʽ�����Գ�ֵ����Ϊ����ƽ��ʱ�����ֵ����Ҫ��������0Ҫ����ը��
  
  float target;                   //PID���ڵ�Ŀ��ֵ     
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

////ϵͳ״̬�ṹ��
//typedef struct
//{
//  long int Time_1ms;
//  
//}System_Status

#define     pit_delay_ms(PITn,ms)          pit_delay(PITn,ms * bus_clk_khz);        //PIT��ʱ ms
#define     pit_delay_us(PITn,us)          pit_delay(PITn,us * bus_clk_khz/1000);   //PIT��ʱ us

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
  u8  Correct_hang;     //�������ߵ���
  int In_Center;
  
  int Stay_Center;
  
  int Out_Center;//�����������ı�׼
  
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
  
  u8  Next_Island_flag;         //����һ������֮���ʱ����
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
  NoCross,//û��ʮ��
  R2Cross_Pre, //����������б��ʮ��
  L2Cross_Pre, //����������б��ʮ��
  R2Cross_True,//���������б��ʮ��
  L2Cross_True,//���������б��ʮ��
  Str2Cross//ֱ��ʮ��

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
  u8 Run_flag;//���ܱ�־
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