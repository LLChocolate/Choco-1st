/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名       ：include.h
 * 描述         ：工程模版头文件
 *
 * 实验平台     ：野火kinetis开发板
 * 库版本       ：
 * 嵌入系统     ：
 *
 * 作者         ：野火嵌入式开发工作室
 * 淘宝店       ：http://firestm32.taobao.com
 * 技术支持论坛 ：http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/

#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"
#include  "define.h"
/*
 * Include 用户自定义的头文件
 */
#include "MK60DZ10.h"
#include  "gpio.h"      //IO口操作
#include  "port.h" 
#include  "uart.h"      //串口
#include  "adc.h"       //ADC模块
#include  "FTM.h"       //FTM模块（FTM0：电机控制 / 通用 /PWM     FTM1、2：正交解码 / 通用 /PWM ）
#include  "PIT.h"       //周期中断计时器
#include  "lptmr.h"
#include  "exti.h"      //EXTI外部GPIO中断
#include  "common.h"
#include  "arm_math.h"  //DSP库
#include  "MK60_spi.h"
#include  "Car_init.h"
#include  "define.h"
#include  "i2c.h"
#include  "dma.h"
#include  "PIT.h"   
#include  "fire_drivers_cfg.h"   //管脚复用配置
#include  "isr.h"
#include  "math.h"
#include  "mymath.h"
#include  "delay.h"
#include  "led.h"
#include  "mykey.h"
#include  "beep.h"
#include  "dial_switch.h"
#include  "myPWM.h"
#include  "getspeed.h"
#include  "lcd.h"
#include  "pid.h"
#include  "OV7725.h"
#include  "OV7725_REG.h"
#include  "SCCB.h"
#include  "image_process.h"
//#include "mpu6050.h"
//#include "inv_mpu.h"
//#include "inv_mpu_dmp_motion_driver.h" 
//#include "pacman.h"
#include "filter.h"
#include "cmt.h"
#include "mypower.h"

/*************************************************************************
*  模块名称：结构体和变量模块
*  功能说明：Include 用户自定义的结构体和变量
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14
*************************************************************************/
//extern u8 B_f;
extern u16 Brush_Color; 
extern u16 Back_Color;

extern Motor_Status Motor1;//电机状态结构体
extern Motor_Status Motor2;//电机状态结构体
extern s32 Duty_Motor1;//电机占空比
extern s32 Duty_Motor2;//电机占空比
extern PID_Struct Motor1_PID;//pid结构体
extern PID_Struct Motor2_PID;//pid结构体
extern PID_Struct Diff_PID;//差速PID
extern float Speed_goal1;//电机转速目标值
extern float Speed_goal2;//电机转速目标值
extern u16 Diff_goal;
extern int Diff_error;
extern u16 Master_Speed;
extern u16 ADC_Value;//ADC值
extern u16 Speed_stand;
extern int Weight_mean;
extern float stand_p;
extern float stand_d;
//***************************定时器标志位*************************
extern u8 button_timeout;//按键时间标志
extern long int Time_1ms;//时间轴
//****************************************************************

extern u8 Road_Status_Flag;//道路形状判断标志
extern u16 Switch_Status;//拨码状态
extern u8 Key_status;//按键状态
extern u8 LCD_DISPLAY_FLAG;
extern u8 Image_Flag;
extern u8 Black_Lock;
extern int DIFF_UP;
extern int DIFF_DOWN;
//***********************************摄像头**********************************
extern u8 Memory_use_Flag;
extern u8 Image_fire_Memory1[CAMERA_H+1][CAMERA_DMA_NUM];
extern u8 Image_fire_Memory2[CAMERA_H+1][CAMERA_DMA_NUM];
extern u8 (*Image_fire)[CAMERA_DMA_NUM];//指针
//extern u8 Image_fire_extract[CAMERA_H+1][CAMERA_W];
extern volatile u8 img_flag;		//图像状态
//extern int black_L[ALL_LINE];
//extern int black_R[ALL_LINE];
//extern u16 Right_count[ALL_LINE];
//extern u16 Left_count[ALL_LINE];
extern u8 getLeft_flag[ALL_LINE];
extern u8 getRight_flag[ALL_LINE];
extern u8 LED_timeout;
extern u16 hang;
extern u8 DIFF_PID_CHANGE_FLAG;
extern long int  temp_cnt;
extern u16 temp_CNT_WATCH;
extern u8 Three_lie_end[3];
extern u8 Speed_max_to_min_diff;
extern u8 Motor_enable_Flag;
extern u8 Slow_Flag;
extern u8 Reduct_Flag;
extern u8 Blue_Start_Flag;//蓝牙开启
extern u8 Key_Start_Flag;
extern u8 Acc_Limit;
extern float Acc_K;
extern int Speed_L_sum;
extern int Speed_R_sum;
extern u32 L_AD_Ave;
extern u32 R_AD_Ave;
extern Image_hangData Image_hang;
extern Image_lieData  Image_lie;
extern Island_Data    Island;
extern Beep_Str       Image_Island_Test_Beep;//测试摄像头环岛监测
extern int _1000ms_cnt;
extern u16 stand_AD_L;
extern u16 stand_AD_R;
extern u16 stand_AD;
extern u8 TimeCnt_Start_Reduct_Flag;
extern Beep_Str Image_Island_Test_Beep;//测试摄像头环岛监测
extern Beep_Str Test_100ms;
extern Beep_Str Diaozhen_2000ms;
extern Beep_Str Test_10000ms;
extern PID_Struct Servo_PID;
extern Cap_Run_Str Run_;
extern Start_line_Str Start_line;
extern Cross_Data Cross;
extern Podao_Str Podao;
extern u32 L_AD_Sum;
extern u32 R_AD_Sum;
extern u16 Center_for_debug;
extern int a,b;
extern u8 Podao_flag;
extern u16 Podao_delay;
extern u16 Podao_delay_const;
extern float Real_AD;
extern u8 Start_Point;
extern int Ave;
extern Register_Info ov7727_reg[];
extern u8 Liner_threshold;
extern u16 SERVO_MIDDLE;
extern int now_err;

#endif  //__INCLUDE_H__
