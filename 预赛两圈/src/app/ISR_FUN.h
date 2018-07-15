#ifndef _ISR_FUN_H
#define _ISR_FUN_H

#define speed_Period_Constant 10 //速度控制周期
#define Diff_error_Period_Constant  10
void Speed_Control(void);
void Speed_output(void);
void AD_new(void);
void Servo_Diff_PID(void);
void get_center();

#endif