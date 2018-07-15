#include "ISR_FUN.h"
#include "include.h"
int Diff_PID_ave = 0;
Filter_1st_Str AD_L = {0.8,{0,0},{0,0}};
Filter_1st_Str AD_R = {0.8,{0,0},{0,0}};
int now_err;
void Speed_Control(void)
{
  static unsigned char speed_Period=0;//�ٶȿ������ڱ���
  static int R_ALL[Speed_filter_Period]={0};//100ms�ڵ��ٶ�ֵ
//  static float speed_Delta=0;
//  static float Tmp_Speed_P =0; 
//  float SpeedRate = 0;
  //float SpeedRate;  
  //�����ٶȣ�ǰ20ms���ۼ�ֵ��
  if(speed_Period >= (Speed_filter_Period))   
  speed_Period = 0;    
  Speed_R_sum-=R_ALL[speed_Period];
  R_ALL[speed_Period]=Motor2.Speed;
  Speed_R_sum+=R_ALL[speed_Period];
  speed_Period++;
}

void Speed_output(void)
{
  //���ٶȲ��ٻ�

  //�ٶȷ�������ֵ�˲�
//  if(TimeCnt_Start_Reduct_Flag==1)
//  {
    Motor2_PID.feedback = Motor2.Speed;
//  }
//  else 
//  {
//    Motor2_PID.feedback = Speed_R_sum;//Motor2.Speed;
//  }
  if(In_double_AD()==1
     ||Island.State==Left_Island_pre
     ||Island.State==Right_Island_pre
     ||Island.State==Left_Island_out
     ||Island.State==Right_Island_out
     ||Island.State==Left_Wait_Next
     ||Island.State==Right_Wait_Next
     ||Island.State==Left_Island_in
     ||Island.State==Right_Island_in)
  {
    Speed_goal2 = Island.Speed;
    Motor2_PID.LOW_Limit = 0;
  }
  else if(Start_line.Square_flag)
  {
    Speed_goal2 = 0;
    Motor2_PID.LOW_Limit = -65500;
  }
  else if(Podao.Stay_flag == 1
          ||Podao.Verify_flag==1)
  {
    Speed_goal2 = Podao.Speed;
    Motor2_PID.LOW_Limit = 0;
  }
  else 
  {
    Speed_goal2 = Speed_stand;
    Motor2_PID.LOW_Limit = 0;
  }
  Motor2_PID.target = Speed_goal2;
  PID_process(&Motor2_PID);
  Duty_Motor2 = Motor2_PID.result;// + Diff_PID.result;
  
  
  if(Start_line.Square_flag)
  {
    if(Motor2_PID.result>=0)
    {
      MOTOR2_DIR = 1;
      if(Motor2_PID.result>=65500)Motor2_PID.result = 65500;
      else if(Motor2_PID.result<=50)Motor2_PID.result = 50;
      
      Duty_Motor2 = 0xffff - Motor2_PID.result;//pwmԽ���ٶ�Խ��
      FTM_PWM_Duty(MOTOR_2,Duty_Motor2);
    }
    else
    {
      MOTOR2_DIR = 0;
      if(Motor2_PID.result<=-65500)Motor2_PID.result = -65500;
      else if(Motor2_PID.result>=-50)Motor2_PID.result = -50;
      
      Duty_Motor2 = 0xffff + Motor2_PID.result;//pwmԽ���ٶ�Խ��
      FTM_PWM_Duty(MOTOR_2,Duty_Motor2);
    }
  }
  else
  {
    MOTOR2_DIR = 1;
    if(Motor2_PID.result>=65500)Motor2_PID.result = 65500;
    else if(Motor2_PID.result<=50)Motor2_PID.result = 50;
    
    Duty_Motor2 = 0xffff - Motor2_PID.result;//pwmԽ���ٶ�Խ��
    FTM_PWM_Duty(MOTOR_2,Duty_Motor2);
  }
}


void AD_new(void)
{
  static u8  AD_Period = 0;
  static u16 L_AD[speed_Period_Constant],
              R_AD[speed_Period_Constant];
  if(AD_Period > (speed_Period_Constant - 1))
    AD_Period = 0;
  
  L_AD_Sum -= L_AD[AD_Period];
  R_AD_Sum -= R_AD[AD_Period];
  L_AD_Ave = ad_once(MYADC_1,ADC_12bit);
  R_AD_Ave = ad_once(MYADC_2,ADC_12bit);
  L_AD[AD_Period] = L_AD_Ave;
  R_AD[AD_Period] = R_AD_Ave;
  L_AD_Sum += L_AD[AD_Period];
  R_AD_Sum += R_AD[AD_Period];
  AD_Period++;  
}

void Servo_Diff_PID(void)
{
  Servo_PID.feedback = Diff_error;
  if(In_double_AD()==1
     ||Island.State==Left_Island_pre
     ||Island.State==Left_Island_out
     ||Island.State==Left_Wait_Next
     ||Island.State==Left_Island_in)
  {
    Servo_PID.P = Island.L_P;
    Servo_PID.D = Island.L_D;
  }
  else if(In_double_AD()==1
     ||Island.State==Right_Island_pre
     ||Island.State==Right_Island_out
     ||Island.State==Right_Wait_Next
     ||Island.State==Right_Island_in)
  {
    Servo_PID.P = Island.R_P;
    Servo_PID.D = Island.R_D;
  }
  else if(Podao.Stay_flag == 1
          ||Podao.Verify_flag==1)
  {
    Servo_PID.P = 1.8;
    Servo_PID.D = 3;
  }
  else
  {
    Servo_PID.P = stand_p;
    Servo_PID.D = stand_d;
  }
  Diff_PID_Process(&Servo_PID);
  FTM_PWM_Duty(STEER_,Servo_PID.result+SERVO_MIDDLE);
}


//��Ⱥ� 

//��Ⱥ͵�Ŀ�����ڰ���������еĲ�Ⱥ͵�ֵ�복��λ����������صķ�ʽ���ֳ�����Ȼ��ʵ����
//���ֵ�ڳ��������ĵĵط��ܹ��ܺõ����Ա������������Ե�ط�����ƫ������������ң������ڵ��������һ��ֵ�ļ�С��
//��Ⱥͷ�ĸ�����С���������ƴ�ʱ���������Լ�С��������ʱ����������������������Ҫ�����޷�����
void get_center()
{
  static int delta = 500;
  int best_data[2];
  best_data[0] = R_AD_Ave;
  best_data[1] = L_AD_Ave;

    if(Min_2_num(best_data[0],best_data[1])<300)//��ֹ��������һ�����ֵ��Ϊ̫С��ʧЧ�Ĵ��� 400��ֵ�ĵ���Ӧ���ǿ������Ⱥ�����ͼ���쳣�仯֮ǰ���ҵĵ�й�һ����ֵ��3100��
    {
      if(Abs_(best_data[0]-best_data[1]) > Abs_(delta))//��������һ�����ֵ̫Сʱ��ֻ�е���ֵ������ʷֵ������Ч
      {
        if((best_data[0]-best_data[1])*delta > 0)    //���Ҵ�ʱ��Ϊ�����ܳ�ͻȻ����ƫ��䵽��ƫ�����������һ������>�ң��´���>�󣩣������ҲҪ����
        {
          delta=best_data[0]-best_data[1];
        }
        else ;
      }  
      else ;
    }
    else
    {
      delta=best_data[0]-best_data[1];
    }		
    now_err=(int)(400*(delta)/(best_data[0]+best_data[1]+1));//��Ⱥ�
    if(now_err>500)			//330
    {
      now_err=500;
    }
    else;
    if(now_err<-500)
    {
      now_err=-500;
    }
    else;
    Diff_error = now_err;
}
