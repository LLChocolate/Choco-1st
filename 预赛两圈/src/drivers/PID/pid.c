#include "pid.h" 

////PID结构体
//typedef struct                    //结构体，存放PID相关变量
//{
//  int P;                        //参数P
//  int I;                        //参数I
//  int D;                        //参数D
//  float error[3];                 //误差存储数组
//  float delat;                    //每次的调节结果
//  float derr;                     //一阶误差
//  float dderr;                    //二阶误差
//  signed long result;                      //PID的输出，由于是增量式的所以初值请设为导轨平衡时的输出值，重要！不能是0要不就炸了
//  
//  float target;                   //PID调节的目标值     
//  float feedback;
//  float UP_Limit;
//  float LOW_Limit;
//}PID_Struct;

void PID_process(PID_Struct* pid)                    //PID程序
{
    pid->error[0] = pid->target   - pid->feedback ;  //计算位置偏差（预计值-现在值）【如果不小心正反馈了可以反过来减】      
//    pid->error[0] = pid->feedback   - pid->target ;
    pid->derr     = pid->error[0] - pid->error[1];   //计算一阶误差
    pid->dderr    = pid->error[0] - 2*pid->error[1] + pid->error[2]; //计算二阶误差
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];  //用新误差更新老误差
    pid->delat    = pid->I*pid->error[0] + pid->P*pid->derr + pid->D*pid->dderr ;  //计算PID调节的结果
    pid->result  += pid->delat;                                          //用此结果作为增量调节
    if(pid->result>pid->UP_Limit)
    {
      pid->result=pid->UP_Limit;
    }
    else if(pid->result<pid->LOW_Limit)
    {
      pid->result=pid->LOW_Limit;
    }
    
}

void Diff_PID_Process(PID_Struct* pid)
{
  pid->error[2]=pid->error[1];//更新误差
  pid->error[1]=pid->error[0];
  pid->error[0]=pid->feedback;
  pid->result=pid->P*pid->error[0]+pid->D*(pid->error[0]-pid->error[1]);
  if(pid->result>pid->UP_Limit)
  {
    pid->result=pid->UP_Limit;
  }
  else if(pid->result<pid->LOW_Limit)
  {
    pid->result=pid->LOW_Limit;
  }
}

void PID_Init(PID_Struct* pid,float P_,float I_,float D_,signed long Result,float Target,float up_limit,float low_limit)
{
  pid->P=P_;
  pid->I=I_;
  pid->D=D_;
  pid->error[0]=pid->error[1]=pid->error[2]=0;
  pid->delat=0;
  pid->derr=0;
  pid->dderr=0;
  pid->result=Result;
  pid->target=Target;
  pid->feedback=0;
  pid->UP_Limit=up_limit;
  pid->LOW_Limit=low_limit;
}

