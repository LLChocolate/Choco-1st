#ifndef __PID_H__
#define __PID_H__

#include "include.h"


void PID_process(PID_Struct* pid);
void PID_Init(PID_Struct* pid,float P_,float I_,float D_,signed long Result,float Target,float up_limit,float low_limit);
void Diff_PID_Process(PID_Struct* pid);
#endif