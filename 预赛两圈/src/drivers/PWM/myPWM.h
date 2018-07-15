#ifndef __MYPWM_H__ 
#define __MYPWM_H__

#include "common.h"
#include "gpio.h"
#include "FTM.h"
#include "define.h"
#include "include.h"


void PWM_DISENABLE(FTMn ftmn, CHn ch);
void Motot2_Init(void);
void Motot1_Init(void);
void Steer_Init(void);

#endif
