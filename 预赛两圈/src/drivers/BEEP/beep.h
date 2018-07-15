#ifndef __MYBEEP_H__
#define __MYBEEP_H__

#include "common.h"
#include "gpio.h"
#include "define.h"
#include "delay.h"

void myBEEP_Init(void);
void BEEP_Open_once(void);
void Beep_Once(Beep_Str* Beep);
void Beep_ISR(Beep_Str* Beep);
#endif
