#ifndef __MYKEY_H__ 
#define __MYKEY_H__

#include "common.h"
#include "gpio.h"
#include "exti.h"
#include "define.h"
#include "include.h"

void myKEY_Init(void);
u8 KEY_Scan(void);
void myKEY_Exti_Init(void);

#endif
