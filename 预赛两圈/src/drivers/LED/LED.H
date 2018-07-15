#ifndef __LED_H__
#define __LED_H__
#include "include.h"
#include "common.h"
#include "gpio.h"
#include "define.h"

void myLED_Init(void);
void water_LED(u16 ms);
void display_status(u8 status);
void LED_Open_once(void);

#endif