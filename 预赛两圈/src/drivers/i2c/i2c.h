/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名       ：i2c.h
 * 描述         ：I2C头文件
 *
 * 实验平台     ：野火kinetis开发板
 * 库版本       ：
 * 嵌入系统     ：
 *
 * 作者         ：
 * 淘宝店       ：http://firestm32.taobao.com
 * 技术支持论坛 ：http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/
#ifndef __I2C_H__
#define __I2C_H__
#include "i2c_cfg.h"

typedef enum I2Cn
{
    I2C0  = 0,
    I2C1  = 1
} I2Cn;

typedef enum MSmode
{
    MWSR =   0x00,  /* Master write  */
    MRSW =   0x01   /* Master read */
} MSmode;


void  I2C_init(I2Cn);                                         //初始化I2C
void  I2C_WriteAddr(I2Cn, unsigned char SlaveID, unsigned char Addr, unsigned char Data);      //读取地址里的内容
unsigned char    I2C_ReadAddr(I2Cn, unsigned char SlaveID, unsigned char Addr);                //往地址里写入内容





void  I2C_StartTransmission (I2Cn, unsigned char SlaveID, MSmode);       //启动传输
#endif