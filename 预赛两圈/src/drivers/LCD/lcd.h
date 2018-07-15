#ifndef _LCD_H
#define _LCD_H

#include "include.h"
//颜色
#define White          0xFFFF
#define Black          0x0000
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0


//引脚连接定义//SDO引脚可以不连接
#define  reset   PTE10_OUT       //复位
#define  dcx     PTE11_OUT      //地址/数据切换
#define  sdi     PTE9_OUT      // SDI  MOSI
#define  scl     PTE8_OUT      //时钟

//位域
#define GET_BITFIELD(addr) (volatile bit_field *)(&addr)//返回的是指针
typedef struct bit_S
{
	unsigned char bit0:1;
	unsigned char bit1:1;
	unsigned char bit2:1;
	unsigned char bit3:1;
	unsigned char bit4:1;
	unsigned char bit5:1;
	unsigned char bit6:1;
	unsigned char bit7:1;
}bit_field;

//硬件底层
void LCD_PORT_init();
void LCD_Reset();
void LCD_init(unsigned char direction);

void write_command(unsigned char c);
void write_data(unsigned char  d); 
void write_word(unsigned int dat);

void RamAdressSet();
void LCD_SetPos(unsigned int x0,unsigned int x1,unsigned int y0,unsigned int y1);

//画图
void LCD_Draw_Line(u16 x1, u16 y1, u16 x2, u16 y2);
void LCD_DrawPoint(u16 x,u16 y);
void LCD_DrawBigPoint(u16 x,u16 y,u16 color);

//写文字
void LCD_PutChar(unsigned int x, unsigned int y,unsigned char c, unsigned int fColor, unsigned int bColor);
void LCD_PutString(unsigned int x, unsigned int y, unsigned char *s, unsigned int fColor, unsigned int bColor);
void LCD_PutChar8x16(unsigned int x, unsigned int y,unsigned char c, unsigned int fColor, unsigned int bColor);
void PutGB1616(unsigned int x, unsigned int  y, unsigned char c[2], unsigned int fColor,unsigned int bColor);
void PutGB3232(unsigned int x, unsigned int  y, unsigned char c[2], unsigned int fColor,unsigned int bColor);
void LCD_Put_Unsigned_Int(unsigned int x,unsigned int y,unsigned char s0[],unsigned int d,unsigned int fColor, unsigned int bColor);
void LCD_Put_Int(unsigned int x,unsigned int y,unsigned char s0[],int d,unsigned int fColor, unsigned int bColor);
void LCD_Put_Float(unsigned int x,unsigned int y,unsigned char s0[],float pnum,unsigned int fColor, unsigned int bColor);
void Disp_single_colour(unsigned int color);
void LCD_Fill_onecolor(u16 xx1 ,u16 yy1,u16 xx2, u16 yy2,u16 color);
void Draw_single_line(unsigned char axis,unsigned int line,unsigned int color);
void Print_child();
void Draw_line(unsigned int line,unsigned int color);
//调试显示函数
void SendPixel(unsigned char n);
void Parameters_On_LCD(unsigned char n );
void Print_runway(unsigned char CCD);
void Send_Image_to_LCD();
void ShowStart(void);
#if defined SmartCarPlatform
void number_input(u8 xx,u8 yy,u16* number);
#endif
#endif