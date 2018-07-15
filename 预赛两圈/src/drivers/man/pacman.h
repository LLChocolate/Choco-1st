#include "lcd.h"
#ifndef PACMAN
#define PACMAN
#define POINT_COLOR Brush_Color
#define delay_ms delayms
#define LCD_DrawLine LCD_Draw_Line

#define div_x(t) 117+(t-1)*7+3 //由网格地址获得物理地址
#define div_y(t) 5+(t-1)*7+3

#define true_x(t) (t-120)/7+1  //由物理地址获得网格地址
#define true_y(t) (t-8)/7+1


void LCD_Draw_Pacman(char e);
void LCD_Draw_a(u16 x0,u16 y0);
void LCD_Move_Pacman();
char LCD_border(int x0,int y0);
void xy_change();
void Draw_Map();
void xy_judge();
void pacman_init();
void pacman_gaming();
void LCD_Draw_Circle(u16 x0,u16 y0,u8 r);
void Ghosts_change();
void match_d(int x0,int y0,int num);
void Draw_Wall(int x1,int y1,int x2,int y2);
void Draw_Walls();
void Draw_beans();
//void LCD_Remove();
char btn_scan();
void Ghosts_move();

#endif
