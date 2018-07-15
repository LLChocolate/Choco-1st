#include "include.h"
#include "pacman.h"



int x;	//吃豆人当前x坐标
int y;// 吃豆人当前y坐标
int speed=7;//吃豆人歩进距离：1格
int di=2;//初始方向：左
char t = 3;//
char score = 0;
char energy;
char power;
char check;
char btn_1;
int  checknum;
int  checknum2;
char gg[] = "game over!";
char win[] = "you win!";
int gx[4] = {13,13,13,13};
int gy[4] = {15,15,15,15};//chushengdian 14 12
int gd[4] ={ 4,4,4,4};
char born;
char beanmap[868]={
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,1,0,0,0,0,1,0,
    0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,1,0,0,0,0,1,0,
    0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,1,0,0,0,0,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,1,0,
    0,1,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,1,0,
    0,1,1,1,1,1,1,0,0,1,1,1,1,0,0,1,1,1,1,0,0,1,1,1,1,1,1,0,
    0,0,0,0,0,0,1,0,0,0,0,0,2,0,0,2,0,0,0,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,0,0,0,2,0,0,2,0,0,0,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,2,2,2,2,2,2,2,2,2,2,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,2,0,0,0,0,0,0,0,0,2,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,2,0,3,3,3,3,3,3,0,2,0,0,1,0,0,0,0,0,0,
    2,2,2,2,2,0,1,2,2,2,0,3,3,3,3,3,3,0,2,2,2,1,0,2,2,2,2,2,
    0,0,0,0,0,0,1,0,0,2,0,3,3,3,3,3,3,0,2,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,2,0,0,0,0,0,0,0,0,2,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,2,2,2,2,2,2,2,2,2,2,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,2,0,0,0,0,0,0,0,0,2,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,0,2,0,0,0,0,0,0,0,0,2,0,0,1,0,0,0,0,0,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,1,0,0,0,0,1,0,
    0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,1,0,0,0,0,1,0,
    0,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,0,
    0,0,0,1,0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,1,0,0,1,0,0,0,
    0,0,0,1,0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,1,0,0,1,0,0,0,
    0,1,1,1,1,1,1,0,0,1,1,1,1,0,0,1,1,1,1,0,0,1,1,1,1,1,1,0,
    0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,
    0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};
char LCD_border(int x0,int y0)//判断
{
	switch(di)
	{
		case 0:
                        checknum = x0;
			if(x0<x+4)
				return 1;
			else 
				return 0;
		case 1:
                  checknum = y0;
			if(y0<y+4)
				return 1;
			else
				return 0;
		case 2:
                  checknum = x0;
			if(x0>x-4)
				return 1;
			else
				return 0;
		case 3:
                  checknum = y0;
			if(y0>y-4)
				return 1;
			else 
				return 0;
	}
	return 0;
}		
//在指定位置画一个指定弧线
//(x,y):中心点
//r    :半径
void LCD_Draw_a(u16 x0,u16 y0)
{
	int a,b;
        int dir;
	u8 r = 6;
	a=0;b=r;
	dir=3-(r<<1);             //判断下个点位置的标志
	while(a<=b)
	{
		if(LCD_border(x+a,y-b))
		LCD_DrawPoint(x+a,y-b);             //5
		if(LCD_border(x+b,y-a))
 		LCD_DrawPoint(x+b,y-a);             //0   
                if(LCD_border(x+b,y+a))
		LCD_DrawPoint(x+b,y+a);             //4   
		if(LCD_border(x+a,y+b))
		LCD_DrawPoint(x+a,y+b);             //6 
		if(LCD_border(x-a,y+b))
		LCD_DrawPoint(x-a,y+b);             //1  
		if(LCD_border(x-b,y+a))
 		LCD_DrawPoint(x-b,y+a);             
		if(LCD_border(x-a,y-b))
		LCD_DrawPoint(x-a,y-b);             //2   
		if(LCD_border(x-b,y-a))
  		LCD_DrawPoint(x-b,y-a);             //7     	         
		a++;
		//使用Bresenham算法画圆
		if(dir<0)dir +=4*a+6;
		else
		{
                    dir+=10+4*(a-b);
                    b--;
		}
	}
}
void LCD_Draw_Pacman(char e)
{
	if(e)
        {
          if(power)
            POINT_COLOR = RED;
          else
            POINT_COLOR = BLACK;
        }
	else 
		POINT_COLOR = WHITE;
	switch(di)
	{
		case 0:
			LCD_Draw_a(x,y);
                        checknum = x+4;
                        checknum2 =y+4;
			LCD_DrawLine(x,y,checknum,checknum2);
			LCD_DrawLine(x,y,x+4,y-4);
			break;
		case 1:
			LCD_Draw_a(x,y);
			LCD_DrawLine(x,y,x-4,y+4);
			LCD_DrawLine(x,y,x+4,y+4);
			break;			
		case 2:
			LCD_Draw_a(x,y);
			LCD_DrawLine(x,y,x-4,y+4);
			LCD_DrawLine(x,y,x-4,y-4);
			break;			
		case 3:
			LCD_Draw_a(x,y);
			LCD_DrawLine(x,y,x-4,y-4);
			LCD_DrawLine(x,y,x+4,y-4);
			break;
	}
}
void LCD_Draw_Ghost(int num,char e)
{
  if(e)
  {
    switch(num)
    {
    case 0:
      POINT_COLOR = BLUE;
      break;
    case 1:
      POINT_COLOR = BLUE2;
      break;
    case 2:
      POINT_COLOR = GREEN;
      break;
    case 3:
      POINT_COLOR = CYAN;
      break;
    }
  }
  else 
    POINT_COLOR = WHITE;
  LCD_Draw_Circle(gx[num],gy[num],5);   
}
void LCD_Move_Pacman()
{
  int ti;
  POINT_COLOR = WHITE;
  LCD_Draw_Circle(x,y,1);
  POINT_COLOR = BLACK;
  LCD_Draw_Pacman(1);
  for(ti = 0;ti < born;ti++)
    LCD_Draw_Ghost(ti,1);
}

void Ghosts_move()
{
  int ti;
    for(ti = 0;ti < 4;ti++)
    {
      match_d(x,y,ti);
      switch(gd[ti])
      {
      case 0:
        gx[ti]+=7;
        break;
      case 1:
        gy[ti]+=7;
        break;
      case 2:
        gx[ti]-=7;
        break;
      case 3:
        gy[ti]-=7;
        break;
      default:
        break;
      }
      if(x==gx[ti]&&y==gy[ti])
            gd[ti] = 5;
    }
}

void Ghost_born()
{
  gx[born] = div_x(14);
  gy[born] = div_y(12);
  gd[born] = 0;
  born++;
}

void Ghosts_change()
{
   if(score/30+1-born)
     Ghost_born();
   Ghosts_move();
    
}
void LCD_Remove()
{
  int ti;
  LCD_Draw_Pacman(0);
  for(ti = 0;ti < born;ti++)
    LCD_Draw_Ghost(ti,0);
}
void xy_change()
{
        int t_x,t_y,ti;
        char gnum;
        t_x = true_x(x);
        t_y = true_y(y);
        ti = (t_y-1)*28+t_x-1;
	switch(di)
	{
		case 0:
			if(beanmap[ti+1]!=0&&beanmap[ti+1]!=3)
                          x+=speed;
			break;
		case 1:
			if(beanmap[ti+28]!=0&&beanmap[ti+28]!=3)
                          y+=speed;
			break;
		case 2:
			if(beanmap[ti-1]!=0&&beanmap[ti-1]!=3)
                          x-=speed;
			break;
		case 3:
			if(beanmap[ti-28]!=0&&beanmap[ti-28]!=3)
                          y-=speed;
			break;
	}
        t_x = true_x(x);
        t_y = true_y(y);
        ti = (t_y-1)*28+t_x-1;
        
        if(beanmap[ti]==1)
        {
          beanmap[ti] = 4;
          score++;
          energy++;
          if(energy >=5)
          {
            energy = 0;
            power = 4;
          }
        }
        
        for(gnum = 0;gnum<4;gnum++)
        {
          if(x==gx[gnum]&&y==gy[gnum])
              gd[born] = 5;            
        }
        if(power >0)
          power--;
}
void match_d(int x0,int y0,int num)
{
    int d1,d2;
    int t_x,t_y,ti,turni;
    char rd[4];
    //已知当前方向

    //获取当前坐标
    t_x = true_x(gx[num]);
    t_y = true_y(gy[num]);
    ti = (t_y-1)*28+t_x-1;

	
     if(beanmap[ti]==3)
    {
        gd[num] = 4;
        return;
    }
    
    //获取可以去的方向
    if(beanmap[ti+1]!=0&&beanmap[ti+1]!=3)
    {
        if(gd[num]!=2)
            rd[0] = 1;
        else
            rd[0] = 0;
    }
    else
        rd[0] = 0;
    if(beanmap[ti+28]!=0&&beanmap[ti+28]!=3)
    {
        if(gd[num]!=3)
            rd[1] = 1;
        else
          rd[1] = 0;
    }
        
    else 
        rd[1] = 0;
    if(beanmap[ti-1]!=0&&beanmap[ti-1]!=3)
    {
        if(gd[num]!=0)
            rd[2] = 1;
        else
          rd[2] =0;
    }
    else 
        rd[2] = 0;
    if(beanmap[ti-28]!=0&&beanmap[ti-28]!=3)
    {
        if(gd[num]!=1)
            rd[3] = 1;
        else
          rd[3] = 0;
    }
    else
        rd[3] = 0;
   
            

    //获取想去的方向
    if(true_x(x0)>true_x(gx[num]))
        d1 = 1;
    else if(true_x(x0)==true_x(gx[num]))
        d1 = 0;
    else
        d1 = -1;
    if(true_y(y0)>true_y(gy[num]))
        d2 = 1;
    else if(true_y(y0)==true_y(gy[num]))
        d2 = 0;
    else
        d2 = -1;
    
    
    if(d1==0&&d2==0)
    {
      gd[num] = 5;
      return;
    }
    
    
    
    if((rd[0]+rd[1]+rd[2]+rd[3])>1)
    {
        if(d1 == 1)
        {
            if(d2 == 1)
            {
                if(rd[0])
                {
                    gd[num] = 0;
                    return;
                }
                if(rd[1])
                {
                    gd[num] = 1;
                    return;
                }
                if(rd[2])
                {
                        gd[num] = 2;
                        return;
                }
                if(rd[3])
                {
                        gd[num] = 3;
                        return;
                }
            }
            if(d2 == 0)
            {
                if(rd[0])
                {
                    gd[num] = 0;
                    return;
                }
                if(rd[1])
                {
                    gd[num] = 1;
                    return;
                }
                if(rd[2])
                {
                    gd[num] = 2;
                    return;
                }
                if(rd[3])
                {
                    gd[num] = 3;
                    return;
                }
            }
            if(d2 == -1)
            {
                if(rd[0])
                {
                    gd[num] = 0;
                    return;
                }
                if(rd[3])
                {
                    gd[num] = 3;
                    return;
                }
                if(rd[1])
                {
                    gd[num] = 1;
                    return;
                }
                
                if(rd[2])
                {
                    gd[num] = 2;
                    return;
                }
            }
        }
        if(d1 == 0)
        {
            if(d2 == 1)
            {
                if(rd[1])
                {
                    gd[num] = 1;
                    return;
                }
                if(rd[0])
                {
                    gd[num] = 0;
                    return;
                }
                
                if(rd[2])
                {
                    gd[num] = 2;
                    return;
                }
                if(rd[3])
                {
                    gd[num] = 3;
                    return;
                }
            }
            if(d2 == -1)
            {
                if(rd[3])
                {
                    gd[num] = 3;
                    return;
                }
                if(rd[0])
                {
                    gd[num] = 0;
                    return;
                }
                
                if(rd[1])
                {
                    gd[num] = 1;
                    return;
                }
                
                if(rd[2])
                {
                    gd[num] = 2;
                    return;
                }
            }
        }
        if(d1 == -1)
        {
            if(d2 == 1)
            {
                if(rd[2])
                {
                    gd[num] = 2;
                    return;
                }
                if(rd[0])
                {
                    gd[num] = 0;
                    return;
                }
                if(rd[1])
                {
                    gd[num] = 1;
                    return;
                }
                
                if(rd[3])
                {
                    gd[num] = 3;
                    return;
                }
            }
            if(d2 == 0)
            {
                if(rd[2])
                {
                    gd[num] = 2;
                    return;
                }
                if(rd[0])
                {
                    gd[num] = 0;
                    return;
                }
                if(rd[1])
                {
                    gd[num] = 1;
                    return;
                }
                
                if(rd[3])
                {
                    gd[num] = 3;
                    return;
                }
            }
            if(d2 == -1)
            {
                if(rd[2])
                {
                    gd[num] = 2;
                    return;
                }
                if(rd[3])
                {
                    gd[num] = 3;
                    return;
                }
                if(rd[0])
                {
                    gd[num] = 0;
                    return;
                }
                if(rd[1])
                {
                    gd[num] = 1;
                    return;
                }
            }
        }
    }
    else
        for(turni = 0;turni<4;turni++)
            if(rd[turni])
                gd[num] = turni;
}
void Draw_Wall(int x1,int y1,int x2,int y2)
{
  LCD_DrawLine(117+(x1-1)*7+3,5+(y1-1)*7+3,117+(x2-1)*7+3,5+(y2-1)*7+3);
}
void Draw_Walls()
{
  Draw_Wall(1,1,1,10);//AB
  Draw_Wall(1,10,6,10);//bc
  Draw_Wall(6,10,6,14);//cd
  Draw_Wall(6,14,1,14);//de
  Draw_Wall(1,16,6,16);//fg
  Draw_Wall(6,16,6,20);//gh
  Draw_Wall(6,20,1,20);//hi
  Draw_Wall(1,20,1,25);//ij
  Draw_Wall(1,25,3,25);//jk
  Draw_Wall(3,25,3,26);//kl
  Draw_Wall(3,26,1,26);//lm
  Draw_Wall(1,26,1,31);//mn
  Draw_Wall(1,31,28,31);//no
  Draw_Wall(28,31,28,26);//op
  Draw_Wall(28,26,26,26);//pq
  Draw_Wall(26,26,26,25);//qr
  Draw_Wall(26,25,28,25);//rs
  Draw_Wall(28,25,28,20);//st
  Draw_Wall(28,20,23,20);//tu
  Draw_Wall(23,20,23,16);//uv
  Draw_Wall(23,16,28,16);//vw
  Draw_Wall(28,14,23,14);//xy
  Draw_Wall(23,14,23,10);//yz
  Draw_Wall(23,10,28,10);//zaa
  Draw_Wall(28,10,28,1);//aaab
  Draw_Wall(28,1,15,1);//abac
  Draw_Wall(15,1,15,5);//acad
  Draw_Wall(15,5,14,5);//adae
  Draw_Wall(14,5,14,1);//aeaf
  Draw_Wall(14,1,1,1);//afaa
  
  Draw_Wall(3,3,6,3);//a
  Draw_Wall(6,3,6,5);
  Draw_Wall(6,5,3,5);
  Draw_Wall(3,5,3,3);
  
  Draw_Wall(8,3,12,3);//b
  Draw_Wall(12,3,12,5);
  Draw_Wall(12,5,8,5);
  Draw_Wall(8,5,8,3);
  
  Draw_Wall(17,3,21,3);//c
  Draw_Wall(21,3,21,5);
  Draw_Wall(21,5,17,5);
  Draw_Wall(17,5,17,3);
  
  Draw_Wall(23,3,26,3);//d
  Draw_Wall(26,3,26,5);
  Draw_Wall(26,5,23,5);
  Draw_Wall(23,5,23,3);
  
  Draw_Wall(3,7,6,7);//e
  Draw_Wall(6,7,6,8);
  Draw_Wall(6,8,3,8);
  Draw_Wall(3,8,3,7);
  
  Draw_Wall(23,7,26,7);//f
  Draw_Wall(26,7,26,8);
  Draw_Wall(26,8,23,8);
  Draw_Wall(23,8,23,7);
  
  Draw_Wall(8,16,9,16);//g
  Draw_Wall(9,16,9,20);
  Draw_Wall(9,20,8,20);
  Draw_Wall(8,20,8,16);
  
  Draw_Wall(20,16,21,16);//h
  Draw_Wall(21,16,21,20);
  Draw_Wall(21,20,20,20);
  Draw_Wall(20,20,20,16);
  
  Draw_Wall(8,22,12,22);//i
  Draw_Wall(12,22,12,23);
  Draw_Wall(12,23,8,23);
  Draw_Wall(8,23,8,22);
  
  Draw_Wall(17,22,21,22);//j
  Draw_Wall(21,22,21,23);
  Draw_Wall(21,23,17,23);
  Draw_Wall(17,23,17,22);
  
  Draw_Wall(8,7,9,7);
  Draw_Wall(9,7,9,10);
  Draw_Wall(9,10,12,10);
  Draw_Wall(12,10,12,11);
  Draw_Wall(12,11,9,11);
  Draw_Wall(9,11,9,14);
  Draw_Wall(9,14,8,14);
  Draw_Wall(8,14,8,7);
  
  Draw_Wall(20,7,21,7);
  Draw_Wall(21,7,21,14);
  Draw_Wall(21,14,20,14);
  Draw_Wall(20,14,20,11);
  Draw_Wall(20,11,17,11);
  Draw_Wall(17,11,17,10);
  Draw_Wall(17,10,20,10);
  Draw_Wall(20,10,20,7);
  
  Draw_Wall(11,7,18,7);
  Draw_Wall(18,7,18,8);
  Draw_Wall(18,8,15,8);
  Draw_Wall(15,8,15,11);
  Draw_Wall(15,11,14,11);
  Draw_Wall(14,11,14,8);
  Draw_Wall(14,8,11,8);
  Draw_Wall(11,8,11,7);
  
  Draw_Wall(11,7+12,18,7+12);
  Draw_Wall(18,7+12,18,8+12);
  Draw_Wall(18,8+12,15,8+12);
  Draw_Wall(15,8+12,15,11+12);
  Draw_Wall(15,11+12,14,11+12);
  Draw_Wall(14,11+12,14,8+12);
  Draw_Wall(14,8+12,11,8+12);
  Draw_Wall(11,8+12,11,7+12);
  
  Draw_Wall(11,7+18,18,7+18);
  Draw_Wall(18,7+18,18,8+18);
  Draw_Wall(18,8+18,15,8+18);
  Draw_Wall(15,8+18,15,11+18);
  Draw_Wall(15,11+18,14,11+18);
  Draw_Wall(14,11+18,14,8+18);
  Draw_Wall(14,8+18,11,8+18);
  Draw_Wall(11,8+18,11,7+18);
  
  
  Draw_Wall(3,28,8,28);
  Draw_Wall(8,28,8,25);
  Draw_Wall(8,25,9,25);
  Draw_Wall(9,25,9,28);
  Draw_Wall(9,28,12,28);
  Draw_Wall(12,28,12,29);
  Draw_Wall(12,29,3,29);
  Draw_Wall(3,29,3,28);
  
  Draw_Wall(17,28,20,28);
  Draw_Wall(20,28,20,25);
  Draw_Wall(20,25,21,25);
  Draw_Wall(21,25,21,28);
  Draw_Wall(21,28,26,28);
  Draw_Wall(26,28,26,29);
  Draw_Wall(26,29,17,29);
  Draw_Wall(17,29,17,28);

  
  
  Draw_Wall(3,22,6,22);
  Draw_Wall(6,22,6,26);
  Draw_Wall(6,26,5,26);
  Draw_Wall(5,26,5,23);
  Draw_Wall(5,23,3,23);
  Draw_Wall(3,23,3,22);
  
  Draw_Wall(29-3,22,29-6,22);
  Draw_Wall(29-6,22,29-6,26);
  Draw_Wall(29-6,26,29-5,26);
  Draw_Wall(29-5,26,29-5,23);
  Draw_Wall(29-5,23,29-3,23);
  Draw_Wall(29-3,23,29-3,22);
  
  Draw_Wall(13,13,11,13);
  Draw_Wall(11,13,11,17);
  Draw_Wall(11,17,18,17);
  Draw_Wall(18,17,18,13);
  Draw_Wall(18,13,16,13);
  
}
void LCD_Draw_Circle(u16 x0,u16 y0,u8 r)
{
	int a,b;
	int dir;
	a=0;b=r;	  
	dir=3-(r<<1);             //?D??????μ?????μ?±ê??
	while(a<=b)
	{
		LCD_DrawPoint(x0+a,y0-b);             //5
 		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-a,y0+b);             //1       
 		LCD_DrawPoint(x0-b,y0+a);             
		LCD_DrawPoint(x0-a,y0-b);             //2             
  		LCD_DrawPoint(x0-b,y0-a);             //7     	         
		a++;
		//ê1ó?Bresenham??·¨?-?2     
		if(dir<0)dir +=4*a+6;	  
		else
		{
			dir+=10+4*(a-b);   
			b--;
		} 						    
	}
} 				
void Draw_beans()
{
  int t_x,t_y;
  int ti;
  for(ti = 0;ti<868;ti++)
  {
    if(beanmap[ti] == 1)
    {
        t_y = ti/28+1;
        t_x = ti%28+1;
        LCD_Draw_Circle(div_x(t_x),div_y(t_y),1);
    }
  }
    
}
void Draw_Map()
{
    Draw_Walls();
    Draw_beans();  
}
char btn_scan()
{
  btn_1 = 0;
  check = gpio_get(PORTE,26);
  if(check == LOW)
  {
    delayms(50);
    check = gpio_get(PORTE,26);
    if(check == LOW)
    {
      btn_1 = 1;
    }
  }
  else btn_1 = 0;
  check = gpio_get(PORTE,28);
  if(check == LOW)
  {
    delayms(50);
    check = gpio_get(PORTE,28);
    if(check == LOW)
    {
      return 2+btn_1;
    }
  }
  else return btn_1;
  return 0;
}
void pacman_init()
{

	int ti;
	//地图元素初始化
	for(ti = 0;ti < 868;ti++)
	{
		if(beanmap[ti] == 4)
		{
			beanmap[ti] = 1;
		}
	}
	//绘制地图；
        Brush_Color=Black;
    Draw_Map();
	//初始化pacman信息；
    x = div_x(14);
    y = div_y(30);
    di = 2;
    speed  = 7;
	score = 0;
	energy = 0;
	power = 0;
	//ghost信息初始化
	gx[0]=13;gx[1]=13;gx[2]=13;gx[3]=13;
	gy[0]=15;gy[1]=15;gy[2]=15;gy[3]=15;
	gd[0]=4;gd[1]=4;gd[2]=4;gd[3]=4;
	born = 0;

}
void pacman_gaming()
{
    int gt;
    t=KEY_Scan();
    switch(t)
    {				 
    case 1:
    
        if(di)
          di--;
        else
          di = 3;
        break;
    case 2:
    
        if(di<3)
          di++;
        else 
          di = 0;
        break;
  default:
        delayms(50);	
        break;
  } 
  
  xy_change();
  Ghosts_change();
  LCD_Move_Pacman();
  if(score>=100)
  {
    LCD_PutString(140,120,win,RED,WHITE);
    while(1)
    {
      if(KEY_Scan()==KEY0_PRES)
      {
        Mark_&=~(1<<3);
        break;
      }
    }
  }
  for(gt = 0;gt< 4;gt++)
  {
    if(gd[gt] == 5)
    {
      if(power)
      {
        LCD_Draw_Ghost(gt,0);
        gx[gt] = div_x(14);
        gy[gt] = div_y(12);
        gd[gt] = 0;
        
        score+=5;
        power+=5;
      }
      else
      {
        LCD_PutString(140,120,gg,RED,WHITE);
        while(1)
        {
          if(KEY_Scan()==KEY0_PRES)
          {
            Mark_&=~(1<<3);
            break;
          }
        }
      }
    }
  }
        
      
  delayms(250);
  LCD_Remove();
}
