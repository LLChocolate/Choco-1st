#include "mymath.h"

int temp_1st[100];

int max_s16(int temp[],u16 num)
{
  u16 i;
  int max=-32768;
  for(i=0;i<num;i++)
  {
    if(max<temp[i])
    {
      max=temp[i];
    }
  }
  return max;
}

u8 max_u8(u8 temp[],u16 num)
{
  u16 i;
  u8 max=0;
  for(i=0;i<num;i++)
  {
    if(max<temp[i])
    {
      max=temp[i];
    }
  }
  return max;
}


u8 max_u8_index(u8 temp[],u16 num)
{
  u16 i;
  u8 max=0;
  u8 index=0;
  for(i=0;i<num;i++)
  {
    if(max<temp[i])
    {
      max=temp[i];
      index=i;
    }
  }
  return index;
}

u8 min_u8_index(u8 temp[],u16 num)
{
  u16 i;
  u8 min=0xff;
  u8 index=0;
  for(i=0;i<num;i++)
  {
    if(min>temp[i])
    {
      min=temp[i];
      index = i;
    }
  }
  return index;
}

u8 min_u8_index_RIsland(u8 temp[],u16 num)
{
  u16 i;
  u8 min=0xff;
  u8 index=0;
  for(i=0;i<num;i++)
  {
    if(min>=temp[i])
    {
      min=temp[i];
      index = i;
    }
  }
  return index;
}

u8 min_int_index(int temp[],u8 num)
{
  u8 i;
  int min=32767;
  u8 index=0;
  for(i=0;i<num;i++)
  {
    if(min>temp[i])
    {
      min=temp[i];
      index = i;
    }
  }
  return index;
}

int min_s16(int temp[],u16 num)
{
  u16 i;
  int min=32767;
  for(i=0;i<num;i++)
  {
    if(min>temp[i])
    {
      min=temp[i];
    }
  }
  return min;
}
u8 min_u8(u8 temp[],u16 num)
{
  u16 i;
  u8 min=0xff;
  for(i=0;i<num;i++)
  {
    if(min>temp[i])
    {
      min=temp[i];
    }
  }
  return min;
}

int power_s16(int buttom,u8 po)
{
  u8 i;
  int result=1;//0次幂为1
  for(i=0;i<po;i++)
  {
    result*=buttom;
  }
  return result;
}

int sum_s16(int input[],u16 num)
{
  u16 i;
  int sum=0;
  for(i=0;i<num;i++)
  {
    sum+=input[i];
  }
  return sum;
}
int sum_u8(u8 input[],u16 num)
{
  u16 i;
  int sum=0;
  for(i=0;i<num;i++)
  {
    sum+=input[i];
  }
  return sum;
}
int sum_point(u8 input[],u8 num)
{
  u8 i,j;
  int sum=0;
  for(i=0;i<num;i++)
  {
    for(j=0;j<8;j++)
    {
      sum+=(input[i]>>j)&1;
    }
  }
  return sum;
}

u16 abs_s16(int i)
{
    if(i>=0)return i;
  else return -i;
}


//signed char L_Diff_temp[70];
//signed char L_Second_Diff_temp[70];
//signed char R_Diff_temp[70];
//signed char R_Second_Diff_temp[70];
//u8 L_Diff_Test(int* Origin,u16 n,float* Circle_k,u8* _Turn)
//{
//  float k1,k2;
//  int sum_temp=0;
//  u8 turn_point=0;
//  u8 i,j,cnt=0,end_cnt=n;
//  for(i=0;i<n;i++)
//  {
//    L_Diff_temp[i]=Origin[i+1]-Origin[i];
//    if((Origin[i+1]>=320)||(Origin[i+1]<=0))
//    {
//      end_cnt = i;
//      break;
//    }
//  }
//  for(i=0;i<n-1;i++)
//  {
//    L_Second_Diff_temp[i]=L_Diff_temp[i+1]-L_Diff_temp[i];
//    if(abs_s16(L_Second_Diff_temp[i])<2)cnt++;
//    else if(abs_s16(L_Second_Diff_temp[i])>3)turn_point=i;
//  }
//  if(turn_point!=0)
//  {
//    *_Turn=turn_point;
//    sum_temp=0;
//    for(i=0;i<turn_point;i++)
//      sum_temp+=L_Diff_temp[i];
//    k1=sum_temp*1.0/turn_point;
//    sum_temp=0;
//    for(i=turn_point;i<end_cnt;i++)
//      sum_temp+=L_Diff_temp[i];
//    k2=sum_temp*1.0/(end_cnt-turn_point);
//    if(fabs(k1-k2)>2)
//    {
//      *Circle_k=k2;
//      return 2;
//    }
//  }
//  else if(cnt>(n*4.0/5))return 1;//线性的
//  else return 0;//非线性的
//}
//      
//u8 R_Diff_Test(int* Origin,u16 n,float* Circle_k,u8* _Turn)
//{
//  float k1,k2;
//  int sum_temp=0;
//  u8 turn_point=0;
//  u8 i,j,cnt=0,end_cnt=n;
//  for(i=0;i<n;i++)
//  {
//    R_Diff_temp[i]=Origin[i+1]-Origin[i];
//    if((Origin[i+1]>=319)||(Origin[i+1]<=0))
//    {
//      end_cnt = i;
//      break;
//    }
//  }
//  for(i=0;i<end_cnt-2;i++)
//  {
//    R_Second_Diff_temp[i]=R_Diff_temp[i+1]-R_Diff_temp[i];
//    if(abs_s16(R_Second_Diff_temp[i])<2)cnt++;
//    else if(abs_s16(R_Second_Diff_temp[i])>3)turn_point=i;
//  }
//  if(turn_point!=0)
//  {
//    *_Turn=turn_point;
//    sum_temp=0;
//    for(i=0;i<turn_point;i++)
//      sum_temp+=R_Diff_temp[i];
//    k1=sum_temp*1.0/turn_point;
//    sum_temp=0;
//    for(i=turn_point;i<end_cnt;i++)
//      sum_temp+=R_Diff_temp[i];
//    k2=sum_temp*1.0/(end_cnt-turn_point);
//    *Circle_k=k2;
//    return 2;
//  }
//  else if(cnt>(n*4.0/5))return 1;//线性的
//  else return 0;//非线性的
//}


float calculate_fangcha(s16 *a,u16 cnt)
{
  float sum = 0;
  float ave;
  u16 i;
  for (i = 0;i<cnt;i++)
  {
    sum += a[i];
  }
  ave = sum  / cnt;
  sum = 0;
  for (i = 0;i<cnt;i++)
  {
    sum += (a[i] - ave)*(a[i] - ave);
  }
  
  return sum/cnt;
}


float myInvSqrt(float x)	/*快速开平方求倒*/
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}


int ave_s16(int temp[],const u16 num)
{
  u16 i;
  int sum = 0;
  if(num==0)return -1;
  
  for(i=0;i<num;i++)
  {
    sum += temp[i];
  }
  
  return sum/num;
}

u8 ave_u8(u8 temp[],const u16 num)
{
  u16 i;
  u16 sum = 0;
  if(num==0)return -1;
  
  for(i=0;i<num;i++)
  {
    sum += temp[i];
  }
  
  return sum/num;
}


float fave_s16(int temp[],const u16 num)
{
  u16 i;
  int sum = 0;
  if(num==0)return -1;
  
  for(i=0;i<num;i++)
  {
    sum += temp[i];
  }
  
  return sum*1.0/num;
}

int _2nd_Diff(int temp[],const u16 num)
{
  u16 i;
  u16 cnt=0;
  int _2nd;
  for(i=0;i<num-1;i++)
  {
    temp_1st[i]=temp[i+1]-temp[i];
  }
  for(i=0;i<num-2;i++)
  {
    _2nd=temp_1st[i+1]-temp_1st[i];
    if(abs_s16(_2nd)>2)
      cnt++;
  }
  if(cnt<(num/6))
    return 1;//是线性
  else 
    return 0;//非线性
}