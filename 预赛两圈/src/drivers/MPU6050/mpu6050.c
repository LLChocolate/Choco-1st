#include "mpu6050.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK精英STM32开发板V3
//MPU6050 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/1/17
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
 
//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init(void)
{ 
	u8 res;
	
	MPU_IIC_Init();//初始化IIC总线
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
        delayms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正确  MPU_ADDR
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(50);						//设置采样率为50Hz
 	}else return 1;
	return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//发送数据
		if(MPU_IIC_Wait_Ack())		//等待ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=MPU_IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//产生一个停止条件 
	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答 
	MPU_IIC_Send_Byte(data);//发送数据
	if(MPU_IIC_Wait_Ack())	//等待ACK
	{
		MPU_IIC_Stop();	 
		return 1;		 
	}		 
    MPU_IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	MPU_IIC_Wait_Ack();		//等待应答 
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 
	res=MPU_IIC_Read_Byte(0);//读取数据,发送nACK 
    MPU_IIC_Stop();			//产生一个停止条件 
	return res;		
}

void mix_gyrAcc_crossMethod(float * attitude,short gyr[3],short acc[3])
{
const static float FACTOR = 0.001;//两个重力矢量叉积后所乘的系数p，用于和陀螺仪积分角度相叠加来修正陀螺仪（这里只用了比例p，没用积分i，）
//FACTOR 为1，则完全信任加速度计，为0，则完全信任陀螺仪
float w_q = attitude[0];//w=cos(alpha/2)
float x_q = attitude[1];//x=ax*sin(alpha/2)
float y_q = attitude[2];//y=ay*sin(alpha/2)
float z_q = attitude[3];//z=az*sin(alpha/2)
float x_q_2 = x_q * 2;
float y_q_2 = y_q * 2;
float z_q_2 = z_q * 2;
//
// 加速度计的读数，单位化。
float norm = sqrt((acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2])*1.0);
float x_aa = acc[0]*1.0 * norm;
float y_aa = acc[1]*1.0 * norm;
float z_aa = acc[2]*1.0 * norm;
//
// 载体坐标下的重力加速度常量，单位化。//用旋转矩阵将世界坐标系的单位化重力矢量(0,0,1）不是（0,0，-1）,mpu6050 只感应非重力加速度)转换到机载坐标系中。
//机载坐标下的重力矢量旋转矩阵（坐标系转换矩阵的逆矩阵也就是转置矩阵，因为欧拉角解得的旋转矩阵必是正交阵） 世界坐标下的重力矢量
// x cos(T)cos(K) cos(T)sin(C) -sin(C) 0
// [ y ] = [ sin(F)sin(T)cos(K)-cos(F)sin(K) sin(F)sin(T)sin(K)+cos(F)cos(K) sin(F)cos(T) ] * [ 0 ]
// z cos(F)sin(T)cos(K)+sin(F)sin(K) cos(F)sin(T)sin(K)-sin(F)cos(K) cos(F)cos(T) 1
//K 是yaw,T 是pitch,F 是roll,旋转顺序为ZYX
// w^2+x^2-y^2-z^2 2*(x*y+w*z) 2*(x*z-w*y)
//上式中的旋转矩阵用四元数表示即为：[ 2*(x*y-w*z) w^2-x^2+y^2-z^2 2*(y*z+w*x) ]
// 2*(x*z+w*y) 2*（y*z-w*x） w^2-x^2-y^2+z^2
//
float x_ac = x_q*z_q_2 - w_q*y_q_2;// 2*(x*z-w*y) =ax*az(1-cos(alpha))-ay*sin(alpha)
float y_ac = y_q*z_q_2 + w_q*x_q_2;// 2*(y*z+w*x) =az*ay(1-cos(alpha))+ax*sin(alpha)
float z_ac = 1 - x_q*x_q_2 - y_q*y_q_2;// w^2+x^2-y^2-z^2 =1-2*x^2-2*y^2 = cos(alpha)+(1-cos(alpha)*z^2)
//
// 测量值与常量的叉积。//测量值叉乘常量值,并以此向量表示误差角度大小与转轴方向，用于修正陀螺仪积分角度
float x_ca = y_aa * z_ac - z_aa * y_ac;
float y_ca = z_aa * x_ac - x_aa * z_ac;
float z_ca = x_aa * y_ac - y_aa * x_ac;
//
// 构造增量旋转。//可看成分别绕xyz 轴的三次旋转的叠加。sin(delta/2)近似为delta/2,cos(delta/2)近似为0
float delta_x = gyr[0]*1.0 * halfT + x_ca * FACTOR;//绕x 轴旋转角度的一半，记d_x 看作绕x 轴的一次旋转：w=1,x=d_x,y=0,z=0
float delta_y = gyr[1]*1.0 * halfT + y_ca * FACTOR;//绕y 轴旋转角度的一半，记d_y 看作绕y 轴的一次旋转：w=1,x=0,y=d_y,z=0
float delta_z = gyr[2]*1.0 * halfT + z_ca * FACTOR;//绕z 轴旋转角度的一半，记d_z 看作绕z 轴的一次旋转：w=1,x=0,y=0,z=d_z
//三次旋转叠加为一次旋转，即三个四元数相乘
//四元数乘法公式：q3=q1*q2
//(w1*w2 - x1*x2 - y1*y2 - z1*z2) = w3
//(w1*x2 + x1*w2 + y1*z2 - z1*y2) = x3
//(w1*y2 - x1*z2 + y1*w2 + z1*x2) = y3
//(w1*z2 + x1*y2 - y1*x2 + z1*w2) = z3
//合成的一次旋转：
// w=1 - d_x*d_y*d_z(多个小角度相乘，忽略，下同) =1
// x=d_x + d_y*d_z（忽略） =d_x
// y=d_y - d_x*d_z（忽略） =d_y
// z=d_z + d_x*d_y（忽略） =d_z
//
// 融合，四元数乘法。//将上面合成的旋转四元数与之前的姿态四元数相乘，得到新的姿态四元数并归一化为单位四元数。
attitude[0] = w_q - x_q*delta_x - y_q*delta_y - z_q*delta_z;
attitude[1] = w_q*delta_x + x_q + y_q*delta_z - z_q*delta_y;
attitude[2] = w_q*delta_y - x_q*delta_z + y_q + z_q*delta_x;
attitude[3] = w_q*delta_z + x_q*delta_y - y_q*delta_x + z_q;
norm=sqrt(attitude[0]*attitude[0]+attitude[1]*attitude[1]+attitude[2]*attitude[2]+attitude[3]*attitude[3]);
attitude[0]=norm*attitude[0];
attitude[1]=norm*attitude[1];
attitude[2]=norm*attitude[2];
attitude[3]=norm*attitude[3];
		Pitch = asin(-2 * attitude[1] * attitude[3] + 2 * attitude[0]* attitude[2])* 57.3;	// pitch
		Roll  = atan2(2 * attitude[2] * attitude[3] + 2 * attitude[0] * attitude[1], -2 * attitude[1] * attitude[1] - 2 * attitude[2]* attitude[2] + 1)* 57.3;	// roll
		Yaw   = atan2(2*(attitude[1]*attitude[2] + attitude[0]*attitude[3]),attitude[0]*attitude[0]+attitude[1]*attitude[1]-attitude[2]*attitude[2]-attitude[3]*attitude[3]) * 57.3;	//yaw
//四元数归一化
}