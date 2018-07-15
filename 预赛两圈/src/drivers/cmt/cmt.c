#include "cmt.h"

vuint16 cmt_period;
     

//-------------------------------------------------------------------------------------------------------------------
//  @brief      CMT_PWM初始化 
//  @param      freq         设置PWM的频率
//  @param      duty         设置PWM的占空比
//  @return     void
//  @since      v1.0
//  Sample usage:               cmt_pwm_init(50,50);    // 频率50HZ，占空比为百分之（50/CMT_PRECISON *100）;
//------------------------------------------------------------------------------------------------------------------- 

void cmt_pwm_init(uint16 freq, uint16 duty)
{
    uint32 temp_clk;
    uint32 temp_high_num, temp_low_num;
    uint32 temp_div;
    //使能时钟
    SIM_SCGC4 |= 0x4u;
    //设置复用功能为CMT功能
//    port_init (PTD7, ALT2 | PULLUP );
    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK << (PTD7>>5) );
    PORT_PCR_REG(PORTX[PTD7>>5], PTD7&0x1f) = ALT2|PULLUP;
    
    PORTD_ISFR = (uint32)1<<(PTD7>>5);
    DDRD7 = 0;
    
    //使能输出
    CMT_OC |= 0x20u;
    
    //由于CMT模块会固定八分频，因此这里计算，便于后面使用
    temp_clk = bus_clk_khz*1000/8;
    
    //计算最佳分频
    temp_div = temp_clk/freq;
    temp_div = temp_div>>16;
    if(temp_div>0x0f)   temp_div = 0x0f;
        
    //设置分频
    CMT_PPS = CMT_PPS_PPSDIV(temp_div);
    
    //计算一个周期需要计数的次数
    cmt_period = temp_clk/(temp_div+1)/freq;

    //计算高低电平的计数次数
    temp_low_num = (cmt_period*(CMT_PRECISON-duty)/CMT_PRECISON);
    temp_high_num = (cmt_period*(duty)/CMT_PRECISON);
    
    //设置低电平时间
    temp_low_num--;
    CMT_CMD1 = temp_low_num >> 8;
    CMT_CMD2 = (uint8)temp_low_num;
    
    //设置高电平时间
    CMT_CMD3 = temp_high_num >> 8;
    CMT_CMD4 = (uint8)temp_high_num;
    
    //设置模式且使能CMT模块    
    CMT_MSC = 0x8u | 0x1u;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      CMT_PWM占空比设置
//  @param      duty         设置PWM的占空比
//  @return     void
//  @since      v1.0
//  Sample usage:               cmt_pwm_duty(50);    //占空比为百分之（50/CMT_PRECISON *100）;
//-------------------------------------------------------------------------------------------------------------------
void cmt_pwm_duty(uint16 duty)
{
    
    uint32 temp_high_num, temp_low_num;

    //计算高低电平的计数次数
    temp_low_num = (cmt_period*(CMT_PRECISON-duty)/CMT_PRECISON);
    temp_high_num = (cmt_period*(duty)/CMT_PRECISON);
    
    //设置低电平时间
    temp_low_num--;
    CMT_CMD1 = temp_low_num >> 8;
    CMT_CMD2 = (uint8)temp_low_num;
    
    //设置高电平时间
    CMT_CMD3 = temp_high_num >> 8;
    CMT_CMD4 = (uint8)temp_high_num;

}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      CMT_PWM频率设置
//  @param      freq         设置PWM的频率
//  @param      duty         设置PWM的占空比
//  @return     void
//  @since      v1.0
//  Sample usage:               cmt_pwm_init(50,50);    // 频率50HZ，占空比为百分之（50/CMT_PRECISON *100）;
//-------------------------------------------------------------------------------------------------------------------
void cmt_pwm_freq(uint16 freq, uint16 duty)
{
    uint32 temp_clk;
    uint32 temp_high_num, temp_low_num;
    uint32 temp_div;

    
    //由于CMT模块会固定八分频，因此这里计算，便于后面使用
    temp_clk = bus_clk_khz*1000/8;
    
    //计算最佳分频
    temp_div = temp_clk/freq;
    temp_div = temp_div>>16;
    if(temp_div>0x0f)   temp_div = 0x0f;
    
    //设置分频
    CMT_PPS = CMT_PPS_PPSDIV(temp_div);

    
    //计算一个周期需要计数的次数
    cmt_period = temp_clk/(temp_div+1)/freq;
    
    //计算高低电平的计数次数
    temp_low_num = (cmt_period*(CMT_PRECISON-duty)/CMT_PRECISON);
    temp_high_num = (cmt_period*(duty)/CMT_PRECISON);
    
    //设置低电平时间
    temp_low_num--;
    CMT_CMD1 = temp_low_num >> 8;
    CMT_CMD2 = (uint8)temp_low_num;
    
    //设置高电平时间
    CMT_CMD3 = temp_high_num >> 8;
    CMT_CMD4 = (uint8)temp_high_num;
}