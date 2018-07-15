/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名       ：lptmr.c
 * 描述         ：lptmr定功耗定时器，目前只有一个定时器延时函数
 * 备注         ：官方例程
 *
 * 实验平台     ：野火kinetis开发板
 * 库版本       ：
 * 嵌入系统     ：
 *
 * 作者         ：飞思卡尔公司
 * 淘宝店       ：http://firestm32.taobao.com
 * 技术支持论坛 ：http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/

#include "common.h"
#include "lptmr.h"

#define LPTMR_irq     85u


/*!
 *  @brief      LPTMR脉冲计数初始化
 *  @param      LPT0_ALTn   LPTMR脉冲计数管脚
 *  @param      count       LPTMR脉冲比较值
 *  @param      LPT_CFG     LPTMR脉冲计数方式：上升沿计数或下降沿计数
 *  @since      v5.0
 *  Sample usage:       lptmr_pulse_init(LPT0_ALT1,0xFFFF,LPT_Rising);     // LPTMR 脉冲捕捉，捕捉0xFFFF后触发中断请求（需要开中断才执行中断复位函数），上升沿捕捉
 */
void lptmr_pulse_init(LPT0_ALTn altn, u16 count, u8 PrescaleValue, LPT_CFG cfg)
{
    if(PrescaleValue > 0x0f)PrescaleValue = 0x0f;
    if(altn == LPT0_ALT1)
    {
        SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;  //打开 PORTA 时钟
        PORTA_PCR19 = PORT_PCR_MUX(0x6);    //在PTA19上使用 ALT6
    }
    else if(altn == LPT0_ALT2)
    {
        SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;  //使能 PORTC 时钟
        PORTC_PCR5 = PORT_PCR_MUX(0x4);     //在PTC5上使用 ALT4
    }
    else                                    //不可能发生事件
    {
        assert_failed(__FILE__, __LINE__);  //设置管脚有误？
    }
    /* 开启模块时钟 */
    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;    //使能LPT模块时钟
    /* 清状态寄存器 */
    LPTMR0_CSR = 0x00;                      //先关了LPT，这样才能设置时钟分频等
    /* 设置累加计数值  */
    LPTMR_CMR_REG(LPTMR0_BASE_PTR)  =   LPTMR_CMR_COMPARE(count);                       //设置比较值
    /* 时钟选择 */
    LPTMR_PSR_REG(LPTMR0_BASE_PTR)  =   LPTMR_PSR_PCS(0x1) | LPTMR_PSR_PBYP_MASK  |  LPTMR_PSR_PRESCALE(PrescaleValue);        //使用 LPO clock 且 bypass glitch filter
    //                                                        开启和配置脉冲滤波器：2^n个时钟上升沿才识别
    /* 管脚设置、使能中断  */
    LPTMR_CSR_REG(LPTMR0_BASE_PTR)  =   LPTMR_CSR_TPS(altn) | LPTMR_CSR_TMS_MASK  | ( cfg == LPT_Falling ?  LPTMR_CSR_TPP_MASK :   0  )  | LPTMR_CSR_TEN_MASK  | LPTMR_CSR_TIE_MASK  ;
    //                                       选择输入管脚        选择脉冲计数                                   下降沿       上升沿           使能LPT
    // TFC = 0，即计数值等于比较值时，计数值复位
    enable_irq(LPTMR_irq);	                //开引脚的IRQ中断
}

/*!
 *  @brief      获取LPTMR脉冲计数值
 *  @return     脉冲计数值
 *  @since      v5.0
 *  Sample usage:       uint16 data = lptmr_pulse_get();  //获取脉冲计数值
 */
uint16 lptmr_pulse_get(void)
{
    uint16 data;
    if(LPTMR0_CSR & LPTMR_CSR_TCF_MASK)     //已经溢出了
    {
        data = ~0;                          //返回 0xffffffff 表示错误
    }
    else
    {
        data = LPTMR0_CNR;
    }
    return data;
}


/*!
 *  @brief      清空LPTMR脉冲计数
 *  @since      v5.0
 *  Sample usage:       lptmr_counter_clean();          //清空LPTMR脉冲计数
 */
void lptmr_pulse_clean(void)
{
    LPTMR_CSR_REG(LPTMR0_BASE_PTR)  &= ~LPTMR_CSR_TEN_MASK;     //禁用LPT的时候就会自动清计数器的值
    LPTMR_CSR_REG(LPTMR0_BASE_PTR)  |=LPTMR_CSR_TEN_MASK;
}

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：time_delay_ms
*  功能说明：延时函数，使用定功耗定时器延时，准确
*  参数说明：ms   延时时间，单位为ms
*  函数返回：无
*  修改时间：2012-1-20
*  备    注：官方例程自带
*************************************************************************/
void time_delay_ms(uint32 ms)
{
    /* Make sure the clock to the LPTMR is enabled */
    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;

    /* Set the compare value to the number of ms to delay */
    LPTMR0_CMR = ms;

    /* Set up LPTMR to use 1kHz LPO with no prescaler as its clock source */
    LPTMR0_PSR = LPTMR_PSR_PCS(1) | LPTMR_PSR_PBYP_MASK;

    /* Start the timer */
    LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;

    /* Wait for counter to reach compare value */
    while (!(LPTMR0_CSR & LPTMR_CSR_TCF_MASK));

    /* Clear Timer Compare Flag */
    LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;

    return;
}



