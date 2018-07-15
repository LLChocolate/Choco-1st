/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名       ：fire_drivers_cfg.h
 * 描述         ：K60复用管家配置
 * 备注         ：野火耗了很久时间整理出来，希望尊重野火的劳动成功，注明野火原创！！
 *
 * 实验平台     ：野火kinetis开发板
 * 库版本       ：
 * 嵌入系统     ：
 *
 * 作者         ：野火嵌入式开发工作室
 * 淘宝店       ：http://firestm32.taobao.com
 * 技术支持论坛 ：http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/

#ifndef _FIRE_DRIVERS_CFG_H_
#define _FIRE_DRIVERS_CFG_H_

#include "gpio_cfg.h"
#include "define.h"


/**********************************  UART   ***************************************/

//      模块通道    端口          可选范围                          建议
#define UART0_RX    PTB16        //PTA1、PTA15、PTB16、PTD6          PTA1不要用（与Jtag冲突）
#define UART0_TX    PTB17        //PTA2、PTA14、PTB17、PTD7          PTA2不要用（与Jtag冲突）

#define UART1_RX    PTC3        //PTC3、PTE1
#define UART1_TX    PTC4        //PTC4、PTE0

#define UART2_RX    PTD2        //PTD2
#define UART2_TX    PTD3        //PTD3

#define UART3_RX    PTE5       //PTB10、PTC16、PTE5
#define UART3_TX    PTE4       //PTB11、PTC17、PTE4

#define UART4_RX    PTE25       //PTC14、PTE25
#define UART4_TX    PTE24       //PTC15、PTE24

#define UART5_RX    PTD8        //PTD8、PTE9
#define UART5_TX    PTD9        //PTD9、PTE8

/**********************************  FTM    ***************************************/

//      模块通道    端口          可选范围              建议
#define FTM0_CH0    PTC1        //PTC1、PTA3            PTA3不要用（与Jtag、SWD冲突）
#define FTM0_CH1    PTA4        //PTC2、PTA4
#define FTM0_CH2    PTA5        //PTC3、PTA5
#define FTM0_CH3    PTA6        //PTC4、PTA6            电机2

#define FTM0_CH4    PTA7        //PTD4、PTA7            电机1（用户自行添加）


#define FTM0_CH5    PTD5        //PTD5、PTA0            PTA0不要用（与Jtag、SWD冲突）
#define FTM0_CH6    PTD6        //PTD6、PTA1            PTA1不要用（与Jtag冲突）
#define FTM0_CH7    PTD7        //PTD7、PTA2            PTA2不要用（与Jtag冲突）


//      模块通道    端口          可选范围              建议
#define FTM1_CH0    PTA8        //PTA8、PTA12、PTB0
#define FTM1_CH1    PTA9        //PTA9、PTA13、PTB1   右电机测速
/*
#define FTM1_CH2                //空
#define FTM1_CH3                //空
#define FTM1_CH4                //空
#define FTM1_CH5                //空
#define FTM1_CH6                //空
#define FTM1_CH7                //空
*/

//      模块通道    端口          可选范围              建议
#define FTM2_CH0    PTA10       //PTA10、PTB18          左电机测速
#define FTM2_CH1    PTA11       //PTA11、PTB19

/*
#define FTM2_CH2                //空
#define FTM2_CH3                //空
#define FTM2_CH4                //空
#define FTM2_CH5                //空
#define FTM2_CH6                //空
#define FTM2_CH7                //空
*/


/**********************************  I2C   ***************************************/

//      模块通道    端口          可选范围              建议
#define I2C0_SCL    PTB0        // PTB0、PTB2、PTD8
#define I2C0_SDA    PTB1        // PTB1、PTB3、PTD9

#define I2C1_SCL    PTE1       // PTE1、PTC10
#define I2C1_SDA    PTE0       // PTE0、PTC11

/**********************************  SPI   ***************************************/
//PCS接口，不用的时候需要注释，就不会进行初始化对应的管脚
//      模块通道    端口          可选范围                  建议

#define SPI0_SCK    PTC5       // PTA15、PTC5、PTD1        全部都是 ALT2
#define SPI0_SOUT   PTC6       // PTA16、PTC6、PTD2        全部都是 ALT2
#define SPI0_SIN    PTC7       // PTA17、PTC7、PTD3        全部都是 ALT2

#define SPI0_PCS0   PTA14       // PTA14、PTC4、PTD0、      全部都是 ALT2
#define SPI0_PCS1   PTD4        // PTC3、PTD4               全部都是 ALT2
#define SPI0_PCS2   PTC2        // PTC2、PTD5               全部都是 ALT2
#define SPI0_PCS3   PTD6        // PTC1、PTD6               全部都是 ALT2
#define SPI0_PCS4   PTC0        // PTC0、                   全部都是 ALT2
#define SPI0_PCS5   PTB23       // PTB23                    ALT3


#define SPI1_SCK    PTB11       // PTE2、PTB11、            全部都是 ALT2
#define SPI1_SOUT   PTB16       // PTE1、PTB16、            全部都是 ALT2
#define SPI1_SIN    PTB17       // PTE3、PTB17、            全部都是 ALT2

#define SPI1_PCS0   PTB10        // PTE4、PTB10、            全部都是 ALT2
#define SPI1_PCS1   PTE0        // PTE0、PTB9、             全部都是 ALT2
#define SPI1_PCS2   PTE5        // PTE5、                   全部都是 ALT2
#define SPI1_PCS3   PTE6        // PTE6、                   全部都是 ALT2


#define SPI2_SCK    PTB21       // PTB21、PTD12             全部都是 ALT2
#define SPI2_SOUT   PTB22       // PTB22、PTD13             全部都是 ALT2
#define SPI2_SIN    PTB23       // PTB23、PTD14             全部都是 ALT2
#define SPI2_PCS0   PTB20       // PTB20、PTD11             全部都是 ALT2
#define SPI2_PCS1   PTD15       // PTD15                    全部都是 ALT2



#endif  //_FIRE_DRIVERS_CFG_H_


