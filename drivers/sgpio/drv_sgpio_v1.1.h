/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_sgpio_v1.1.h
 * @author yangzhl3 (yangzhl3@starsmicrosystem.com)
 * @date 2025/03/17
 * @brief
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/03/17   yangzhl3        移植自产品软件
 *
 *
 */
#ifndef __DRV_SGPIO_V1P1_H__
#define __DRV_SGPIO_V1P1_H__

/**
 * Macro
 */

#define SGPIO_PORT0_BASEADDR    (0x58500000)
#define SGPIO_PORT1_BASEADDR    (0x58501000)
#define SGPIO_SYS_BASE          (0x2F000000) ///< fpga上没用，流片后需要修改
#define SGPIO1_PIN_MUX_ADDR1    (0x58601110)
#define SGPIO1_PIN_MUX_ADDR2    (0x58601120)

#define SGPIO1_PIN_MUX_VAL1     (0xa0000000)
#define SGPIO1_PIN_MUX_VAL2     (0x0000000a)

#define SGPIO_PORT0_CLK_BIT     (16)
#define SGPIO_PORT1_CLK_BIT     (17)

#define SGPIO_SYS_RESET_OFFSET  (0x14)

#ifdef PS3_MODEL_V300
#define SGPIO_8485_BASE         (0x5820a004) ///< 8485协议
#define SGPIO_8485_BIT          (0x00000003)
#define SGPIO_SYS_RESET_CCU_OFFSET  (0x78)
#define CCU_RESET_VAL (0x0fff0c82)
#define SGPIO_VALUE_0           (0x0)
#define SGPIO_BLINK_HB_PCLK_VALUE           (400000000)
#else
#define SGPIO_8485_BASE         (0x582090b8) ///< 8485协议
#define SGPIO_8485_BIT          (0x00000007)
#endif
#define SGPIO_VALUE_1           (0x1)
#define SGPIO_VALUE_2           (0x2)
#define SGPIO_VALUE_0X30        (0x30)

#define SGPIO_GPIO_CLKOUT_BITS  (6)
#define SGPIO_GPIO_DOUT_BITS    (8)
#define SGPIO_GPIO_LOADOUT_BITS (10)
#define SGPIO_GPIO_DIN_BITS     (12)
#define SGPIO_PIN_MUX_VAL       (2)
#define SGPIO_PIN_MUX_MASK      (3)
#define SGPIO_SYS_CLK_OFFSET    (0x3c)
#define SGPIO_GPIO_MODE1_OFFSET (0x1D0)
#define SGPIO_REG_GROUP_MAX     (12)
#define SGPIO_PHY_NUM_MAX       (48)
#define SGPIO_PHY_PER_REG       (4)
#define SGPIO_BITS_PER_PHY      (3)
#define SGPIO_BITS_PER_REG      (32)
#define SGPIO_VENDER_SPEC_REGS  (2)

#define SGPIO_FILED(v, f) ((v) << (f))

#define SGPIO_CLK_HZ_MIN           (32)     ///< 32HZ
#define SGPIO_CLK_HZ_MAX           (100000) ///< 10KHZ

/**
 * 寄存器offset Macro
 */
#define SGPIO_CFG_REG_OFFSET(n)      (0x0000 + (n)*4)
#define SGPIO_SPEC_REG_OFFSET(n)     (0x0500 + (n)*4)
#define SGPIO_BITORDER_SEL_OFFSET(n) (0x0600 + (n)*4)
#define SGPIO_TRANS_REG_OFFSET(n)    (0x0200 + (n)*4)
#define SGPIO_GP_TRANS_OFFSET(n)     (0x0400 + (n)*4)
#define SGPIO_RECV_REG_OFFSET(n)     (0x0100 + (n)*4)
#define SGPIO_GP_RECV_OFFSET(n)      (0x0300 + (n)*4)
#define SGPIO_SIO_OFFSET(n)          (0x0800 + (n)*4)
#define SGPIO_DRIVE_STROBE(n)        (0x0640 + (n)*4) ///< activity io信号选通

/**
 * SGPIO CFG0,
 */
typedef struct SgpioCfg0 {
    U32 rsvd1 : 8;         ///< bits   0-7
    U32 version : 4;       ///< bits   8-11    版本
    U32 rsvd2 : 4;         ///< bits   12-15
    U32 GPRegCnt : 4;      ///< bits   16-19   通用目的数据寄存器个数
    U32 cfgRegCnt : 3;     ///< bits   20-22   配置寄存器个数
    U32 gpioEn : 1;        ///< bit    23      gpio使能
    U32 supportDrvCnt : 8; ///< bits   24-31   支持驱动个数
} SgpioCfg0_s;

/**
 * SGPIO CFG1,
 */
typedef struct SgpioCfg1 {
    U32 rsvd1 : 8;        ///< bits   0-7
    U32 blinkRateA : 4;   ///< bits   8-11    闪烁发生器A频率
    U32 blinkRateB : 4;   ///< bits   12-15   闪烁发生器B频率
    U32 activeOnMax : 4;  ///< bits   16-19   active on最大时间
    U32 activeOff : 4;    ///< bits   20-23   active off
    U32 activeOnStr : 4;  ///< bit    24-27   连续输出1
    U32 activeOffStr : 4; ///< bits   28-31   连续输出0
} SgpioCfg1_s;

/**
 * SGPIO receive reg, 总共48通道，每个寄存器配置4通道
 */
typedef struct SgpioReceive {
    U32 inData3 : 3; ///< bits   0-2     第4n-1输入
    U32 rsvd1 : 5;   ///< bits   3-7
    U32 inData2 : 3; ///< bits   8-10     第4n-2输入
    U32 rsvd2 : 5;   ///< bits   11-15
    U32 inData1 : 3; ///< bits   16-18     第4n-3输入
    U32 rsvd3 : 5;   ///< bits   19-23
    U32 inData0 : 3; ///< bits   24-26     第4n输入
    U32 rsvd4 : 5;   ///< bits   27-31
} SgpioReceive_s;

/**
 * SGPIO transmit reg, 总共48通道，每个寄存器配置4通道
 */
typedef struct SgpioTransmit {
    U32 error0 : 3;    ///< bits   0-2     第3输入
    U32 local0 : 2;    ///< bits   3-4
    U32 activity0 : 3; ///< bits   5-7
    U32 error1 : 3;    ///< bits   8-10     第2输入
    U32 local1 : 2;    ///< bits   11-12
    U32 activity1 : 3; ///< bits   13-15
    U32 error2 : 3;    ///< bits   16-18     第1输入
    U32 local2 : 2;    ///< bits   19-20
    U32 activity2 : 3; ///< bits   21-23
    U32 error3 : 3;    ///< bits   24-26     第0输入
    U32 local3 : 2;    ///< bits   27-28
    U32 activity3 : 3; ///< bits   29-31
} SgpioTransmit_s;

/**
 * SGPIO general purpose receive reg,
 */
typedef struct SgpioGPRecvCfg {
    U32 rsvd1 : 16;       ///< bits   0-15
    U32 count : 8;        ///< bits   16-23     剩余重复次数
    U32 sloadPattern : 4; ///< bits   24-27     4bit模式
    U32 rsvd2 : 4;        ///< bits   28-31
} SgpioGPRecvCfg_s;

/**
 * SGPIO general purpose receive reg,
 */
typedef struct SgpioGPRecv {
    U32 recvData; ///< bits   注意字节的小端，一字节一字节为单位
} SgpioGPRecv_s;

/**
 * SGPIO general purpose transmit reg,
 */
typedef struct SgpioGPTranCfg {
    U32 rsvd1 : 16; ///< bits   0-15
    U32 count : 8;  ///< bits   16-23     剩余重复次数
    U32 sloadPattern : 4;
    U32 rsvd2 : 4; ///< bits   24-31
} SgpioGPTranCfg_s;

/**
 * SGPIO general purpose transmit reg,
 */
typedef struct SgpioGPTran {
    U32 recvData; ///< bits   注意字节的小端，一字节一字节为单位
} SgpioGPTran_s;

/**
 * SGPIO Vender Spectifi reg,
 */
typedef struct SgpioVenderSpec0 {
    U32 firstSIO : 1;       ///< bit    0 1- 指导bitstream开始
    U32 SIOMode : 1;        ///< bit    1 0- Single 1-Multi
    U32 rsvd1 : 1;          ///< bit    2
    U32 SIOClkEn : 1;       ///< bit    3 1 -output 0 Input 不支持
    U32 SIOCtrlOut : 1;     ///< bit    4
    U32 SIOClkFilterEn : 1; ///< bit    5 1-时钟fliter使能
    U32 SIOCtrlOden : 1;    ///< bit    6 1-
#ifdef PS3_PRODUCT_EXPANDER
    U32 clkDivide : 20;     ///< bits   7-26 时钟分频
    U32 rsvd2 : 3;          ///< bits   27-29
#else
    U32 clkDivide : 21;     ///< bits   7-27 时钟分频
    U32 rsvd2 : 2;          ///< bits   28-29
#endif
    U32 SIOClkoutEn : 1;    ///< bit    30 1- SIOClkout 可以输出
    U32 resetFsm : 1;       ///< bit    31 1 复位所有状态机
} SgpioVenderSpec0_s;

/**
 * SGPIO Vender Spectifi reg,
 */
typedef struct SgpioVenderSpec1 {
    U32 devCount : 7;     ///< bits   0 6- 设备数，和sdrive count一致
    U32 rsvd1 : 1;        ///< bit    7
    U32 gpioModeCtrl : 1; ///< bit    8 1 -8485 0 vender
    U32 sloadMode : 1;    ///< bit    9 0 -当前生效，1 -下一个生效
    U32 GPModeCtrl : 1;   ///< bit    10 0 -sload奇数，进入GP模式，否则Normal 1-非0则进入GP模式
    U32 dinLatch : 1;     ///< bit    11 1- 连续且一样sload可以更新，0-任何sload都可以更新
#ifdef PS3_PRODUCT_EXPANDER
    U32 rsvd2 : 20;       ///< bits   12-31
#else
    U32 sdoutpolsel : 1;     ///< bit    12 1- 更改sdout输出信号的极性
    U32 sloadpolsel : 1;     ///< bit    13 1- 更改sload输出信号的极性
    U32 rsvd2 : 18;       ///< bits   12-31
#endif
} SgpioVenderSpec1_s;

/**
 * SGPIO Sio Blink Sel reg,
 */
typedef struct SgpioSioBlinkSel{
    U32 sioBlinkOutSel : 3;     ///< bits   0 2 选择从哪个sio输出blink信号0~5
    U32 sioBlinkOutOne : 1;     ///< bit    3 blink out 是否使能，0 使能，1禁止
    U32 masterBlinkSel : 1;     ///< bit    4 0: 接受外部blink信号，1: 作为master
    U32 sioBlinkMask   : 1;     ///< bit    5 0: 打开sio blink功能，1: 禁止blink功能
    U32 rsvd           : 26;    ///< bits   6-31
} SgpioSioBlinkSel_s;

typedef union SgpioReg {
    volatile U32 all;
    volatile SgpioCfg0_s sgpioConfig0;
    volatile SgpioCfg1_s sgpioConfig1;
    volatile SgpioReceive_s sgpioReceive;
    volatile SgpioTransmit_s sgpioTransmit;
    volatile SgpioGPRecvCfg_s sgpioGPRecvCfg;
    volatile SgpioGPRecv_s sgpioGPRecv;
    volatile SgpioGPTranCfg_s sgpioGPTranCfg;
    volatile SgpioGPTran_s sgpioGPTran;
    volatile SgpioVenderSpec0_s sgpioVspec0;
    volatile SgpioVenderSpec1_s sgpioVspec1;
    volatile SgpioSioBlinkSel_s sgpioSioBlinkSel;
} SgpioReg_u;

typedef struct SgpioCtrl {
    U32 base;
    U32 clk;
    U32 phyNum;
} SgpioCtrl_s;

typedef enum {
    SGPIO_RET_OK  = 0,
    SGPIO_RET_ERR = 1,
} SgpioRet_e;

#endif
