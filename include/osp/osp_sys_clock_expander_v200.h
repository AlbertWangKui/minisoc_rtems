/**
 * Copyright (C), 2021, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_sys_clock_expander_cs.h
 * @author  tianye
 * @date    2022.12.28
 * @brief   NA
 * @note    NA
 */
#ifndef __OSP_SYS_CLOCK_EXPANDER_V200_H__
#define __OSP_SYS_CLOCK_EXPANDER_V200_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef enum OspModuleId {
    // offset 0x90(clk),  0x10(rst) - 0
    MODULE_EFUSE = 1,
    MODULE_PINMUX = 5,
    MODULE_PVT = 7,
    MODULE_LED = 17,
    MODULE_SGPIO_0 = 18,
    MODULE_SGPIO_1 = 19,
    MODULE_SPINUP = 25,
    MODULE_ECM = 26,
    MODULE_ECR = 27,
    MODULE_BPP = 28,
    MODULE_EDFB = 29,

    // offset 0x94(clk)，0x14(rst)- 32
    MODULE_XLINK_0 = 32,
    MODULE_XLINK_1,
    MODULE_XLINK_2,
    MODULE_XLINK_3,
    MODULE_XLINK_4,
    MODULE_XLINK_5,
    MODULE_XLINK_6,
    MODULE_XLINK_7,
    MODULE_XLINK_8,
    MODULE_XLINK_9,
    MODULE_XLINK_10,
    MODULE_XLINK_11,
    MODULE_XLINK_12,
    MODULE_XLINK_13,
    MODULE_XLINK_14,
    MODULE_XLINK_15,
    MODULE_XLINK_16,
    MODULE_XLINK_17,
    MODULE_XLINK_18,
    MODULE_XLINK_19,
    MODULE_XLINK_20,
    MODULE_XLINK_21,
    MODULE_XLINK_22,
    MODULE_XLINK_23,
    MODULE_XLINK_24,
    MODULE_XLINK_25,
    MODULE_XLINK_26,
    MODULE_XLINK_27,
    MODULE_XLINK_28,
    MODULE_XLINK_29,
    MODULE_XLINK_30,
    MODULE_XLINK_31 = 63,

    // 0x98(clk)，0x18(rst)-64
    MODULE_XLINK_32 = 64,
    MODULE_XLINK_33,
    MODULE_XLINK_34,
    MODULE_XLINK_35,
    MODULE_XLINK_36,
    MODULE_XLINK_37,
    MODULE_XLINK_38,
    MODULE_XLINK_39,
    MODULE_XLINK_40,
    MODULE_XLINK_41,
    MODULE_XLINK_42,
    MODULE_XLINK_43,
    MODULE_XLINK_44,
    MODULE_XLINK_45,
    MODULE_XLINK_46,
    MODULE_XLINK_47 = 79,
    MODULE_VLINK_XL0 = 80,
    MODULE_VLINK_XL1 = 81,
    MODULE_VLINK_XL2 = 82,
    MODULE_VLINK_SXP0 = 83,
    MODULE_VLINK_SXP1 = 84,
    MODULE_VLINK_SXP2 = 85,
    MODULE_X16_0_LANE_0_15 = 88,
    MODULE_X4_0_LANE_16_19 = 89,
    MODULE_X4_1_LANE_20_23 = 90,
    MODULE_X4_2_LANE_24_27 = 91,
    MODULE_X4_3_LANE_28_31 = 92,
    MODULE_X16_1_LANE_32_47 = 93,

    // 0x9C(clk)，0x1c(rst)-96
    MODULE_X4_BUS_0 = 96,
    MODULE_X4_BUS_1,
    MODULE_X4_BUS_2,
    MODULE_X4_BUS_3,
    MODULE_X4_BUS_4,
    MODULE_X4_BUS_5,
    MODULE_X4_BUS_6,
    MODULE_X4_BUS_7,
    MODULE_X4_BUS_8,
    MODULE_X4_BUS_9,
    MODULE_X4_BUS_10,
    MODULE_X4_BUS_11 = 107,
    MODULE_VLINK_X3_BUS = 112,
    MODULE_X16_0_BUS = 120,
    MODULE_X4_0_BUS,
    MODULE_X4_1_BUS,
    MODULE_X4_2_BUS,
    MODULE_X4_3_BUS,
    MODULE_X16_1_BUS = 125,

    // miniSoc baseAddr = 0x49010000; offset = 0x1054(clk)-128,
    MODULE_CLK_TACH_1 = 129,
    MODULE_CLK_TACH_0 = 132,
    MODULE_CLK_PWM_0 = 133,
    MODULE_CLK_PWM_1 = 134,
    MODULE_CLK_TIMER_0 = 135,
    MODULE_CLK_TIMER_1 = 136,
    MODULE_CLK_TIMER_2 = 137,
    MODULE_CLK_UART = 138,
    MODULE_CLK_WDG = 139,
    MODULE_CLK_IIC_0 = 140,
    MODULE_CLK_IIC_1,
    MODULE_CLK_IIC_2,
    MODULE_CLK_IIC_3,
    MODULE_CLK_IIC_4,
    MODULE_CLK_IIC_5,
    MODULE_CLK_IIC_6,
    MODULE_CLK_IIC_7,
    MODULE_CLK_IIC_8,
    MODULE_CLK_IIC_9,
    MODULE_CLK_IIC_10,
    MODULE_CLK_IIC_11,
    MODULE_CLK_IIC_12,
    MODULE_CLK_IIC_13 = 153,
    MODULE_CLK_TACH_2 = 154,
    MODULE_CLK_TACH_3 = 155,
    MODULE_CLK_QSPI = 156,  
    MODULE_CLK_SM2 = 157,
    MODULE_CLK_SM3 = 158,

    // 0x1060 (rst)-160
    MODULE_RST_IIC_0 = 162,
    MODULE_RST_IIC_1 = 163,
    MODULE_RST_IIC_2 = 164,
    MODULE_RST_IIC_3 = 165,
    MODULE_RST_IIC_4 = 166,
    MODULE_RST_IIC_5 = 167,
    MODULE_RST_IIC_6 = 168,
    MODULE_RST_IIC_7 = 169,
    MODULE_RST_IIC_8 = 170,
    MODULE_RST_IIC_9 = 171,
    MODULE_RST_IIC_10 = 172,
    MODULE_RST_IIC_11 = 173,
    MODULE_RST_IIC_12 = 174,
    MODULE_RST_IIC_13 = 175,
    MODULE_RST_PWM_0 = 176,
    MODULE_RST_PWM_1 = 177,
    MODULE_RST_TIMER_0 = 178,
    MODULE_RST_TIMER_1 = 179,
    MODULE_RST_TIMER_2 = 180,
    MODULE_RST_UART = 182,
    MODULE_RST_WDG = 183,
    MODULE_RST_TACH_0 = 184,
    MODULE_RST_TACH_1 = 185,
    MODULE_RST_TACH_2 = 188,
    MODULE_RST_TACH_3 = 189,

    // offset 0x1064, -160
    MODULE_RST_QSPI = 201,
    MODULE_RST_SM2 = 203,
    MODULE_RST_SM3 = 204,
    MODULE_RST_TRNG = 205,

    MODULE_RST_MAX
} OspModuleId_e;

typedef enum OspPeripheralId {
    // clk_config_apb
    PERIP_LED,
    PERIP_SGPIO,
    PERIP_SPINUP,

    // clk_expander
    PERIP_ECM,
    PERIP_ECR,
    PERIP_BPP,
    PERIP_EDFB,
    PERIP_XL,
    PERIP_STP,

    // clk_vlink
    PERIP_VLINK,

    // clk_pcs
    PERIP_PCS,

    // clk_peri_apb_div
    PERIP_TIMER,
    PERIP_IIC,
    PERIP_UART0,
    PERIP_PWM,
    PERIP_WDG,
    PERIP_TACH,

    // clk_soc_apb_div
    PERIP_TRNG,

    // qspi_div
    PERIP_QSPI,

    // sec_div
    PERIP_SM2,
    PERIP_SM3,

    // Ref_clk
    PERIP_EFUSE,
    PERIP_UART1,
    PERIP_RESERVERD,

} OspPeripheralId_e;

/**                                                
 ** @brief   获取指定IP的频率                       
 ** @param   IP的ID                                 
 ** @return  时钟频率，单位是hz                     
 ** warning NA                                     
 ** @note    NA                                     
 */                                                
uint32_t ospGetPeripClk(OspPeripheralId_e peripheralId);

/**
 * @brief   复位指定模块
 * @param   模块id
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospHoldReset(OspModuleId_e module);

/**
 * @brief   解复位指定模块
 * @param   模块id
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospReleaseReset(OspModuleId_e module);

/**
 * @brief   使能指定模块时钟
 * @param   模块id
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospClockEnable(OspModuleId_e module);

/**
 * @brief   关闭指定模块时钟
 * @param   模块id
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospClockDisable(OspModuleId_e module);

/**
 * @brief   获取系统频率
 * @param   none
 * @return  HZ
 * @warning NA
 * @note    NA
 */
uint32_t ospGetSysClock(void);
#ifdef __cplusplus
}
#endif

#endif
