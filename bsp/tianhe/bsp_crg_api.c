/**
 * copyright (C), 2022, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_crg_api.c
 * @author shaogl@starsmicrosystem.com
 * @date 2024/06/30
 * @brief crg api for special bsp.
 */

#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "pll.h"
#include "bsp_topcrg.h"
#include "log_msg.h"
#include "kconfig.h"

#ifdef TIANHE_BSP_SELFTEST_ENABLE
#include "tianhe_bsp_selftest.h"
#endif

#define SRST_REQ_REG_OFST (0xA090)
#define TOP_CRG_SRST_REQ_REG (SYS_TOP_CRG_BASE_ADRS + SRST_REQ_REG_OFST)
#define SRST_LOCK_REG_OFST (0xA300)
#define TOP_CRG_SRST_LOCK_REG (SYS_TOP_CRG_BASE_ADRS + SRST_LOCK_REG_OFST)
#define SRST_LOCK_MAGIC (0xACCE55FF)

#define PERIPS_CLK_FREQ_HZ (250000000)
#define QSPI_CLK_FREQ_HZ (750000000)
#define DMA_CLK_FREQ_HZ (750000000)
#define SGPIO_CLK_FREQ_HZ (200000000)

#define PERIPS_CLK_ENABLE_UNSUPPORT (0)

typedef struct DevID2CrgModule {
    DevList_e startDevID;
    DevList_e endDevID;
    U32 clkRegAddr;
    U32 clkBitsNr;
    U32 resetRegAddr;
    U32 resetBitsNr;
} DevID2CrgModule_s;

static const DevID2CrgModule_s gBspDevID2CrgModule[] = {
    {DEVICE_SMBUS0, DEVICE_SMBUS6, 0xbe600008, 0, 0xbe60000c, 0},  
    {DEVICE_UART0, DEVICE_UART1, 0xbe600010, 0, 0xbe600014, 0},        
    {DEVICE_TACH0, DEVICE_TACH3, 0xbe600018, 0, 0xbe60001c, 0},
    {DEVICE_PWM0, DEVICE_PWM0, 0xbe600020, 0, 0xbe600024, 0},
    {DEVICE_TIMER0, DEVICE_TIMER11, 0xbe600028, 0, 0xbe60002c, 0},       
    {DEVICE_WDT0, DEVICE_WDT3, 0xbe600030, 0, 0xbe600034, 0}, 
    {DEVICE_I3C0, DEVICE_I3C1, 0xbe600050, 0, 0xbe600054, 0}, 
 
    {DEVICE_QSPI0, DEVICE_QSPI0, 0xbe10002c, 24, 0xbe100040, 24},
    {DEVICE_QSPI1, DEVICE_QSPI1, 0xbe10002c, 23, 0xbe100040, 23},
    {DEVICE_DMA_0, DEVICE_DMA_0,  0xbe10002c, 14, 0xbe100040, 14},
    {DEVICE_TRNG0, DEVICE_TRNG0, 0, 0, 0xbe100040, 19}, ///< trng，不支持时钟矩阵门控
    {DEVICE_SM2_0, DEVICE_SM2_0, 0, 0, 0xbe100040, 16}, ///< sm2，不支持时钟矩阵门控
    {DEVICE_SM3_0, DEVICE_SM3_0, 0, 0, 0xbe100040, 17}, ///< sm3，不支持时钟矩阵门控
    {DEVICE_SM4_0, DEVICE_SM4_0, 0, 0, 0xbe100040, 18}, ///< sm4，不支持时钟矩阵门控

    ///< GPIO不提供单独复位和使能，硬件把两个gpio合并在一起了，直接在boardInit中实现；

    {DEVICE_PVT, DEVICE_PVT, 0, 0, 0xb800A05C, 7}, ///< pvt,不支持时钟矩阵门控
    {DEVICE_SGPIO0, DEVICE_SGPIO0, 0, 0, 0xb800A064, 0}, ///< sgpio0,不支持时钟矩阵门控
    {DEVICE_SGPIO1, DEVICE_SGPIO1, 0, 0, 0xb800A068, 0}, ///< sgpio1,不支持时钟矩阵门控
    {DEVICE_SGPIO2, DEVICE_SGPIO2, 0, 0, 0xb800A06C, 0}, ///< sgpi02,不支持时钟矩阵门控
    {DEVICE_SGPIO3, DEVICE_SGPIO3, 0, 0, 0xb800A070, 0}, ///< sgpio3,不支持时钟矩阵门控

};

static S32 findPeripsByDevID(DevList_e devID, U32 *peripsID, U32 *item)
{
    S32 ret = -EXIT_FAILURE;
    U32 i;

    if ((NULL == item) || (NULL == peripsID)) {
        goto exit;
    }
    
    for (i = 0; i < ARRAY_SIZE(gBspDevID2CrgModule); i++) {
        if ((devID >= gBspDevID2CrgModule[i].startDevID) && (devID <= gBspDevID2CrgModule[i].endDevID)) {
            // *perips = gBspDevID2CrgModule[i].perips;
            *peripsID = devID - gBspDevID2CrgModule[i].startDevID;
            *item = i;
            ret = EXIT_SUCCESS;
            break;
        }
    }

exit:
    return ret;
}

void softSysReset(void)
{
    ///< 触发系统复位
    reg32Write(TOP_CRG_SRST_LOCK_REG, SRST_LOCK_MAGIC);
    reg32Write(TOP_CRG_SRST_REQ_REG, 0);
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("DSB");
}

S32 peripsClockFreqGet(DevList_e devID, U32 *Hz)
{
    S32 ret = EXIT_SUCCESS;

    if ((NULL == Hz) || (devID >= DEVICE_NR)) {
        ret = -EXIT_FAILURE;
        goto exit;
    }
	
    ///< 如果当前系统没有切到高频，返会参考时钟，参考时钟有2路，一路是晶振25M，一路是差分100M;
    if (!isSystemClkPll()) {
        *Hz = SYS_XTAL_CLK_FREQ / 2;
#ifdef CONFIG_PLATFORM_FPGA
        if ((devID == DEVICE_SGPIO0) || (devID == DEVICE_SGPIO1) || (devID == DEVICE_SGPIO2) || (devID == DEVICE_SGPIO3)) {
            *Hz = SYS_XTAL_CLK_FREQ / 4;
        }
#endif
        goto exit;
    }

    ///< 切到高频后的时钟域
    switch (devID) {
    case DEVICE_UART0:
    case DEVICE_UART1:
    case DEVICE_SMBUS0:
    case DEVICE_SMBUS1:
    case DEVICE_SMBUS2:
    case DEVICE_SMBUS3:
    case DEVICE_SMBUS4:
    case DEVICE_SMBUS5:
    case DEVICE_SMBUS6:
    case DEVICE_TIMER0:
    case DEVICE_TIMER1:
    case DEVICE_TIMER2:
    case DEVICE_TIMER3:
    case DEVICE_TIMER4:
    case DEVICE_TIMER5:
    case DEVICE_TIMER6:
    case DEVICE_TIMER7:
    case DEVICE_TIMER8:
    case DEVICE_TIMER9:
    case DEVICE_TIMER10:
    case DEVICE_TIMER11:
    case DEVICE_WDT0:
    case DEVICE_WDT1:
    case DEVICE_WDT2:
    case DEVICE_WDT3:
    case DEVICE_PWM0:
    case DEVICE_TACH0:
    case DEVICE_TACH1:
    case DEVICE_TACH2:
    case DEVICE_TACH3:
    case DEVICE_I3C0:
    case DEVICE_I3C1:
        *Hz = PERIPS_CLK_FREQ_HZ;
        break;
    case DEVICE_DMA_0:
        *Hz = DMA_CLK_FREQ_HZ;
        break;
    case DEVICE_QSPI0:
        *Hz = QSPI_CLK_FREQ_HZ;
        break;
    case DEVICE_SGPIO0:
    case DEVICE_SGPIO1:
    case DEVICE_SGPIO2:
    case DEVICE_SGPIO3:
        *Hz = SGPIO_CLK_FREQ_HZ;
        break;
    ///< TODD  业务模块
    default:
        ret = -ENXIO;
        break;
    }

exit:
    return ret;
}

S32 peripsResetHold(DevList_e devID)
{
    S32 ret = EXIT_SUCCESS;
    U32 id;
    U32 regAddr;
    U32 peripsID = 0;
    U32 bitsNum;

    ///< 不是合法的设备id
    if (devID >= DEVICE_NR) {
        ret = -EXIT_FAILURE;
        goto exit;        
    }

    if (EXIT_SUCCESS != findPeripsByDevID(devID, &peripsID, &id)) {
        ret = -ENXIO;
        goto exit;
    }

    regAddr = gBspDevID2CrgModule[id].resetRegAddr;
    bitsNum = gBspDevID2CrgModule[id].resetBitsNr + peripsID; 

    switch (devID) {
    case DEVICE_QSPI0:
    case DEVICE_TRNG0:
    case DEVICE_SM2_0:
    case DEVICE_SM3_0:
    case DEVICE_SM4_0:
    case DEVICE_DMA_0:

#if !defined(TIANHE_BSP_SELFTEST_ENABLEE)
    SET_REG_BIT(regAddr, bitsNum);
#else 
    testPrint(regAddr, bitsNum);
#endif
        break;

    default:
#if !defined(TIANHE_BSP_SELFTEST_ENABLE)
    CLR_REG_BIT(regAddr, bitsNum);
#else 
    testPrint(regAddr, bitsNum);
#endif
        break;
    }

exit:
    return ret;
}

S32 peripsResetRelease(DevList_e devID)
{
    S32 ret = EXIT_SUCCESS;
    U32 id;
    U32 regAddr;
    U32 peripsID = 0;
    U32 bitsNum;

    ///< 不是合法的设备id
    if (devID >= DEVICE_NR) {
        ret = -EXIT_FAILURE;
        goto exit;        
    }

    if (EXIT_SUCCESS != findPeripsByDevID(devID, &peripsID, &id)) {
        ret = -ENXIO;
        goto exit;
    }

    regAddr = gBspDevID2CrgModule[id].resetRegAddr;
    bitsNum = gBspDevID2CrgModule[id].resetBitsNr + peripsID; 

    switch (devID) {
    case DEVICE_QSPI0:
    case DEVICE_TRNG0:
    case DEVICE_SM2_0:
    case DEVICE_SM3_0:
    case DEVICE_SM4_0:
    case DEVICE_DMA_0:   

#if !defined(TIANHE_BSP_SELFTEST_ENABLE)
    CLR_REG_BIT(regAddr, bitsNum);
#else 
    testPrint(regAddr, bitsNum);
#endif
        break;

    default:
#if !defined(TIANHE_BSP_SELFTEST_ENABLE)
    SET_REG_BIT(regAddr, bitsNum);
#else 
    testPrint(regAddr, bitsNum);
#endif
        break;
    }
    
exit:
    return ret;
}

S32 peripsReset(DevList_e devID)
{
    S32 ret = EXIT_SUCCESS;
    U32 id;
    U32 regAddr;
    U32 peripsID = 0;
    U32 bitsNum;

    ///< 不是合法的设备id
    if (devID >= DEVICE_NR) {
        ret = -EXIT_FAILURE;
        goto exit;        
    }

    if (EXIT_SUCCESS != findPeripsByDevID(devID, &peripsID, &id)) {
        ret = -ENXIO;
        goto exit;
    }

    regAddr = gBspDevID2CrgModule[id].resetRegAddr;
    bitsNum = gBspDevID2CrgModule[id].resetBitsNr + peripsID; 

    switch (devID) {
    case DEVICE_QSPI0:
    case DEVICE_TRNG0:
    case DEVICE_SM2_0:
    case DEVICE_SM3_0:
    case DEVICE_SM4_0:
    case DEVICE_DMA_0:

#if !defined(TIANHE_BSP_SELFTEST_ENABLE)
    SET_REG_BIT(regAddr, bitsNum);
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("DSB");
    CLR_REG_BIT(regAddr, bitsNum);
#else 
    testPrint(regAddr, bitsNum);
#endif
        break;

    default:
#if !defined(TIANHE_BSP_SELFTEST_ENABLE)
    CLR_REG_BIT(regAddr, bitsNum);
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("DSB");
    SET_REG_BIT(regAddr, bitsNum);
#else 
    testPrint(regAddr, bitsNum);
#endif
        break;
    }
    
exit:
    return ret;
}

S32 peripsClockEnable(DevList_e devID)
{
    S32 ret = EXIT_SUCCESS;
    U32 id;
    U32 regAddr;
    U32 peripsID = 0;
    U32 bitsNum;

    ///< 不是合法的设备id
    if (devID >= DEVICE_NR) {
        ret = -EXIT_FAILURE;
        goto exit;        
    }

    if (EXIT_SUCCESS != findPeripsByDevID(devID, &peripsID, &id)) {
        ret = -ENXIO;
        goto exit;
    }

    bitsNum = gBspDevID2CrgModule[id].clkBitsNr + peripsID;
    regAddr = gBspDevID2CrgModule[id].clkRegAddr;
    if (PERIPS_CLK_ENABLE_UNSUPPORT == regAddr) {
        ret = -ENXIO;
        goto exit;        
    }
     
#if !defined(TIANHE_BSP_SELFTEST_ENABLE)
    SET_REG_BIT(regAddr, bitsNum);
#else 
    testPrint(regAddr, bitsNum);
#endif
    
exit:
    return ret;
}

S32 peripsClockDisable(DevList_e devID)
{
    S32 ret = EXIT_SUCCESS;
    U32 id;
    U32 regAddr;
    U32 peripsID = 0;
    U32 bitsNum;

    ///< 不是合法的设备id
    if (devID >= DEVICE_NR) {
        ret = -EXIT_FAILURE;
        goto exit;        
    }

    if (EXIT_SUCCESS != findPeripsByDevID(devID, &peripsID, &id)) {
        ret = -ENXIO;
        goto exit;
    }

    bitsNum = gBspDevID2CrgModule[id].clkBitsNr + peripsID; 
    regAddr = gBspDevID2CrgModule[id].clkRegAddr;

    ///< 表示该模块不支持时钟门控
    if (PERIPS_CLK_ENABLE_UNSUPPORT == regAddr) {
        ret = -ENXIO;
        goto exit;        
    }
    
#if !defined(TIANHE_BSP_SELFTEST_ENABLE)
    CLR_REG_BIT(regAddr, bitsNum);
#else 
    testPrint(regAddr, bitsNum);
#endif
    
exit:
    return ret;
}

void pllInit(void)
{
    if (EXIT_SUCCESS != pllSwitch()) {
        LOGE("failed to switch pll!!!\r\n");
    }
}
