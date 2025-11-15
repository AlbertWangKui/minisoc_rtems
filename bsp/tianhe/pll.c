/**
 * copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file pll.c
 * @author shaogl@starsmicrosystem.com
 * @date 2023/03/01
 * @brief
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "pll.h"
#include "bsp_config.h"
#include "common_defines.h"

static S32 isPllReady(void)
{
    S32 ret = EXIT_SUCCESS;
    U32 regVal[PLL_ID_MAX - 1];
    U32 temp;
    U32 timeout = PLL_READY_POLLING_TIMEOUT;

    do {
        regVal[PLL_ID_0] = reg32Read(REG_STS0_PLL0);
        regVal[PLL_ID_1] = reg32Read(REG_STS0_PLL1);
        regVal[PLL_ID_2] = reg32Read(REG_STS0_PLL2);

        temp = regVal[PLL_ID_0] & regVal[PLL_ID_1];
        temp &= regVal[PLL_ID_2];
        temp &= PLL_READY_BITS_MASK;
        timeout--;
    } while ((PLL_READY_DONE != temp) && (timeout > 0));

    if (PLL_READY_DONE != temp) {
        ret = -EXIT_FAILURE;
    }

    return ret;
}

static void pllUnlock(void)
{
    reg32Write(REG_CLK_CFG_LOCK, PLL_ACCESS_MAGIC);
}

static void pllLock(void)
{
    reg32Write(REG_CLK_CFG_LOCK, 0);
}

static void pllIcgEnable(void)
{
    reg32Write(REG_ICG_CLK_PLL0, 1);
    reg32Write(REG_ICG_CLK_PLL1, 1);
    reg32Write(REG_ICG_CLK_PLL2, 1);
}

static void topClkDivParamSet(void)
{
    reg32Write(REG_DIV_CLK_TOP_CTRL_APB, 9);
    reg32Write(REG_DIV_CLK_SOC_BUS_H, 1);
    reg32Write(REG_DIV_CLK_SDS_CPU, 3);
    reg32Write(REG_DIV_CLK_AUX, 0x13);
    reg32Write(REG_DIV_CLK_SOC_BUS_L, 7);
    reg32Write(REG_DIV_CLK_TOP_CTRL_PVT_H, 3);
    reg32Write(REG_DIV_CLK_TOP_CTRL_PVT_L, 0x13);
    asm volatile ("DSB");
    asm volatile ("ISB");
}

static void topClkDivParamUpdate(void)
{
    reg32Write(REG_DIVISOR_UPDATE, 1);
    asm volatile ("NOP");
    asm volatile ("NOP");
    asm volatile ("DSB");
    asm volatile ("ISB");
}

static void minisocClkDivParamSet(void)
{
    reg32Write(REG_CLOCK_DIVISOR_CONFIG0, 0x10055);
    reg32Write(REG_CLOCK_DIVISOR_CONFIG1, 0x305);
    asm volatile ("DSB");
}

static void minisocClkDivParamUpdate(void)
{
    reg32Write(REG_CLOCK_DIVISOR_UPDATE, 1);
    asm volatile ("NOP");
    asm volatile ("NOP");
    asm volatile ("DSB");
    asm volatile ("ISB");
}

static S32 isMinisocClkDivParamValid(void)
{
    S32 ret = EXIT_SUCCESS;
    U32 val;
    U32 timeout = MINISOC_CLK_POLLING_TIMEOUT;

    do {
        val = reg32Read(REG_CLOCK_DIVISOR_UPDATE);
        timeout--;
    } while ((MINISOC_CLK_DONE_MAGIC != val) && (timeout > 0));

    if (MINISOC_CLK_DONE_MAGIC != val) {
        ret = -EXIT_FAILURE;
    }

    return ret;
}

static void pllMuxSwitch(void)
{
    reg32Write(REG_PLL0_MUX_SEL, 1);
    reg32Write(REG_PLL1_MUX_SEL, 1);
    reg32Write(REG_PLL2_MUX_SEL, 1);
    asm volatile ("NOP");
    asm volatile ("NOP");
    asm volatile ("DSB");
    asm volatile ("ISB");
}

Bool isSystemClkPll(void)
{
    U32 val;

    val = reg32Read(REG_PLL0_MUX_SEL);

    return (BIT_IS_ZERO(val, PLL_MUX_DONE_BITS_OFST)? false : true);
}

/**
  * @brief 初始化PLL.
  * @param [in]  none.
  * @return EXIT_FAILURE or EXIT_SUCCESS
  */
S32 pllSwitch(void)
{
    S32 ret = EXIT_SUCCESS;

    ///< fpga平台不需要切高频
#if defined(CONFIG_PLATFORM_FPGA)
    return EXIT_SUCCESS;
#endif

    ///< 为了兼容romBoot,如果系统时钟已切到高频，则直接返回;
    if (isSystemClkPll()) {
        goto exit;   
    }

    ///< 判断PLL是否ready
    if (EXIT_SUCCESS != isPllReady()) {
        ret = -EXIT_FAILURE;
        goto exit;
    }
    
    pllUnlock();
    pllIcgEnable();
    topClkDivParamSet();
    topClkDivParamUpdate();
    minisocClkDivParamSet();
    minisocClkDivParamUpdate();
    if (EXIT_SUCCESS != isMinisocClkDivParamValid()) {
        ret = -EXIT_FAILURE;
        goto exit;        
    }

    pllMuxSwitch();
    pllLock();    

exit:
    return ret;
}
