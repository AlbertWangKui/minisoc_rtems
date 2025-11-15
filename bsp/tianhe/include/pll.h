/**
 * copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file pll.h
 * @author shaogl@starsmicrosystem.com
 * @date 2023/03/01
 * @brief
 */

#ifndef _PLL_H
#define _PLL_H

#include <stdbool.h>
#include "common_defines.h"

#include <stdint.h>

#define PLL_ACCESS_MAGIC (0xACCE5500)
#define PLL_READY_DONE (1)
#define MINISOC_CLK_DONE_MAGIC (0)
#define PLL_MUX_DONE_BITS_OFST (0)
#define MINISOC_CLK_POLLING_TIMEOUT (0x10000)
#define PLL_READY_POLLING_TIMEOUT (0x10000)
#define PLL_READY_BITS_MASK (1)

#define REG_STS0_PLL0 (0xB8000008)
#define REG_STS0_PLL1 (0xB8001008)
#define REG_STS0_PLL2 (0xB8002008)
#define REG_CLK_CFG_LOCK (0xB8008600)
#define REG_ICG_CLK_PLL0 (0xB8008200)
#define REG_ICG_CLK_PLL1 (0xB8008204)
#define REG_ICG_CLK_PLL2 (0xB8008208)
#define REG_DIV_CLK_TOP_CTRL_APB (0xB8008000)
#define REG_DIV_CLK_SOC_BUS_H (0xB8008004)
#define REG_DIV_CLK_SDS_CPU (0xB8008008)
#define REG_DIV_CLK_AUX (0xB800800C)
#define REG_DIV_CLK_SOC_BUS_L (0xB8008010)
#define REG_DIV_CLK_TOP_CTRL_PVT_H (0xB8008014)
#define REG_DIV_CLK_TOP_CTRL_PVT_L (0xB8008018)
#define REG_DIVISOR_UPDATE (0xB8008050)
#define REG_PLL0_MUX_SEL (0xB8008100)
#define REG_PLL1_MUX_SEL (0xB8008104)
#define REG_PLL2_MUX_SEL (0xB8008108)

#define REG_CLOCK_DIVISOR_CONFIG0 (0xBE100020)
#define REG_CLOCK_DIVISOR_CONFIG1 (0xBE100024)
#define REG_CLOCK_DIVISOR_UPDATE (0xBE100028)

typedef enum pllId {
    PLL_ID_0,
    PLL_ID_1,
    PLL_ID_2,
    PLL_ID_MAX,
} PllId_e;

/**
 * @brief 时钟模块初始化
 * @details none
 * @param [in] none
 * @param [inout] none
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 * @warning 不阻塞,不可重入,OS启动前,不可用于中断上下文,可用于线程上下文
 */
S32 pllSwitch(void);

/**
 * @brief  判断系统时钟是否切到了高频
 * @details none
 * @param [in] void
 * @param [inout] none
 * @return  true or false
 * @warning 不阻塞,可重入,OS启动前/后,可用于中断上下文,可用于线程上下文
 */
Bool isSystemClkPll(void);

#endif /* _PLL_H */
