/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file soc_ctrl.c
 * @author taohb@starsmicrosystem.com
 * @date 2024/07/03
 * @brief  soc_ctrl.soc_csrs register
 *
 * @info: SVN//minisoc_junior.soc_ctrl.soc_csrs_reglist.xlsx
 * @version
 *  Hardware version:
 *      v0.0 初版
 *      v0.1 1. 添加了LAR
 *           2. 调整了时钟分频系数配置的寄存器和软件操作方式
 *      v0.2 增加了 build cfg 和版本寄存器
 *      v0.3 增加了 debug monitor 选择控制寄存器
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2024/07/03   taohb           the first version
 *
 *
 */

#include <stdint.h>
#include "bsp_config.h"
#include "common_defines.h"
#include "soc_ctrl.h"

S32 socCtrlPeriClkEnableConfig(BspSocCtrlPeriModule_e module, Bool status)
{
    S32 ret = EXIT_SUCCESS;

    ClockEnableReg_u *clock_enable = &SOC_CTRL->clkDivEnable;

    switch (module) {
    case BSP_SOC_CTRL_PERI_MODULE_QFLASH:
        clock_enable->fields.ocmSysQspi = status;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_GDMA:
        clock_enable->fields.gdma = status;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM2:
    case BSP_SOC_CTRL_PERI_MODULE_SM3:
    case BSP_SOC_CTRL_PERI_MODULE_SM4:
    case BSP_SOC_CTRL_PERI_MODULE_TRNG:
    case BSP_SOC_CTRL_PERI_MODULE_SOC:
        ret = (status) ? EXIT_SUCCESS : -EXIT_FAILURE; ///< no support disable
        break;
    default:
        ret = -EXIT_FAILURE;
        break;
    }
    return ret;
}

S32 socCtrlPeriResetHold(BspSocCtrlPeriModule_e module)
{
    S32 ret = EXIT_SUCCESS;
    MinisocSoftRstReg_u *minisocSoftReset = &(SOC_CTRL->minisocSoftRst);
    SocSoftResetReg_u *socSoftReset = &(SOC_CTRL->socSoftReset);

    switch (module) {
    case BSP_SOC_CTRL_PERI_MODULE_QFLASH:
        minisocSoftReset->fields.ocmSysQspi = 1;
        reg32Read((uint32_t)minisocSoftReset); ///< 王强：先写后读能保序且延时足够
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM2:
        minisocSoftReset->fields.secSysSm2 = 1;
        reg32Read((uint32_t)minisocSoftReset); ///< 王强：先写后读能保序且延时足够
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM3:
        minisocSoftReset->fields.secSysSm3 = 1;
        reg32Read((uint32_t)minisocSoftReset); ///< 王强：先写后读能保序且延时足够
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM4:
        minisocSoftReset->fields.secSysSm4 = 1;
        reg32Read((uint32_t)minisocSoftReset); ///< 王强：先写后读能保序且延时足够
        break;
    case BSP_SOC_CTRL_PERI_MODULE_TRNG:
        minisocSoftReset->fields.secSysTrng = 1;
        reg32Read((uint32_t)minisocSoftReset); ///< 王强：先写后读能保序且延时足够
        break;
    case BSP_SOC_CTRL_PERI_MODULE_GDMA:
        minisocSoftReset->fields.gdma = 1;
        reg32Read((uint32_t)minisocSoftReset); ///< 王强：先写后读能保序且延时足够
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SOC:
        socSoftReset->fields.fullSystem = 1;
        reg32Read((uint32_t)socSoftReset); ///< 王强：先写后读能保序且延时足够
        break;
    default:
        ret = -EXIT_FAILURE;
        break;
    }
    return ret;
}

S32 socCtrlPeriResetRelease(BspSocCtrlPeriModule_e module)
{
    S32 ret = EXIT_SUCCESS;
    MinisocSoftRstReg_u *minisocSoftReset = &(SOC_CTRL->minisocSoftRst);
    SocSoftResetReg_u *socSoftReset = &(SOC_CTRL->socSoftReset);

    switch (module) {
    case BSP_SOC_CTRL_PERI_MODULE_QFLASH:
        minisocSoftReset->fields.ocmSysQspi = 0;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM2:
        minisocSoftReset->fields.secSysSm2 = 0;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM3:
        minisocSoftReset->fields.secSysSm3 = 0;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM4:
        minisocSoftReset->fields.secSysSm4 = 0;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_TRNG:
        minisocSoftReset->fields.secSysTrng = 0;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_GDMA:
        minisocSoftReset->fields.gdma = 0;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SOC:
        socSoftReset->fields.fullSystem = 0;
        break;
    default:
        ret = -EXIT_FAILURE;
        break;
    }
    return ret;
}

S32 socCtrlPeriReset(BspSocCtrlPeriModule_e module)
{
    S32 ret = EXIT_SUCCESS;
    MinisocSoftRstReg_u *minisocSoftReset = &(SOC_CTRL->minisocSoftRst);
    SocSoftResetReg_u *socSoftReset = &(SOC_CTRL->socSoftReset);
#ifdef SOC_SELFTEST_ENABLE
{
    volatile U32 *reg;
    U32 regBit;

    (void)minisocSoftReset;
    (void)socSoftReset;

    switch (module) {
    case BSP_SOC_CTRL_PERI_MODULE_QFLASH:
        reg = (volatile U32 *)&SOC_CTRL->minisocSoftRst;
        GET_REG_STRUCT_BIT_POS(SOC_CTRL->minisocSoftRst, ocmSysQspi, regBit);
        testCrgSetRegBitPrint((U32)reg, regBit);
        testCrgClearRegBitPrint((U32)reg, regBit);
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM2:
        reg = (volatile U32 *)&SOC_CTRL->minisocSoftRst;
        GET_REG_STRUCT_BIT_POS(SOC_CTRL->minisocSoftRst, secSysSm2, regBit);
        testCrgSetRegBitPrint((U32)reg, regBit);
        testCrgClearRegBitPrint((U32)reg, regBit);
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM3:
        reg = (volatile U32 *)&SOC_CTRL->minisocSoftRst;
        GET_REG_STRUCT_BIT_POS(SOC_CTRL->minisocSoftRst, secSysSm3, regBit);
        testCrgSetRegBitPrint((U32)reg, regBit);
        testCrgClearRegBitPrint((U32)reg, regBit);
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM4:
        reg = (volatile U32 *)&SOC_CTRL->minisocSoftRst;
        GET_REG_STRUCT_BIT_POS(SOC_CTRL->minisocSoftRst, secSysSm4, regBit);
        testCrgSetRegBitPrint((U32)reg, regBit);
        testCrgClearRegBitPrint((U32)reg, regBit);
        break;
    case BSP_SOC_CTRL_PERI_MODULE_TRNG:
        reg = (volatile U32 *)&SOC_CTRL->minisocSoftRst;
        GET_REG_STRUCT_BIT_POS(SOC_CTRL->minisocSoftRst, secSysTrng, regBit);
        testCrgSetRegBitPrint((U32)reg, regBit);
        testCrgClearRegBitPrint((U32)reg, regBit);
        break;
    case BSP_SOC_CTRL_PERI_MODULE_GDMA:
        reg = (volatile U32 *)&SOC_CTRL->minisocSoftRst;
        GET_REG_STRUCT_BIT_POS(SOC_CTRL->minisocSoftRst, gdma, regBit);
        testCrgSetRegBitPrint((U32)reg, regBit);
        testCrgClearRegBitPrint((U32)reg, regBit);
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SOC:
        reg = (volatile U32 *)&SOC_CTRL->socSoftReset;
        GET_REG_STRUCT_BIT_POS(SOC_CTRL->socSoftReset, fullSystem, regBit);
        testCrgSetRegBitPrint((U32)reg, regBit);
        testCrgClearRegBitPrint((U32)reg, regBit);
        break;
    default:
        ret = -EXIT_FAILURE;
        break;
    }
}
#else
    switch (module) {
    case BSP_SOC_CTRL_PERI_MODULE_QFLASH:
        minisocSoftReset->fields.ocmSysQspi = 1;
        reg32Read((uint32_t)minisocSoftReset); ///< 王强：先写后读能保序且延时足够
        minisocSoftReset->fields.ocmSysQspi = 0;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM2:
        minisocSoftReset->fields.secSysSm2 = 1;
        reg32Read((uint32_t)minisocSoftReset); ///< 王强：先写后读能保序且延时足够
        minisocSoftReset->fields.secSysSm2 = 0;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM3:
        minisocSoftReset->fields.secSysSm3 = 1;
        reg32Read((uint32_t)minisocSoftReset); ///< 王强：先写后读能保序且延时足够
        minisocSoftReset->fields.secSysSm3 = 0;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SM4:
        minisocSoftReset->fields.secSysSm4 = 1;
        reg32Read((uint32_t)minisocSoftReset); ///< 王强：先写后读能保序且延时足够
        minisocSoftReset->fields.secSysSm4 = 0;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_TRNG:
        minisocSoftReset->fields.secSysTrng = 1;
        reg32Read((uint32_t)minisocSoftReset); ///< 王强：先写后读能保序且延时足够
        minisocSoftReset->fields.secSysTrng = 0;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_GDMA:
        minisocSoftReset->fields.gdma = 1;
        reg32Read((uint32_t)minisocSoftReset); ///< 王强：先写后读能保序且延时足够
        minisocSoftReset->fields.gdma = 0;
        break;
    case BSP_SOC_CTRL_PERI_MODULE_SOC:
        socSoftReset->fields.fullSystem = 1;
        reg32Read((uint32_t)socSoftReset); ///< 王强：先写后读能保序且延时足够
        socSoftReset->fields.fullSystem = 0;
        break;
    default:
        ret = -EXIT_FAILURE;
        break;
    }
#endif
    return ret;
}
