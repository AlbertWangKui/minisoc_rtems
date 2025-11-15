/**
 * Copyright (C), 2021, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_sys_clock.c
 * @author  tianye
 * @date    2021.11.10
 * @brief   封装sys clock操作的对外接口
 * @note    目前针对的是 Cortex-R5 的处理器
 */
#include <bspopts.h>
#include <osp_sys_clock.h>
#if defined(PS3OS_EXPANDER_V200)
#include <bsp/crg.h>

/**
 * @brief   获取指定IP的频率
 * @param   IP的ID
 * @return  时钟频率，单位是hz
 * @warning NA
 * @note    NA
 */
uint32_t ospGetPeripClk(OspPeripheralId_e peripheralId)
{
    return crg_get_clock((peripheral_name_t)peripheralId);
}

/**
 * @brief   复位指定模块
 * @param   模块id
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospHoldReset(OspModuleId_e module)
{
    return crg_hold_reset((module_name_t)module);
}

/**
 * @brief   解复位指定模块
 * @param   模块id
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospReleaseReset(OspModuleId_e module)
{
    return crg_release_reset((module_name_t)module);
}

/**
 * @brief   使能指定模块时钟
 * @param   模块id
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospClockEnable(OspModuleId_e module)
{
    return clock_enable((module_name_t)module);
}

/**
 * @brief   关闭指定模块时钟
 * @param   模块id
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospClockDisable(OspModuleId_e module)
{
    return clock_disable((module_name_t)module);
}

/**
 * @brief   获取系统频率
 * @param   none
 * @return  HZ
 * @warning NA
 * @note    NA
 */
uint32_t ospGetSysClock(void)
{
    return get_sys_clock();
}

#if defined(PS3OS_EXPANDER_V200)
#else
/**
 * @brief   获取模块复位状态
 * @param   模块id
 * @return  0 reset;1 unreset
 * @warning NA
 * @note    NA
 */
uint32_t ospGetResetStatus(OspModuleId_e module)
{
    return crg_get_rst_status((module_name_t)module);
}

/**
 * @brief   获取模块时钟状态
 * @param   模块id
 * @return  0 clk disable;1 clk enable
 * @warning NA
 * @note    NA
 */
uint32_t ospGetClkStatus(OspModuleId_e module)
{
    return crg_get_clk_status((module_name_t)module);
}

/**
 * @brief select lane 44~47 serdes, 1 for newsemi,0 for guc
 * @details none
 * @param [in] type serdes type,newsemi or guc
 * @return none
 * @note none
 * @warning none
 */
void ospSerdesSelect(OspSerdesSel_e type)
{
    return crg_serdes_select(type);
}

/**
 * @brief get serdes type, 1 for newsemi,0 for guc
 * @details none
 * @param [in] none
 * @param [out] none
 * @return serdes type
 * @note none
 * @warning none
 */
uint32_t ospGetSerdesType(void)
{
    return crg_get_serdes_type();
}
/**
 * @brief 设置phy ctrl 的分频因子
 * @details none
 * @param [in] name 表示分频因子
 * @param [out] none
 * @return none
 * @note none
 * @warning none
 */
void ospSetPmClkDiv(OspPhyCtrlPmDiv_e pmDiv)
{
    return crg_set_pm_clk_div(pmDiv);
}
#endif

#elif defined(PS3OS_NETCHIP) || defined(PS3OS_NIC_100G) || defined(PS3OS_SWITCH_V300) || defined(PS3OS_IBCHIP) || defined(PS3OS_IBCHIP_V100)
#include <bsp/crg.h>
/**
 * @brief   获取系统频率
 * @param   none
 * @return  HZ
 * @warning NA
 * @note    NA
 */
uint32_t ospGetSysClock(void)
{
    return get_sys_clock();
}
#endif
