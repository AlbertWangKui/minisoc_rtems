/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file kconfig.h
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/06/07
 * @brief 全局配置头文件
 */

#ifndef __KCONFIG_H__
#define __KCONFIG_H__

 /*
 * 1. 全局配置区域，所有全局配置都在这里定义，不要在其它地方定义
 * 2. 未定义的宏请显性使用#undef
 * 3. 包含关系为：minisoc_fw_config.h -> kconfig.h -> bsp_config.h ...用户只需要包含bsp_config.h即可
 */

/*
 * 这个文件是menuconfig自动生成的
 */
#include "menuconfig.h"
/*
 *  possible BSPs and platforms:
 *
 *  CONFIG_BSP_SHESHOU
 *  CONFIG_BSP_TIANHE
 *  CONFIG_BSP_TIANLONG_CS
 *  CONFIG_BSP_TIANYING
 *  CONFIG_PLATFORM_FPGA
 *  CONFIG_PLATFORM_FPGA_S2C
 *  CONFIG_PLATFORM_FPGA_HAPS
 *  CONFIG_PLATFORM_FPGA_VU19P
 *  CONFIG_PLATFORM_FPGA_1902
 *  CONFIG_PLATFORM_EMU
 *  CONFIG_PLATFORM_EMU_PZ1
 *  CONFIG_PLATFORM_EMU_VEL
 */
/*
 * 如果你想使用快速索引，请在你的BSP头文件中定义 DEVICE_FAST_INDEXING。
 * 这将加快设备查找过程，但要求设备ID必须等于设备列表中的索引。
 * 如果没有定义该宏，设备ID可以是任意值，查找时会遍历整个设备列表。
 */
#undef DEVICE_FAST_INDEXING

/*
 * soc模块自测开关。
 */
//#define TIANHE_BSP_SELFTEST_ENABLE
#undef TIANHE_BSP_SELFTEST_ENABLE

#endif /* __KCONFIG_H__ */
