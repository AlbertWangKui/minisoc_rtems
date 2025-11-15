/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_memory.c
 * @author  tianye
 * @date    2020.08.29
 * @brief   封装rtems操作系统内存管理对外接口
 * @note    NA
 */

#include <osp_memory.h>
#include "common_defines.h"
#include <bsp/linker-symbols.h>

///< 内存区域描述表
OspMemAreaInfo_s gMemAreaInfo[] = {
    {
        OSP_MEM_SRAM_OS_USED,
        (size_t)bsp_sram_os_used_begin,
        (size_t)bsp_sram_os_used_len
    },
};

U32 OspMemAreaInfoCntGet(void)
{
    return ARRAY_SIZE(gMemAreaInfo);
}
