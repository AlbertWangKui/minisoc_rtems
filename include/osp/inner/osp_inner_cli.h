/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_inner_cli.h
 * @author  tanzhengrong
 * @date    2023.07.03
 * @brief   porting inner cli
 * @note    NA
 */
#ifndef __OSP_INNER_CLI_H__
#define __OSP_INNER_CLI_H__

#ifndef __OSP_CLI_H__
#error "include osp_cli.h instead include this header file"
#endif

#include <rtems/coredump.h>

#ifdef __cplusplus
extern "C" {
#endif


#define OSP_INNER_COREDUMP_FUN_DMA          RTEMS_COREDUMP_FUN_DMA
#define OSP_INNER_COREDUMP_FUN_NOF          RTEMS_COREDUMP_FUN_NOF
#define OSP_INNER_COREDUMP_FUN_MEM          RTEMS_COREDUMP_FUN_MEM
#define OSP_INNER_COREDUMP_FUN_FILE         RTEMS_COREDUMP_FUN_FILE
#define OSP_INNER_COREDUMP_FUN_CON          RTEMS_COREDUMP_FUN_CON
#define OSP_INNER_COREDUMP_FUN_GDMA         RTEMS_COREDUMP_FUN_GDMA
#define OSP_INNER_COREDUMP_FUN_FLASH        RTEMS_COREDUMP_FUN_FLASH
#define OSP_INNER_COREDUMP_FUN_FLASH_V2        RTEMS_COREDUMP_FUN_FLASH_V2

#ifdef __cplusplus
}
#endif

#endif

