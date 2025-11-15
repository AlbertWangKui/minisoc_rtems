/**
 * Copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file board_init.h
 * @author shaogl@starsmicrosystem.com
 * @date 2025.05.24
 * @brief board init for bsp.
 */

#ifndef __BOARD_INIT_H__
#define __BOARD_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "common_defines.h"

#define SYS_GPIO_RST_REG (0xb800A074)
#define SYS_GPIO_RST_BITS_NR (0)

/**
 * @brief board init.
 * @param [in] none
 * @param [out] none
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 * @warning 不阻塞；可重入; OS启动前；不可用于中断上下文；不可用于线程上下文
 */
S32 boardInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_INIT_H__ */