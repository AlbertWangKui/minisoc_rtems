/**
 * Copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file minisoc_init.h
 * @author taohb@starsmicrosystem.com
 * @date 2023.11.18
 * @brief bsp init, it is interface for minisoc library.
 */

#ifndef __MINISOC_INIT_H__
#define __MINISOC_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

/**
 * @brief minisoc init.
 * @return -EXIT_FAILURE or EXIT_SUCCESS
 */
S32 socInit(void);

#ifdef __cplusplus
}
#endif

#endif /* _MINISOC_INIT_H */
