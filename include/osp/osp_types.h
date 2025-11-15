/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_types.h
 * @author  lichenxiang
 * @date    2020.08.04
 * @brief   porting types
 * @note    NA
 */

#ifndef __OSP_TYPES_H__
#define __OSP_TYPES_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef uint32_t OspID;
typedef uint32_t OspName;
typedef uint32_t OspInterval;
typedef uint32_t OspEventSet;

typedef uintptr_t OspTaskArgument;

typedef uint32_t OspVector;
typedef uint32_t OspStatesControl;

/**
 * @brief Data structure to manage and manipulate calendar
 * @ref ClassicRTEMSSecTime "time".
 */
typedef struct OspTimeOfDay {
    /**
     * @brief Year, A.D.
     */
    uint32_t year;
    /**
     * @brief Month, 1 .. 12.
     */
    uint32_t month;
    /**
     * @brief Day, 1 .. 31.
     */
    uint32_t day;
    /**
     * @brief Hour, 0 .. 23.
     */
    uint32_t hour;
    /**
     * @brief Minute, 0 .. 59.
     */
    uint32_t minute;
    /**
     * @brief Second, 0 .. 59.
     */
    uint32_t second;
    /**
     * @brief Elapsed ticks between seconds.
     */
    uint32_t ticks;
} OspTimeOfDay_s;

#ifdef __cplusplus
}
#endif

#endif
