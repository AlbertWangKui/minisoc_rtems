/** * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_common.h
 * @author  tianye
 * @date    2020.09.03
 * @brief   对外公共函数接口
 * @note    NA
 */

#ifndef __OSP_COMMON_H__
#define __OSP_COMMON_H__

#include <osp_types.h>
#include <osp_status.h>
#include <inner/osp_inner_support.h>
#include <bspopts.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OSP_SEARCH_ALL_NODES   0UL
#define OSP_SEARCH_OTHER_NODES 0x7FFFFFFEUL
#define OSP_SEARCH_LOCAL_NODE  0x7FFFFFFFUL

#define OSP_MILLISECONDS_TO_TICKS(_ms) INNER_MILLISECONDS_TO_TICKS(_ms)

#define OSP_MICROSECONDS_TO_TICKS(_us) INNER_MICROSECONDS_TO_TICKS(_us)

#define OSP_NAME_LEN (5)

#if defined(PS3OS_HBA_V200) || defined(PS3OS_HBA_V200_DEBUG)
#define PS3OS_OSP_PERF_SECTION __attribute__((section(".perf")))
#else
#define PS3OS_OSP_PERF_SECTION
#endif

/**
 * @brief   osp状态码转字符串接口
 * @param   OspStatusCode_e [in], 状态码
 * @return  const char*：转换后的状态码字符串
 * @warning NA
 * @note    NA
 */
const char *ospStatusText(OspStatusCode_e code);

/**
 * @brief   封装build name接口
 * @param   char1 [in],   第一个字符
 * @param   char2 [in],   第二个字符
 * @param   char3 [in],   第三个字符
 * @param   char4 [in],   第四个字符
 * @return  OspName： 转换后的os适配层name
 * @warning NA
 * @note    NA
 */
OspName ospBuildName(char char1, char char2, char char3, char char4);

/**
 * @brief   osp name转换为字符串
 * @param   OspName [in], osp name,U32，每8位表示一个字符
 * @param   stringName [out], 转换为字符串形式buffer空间，4个字符加1个结束符
 * @param   size [in], 长度为4个字符加1个结束符
 * @return  OspName： 转换后的os适配层name
 * @warning NA
 * @note    buffer size固定为OSP_NAME_LEN
 */
OspStatusCode_e ospNameToStringName(OspName ospName, char *stringName, uint32_t size);

/**
 * @brief   把1秒换算为系统滴答计数
 * @param   NA
 * @return  OspInterval 系统滴答(tick)计数
 * @warning NA
 * @note    NA
 */
OspInterval ospClockGetTicksPerSecond(void);

#ifdef __cplusplus
}
#endif

#endif
