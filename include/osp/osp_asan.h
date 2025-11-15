
/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_asan.h
 * @author  lichenxiang
 * @date    2021.11.15
 * @brief   os asan define
 * @note    NA
 */

#ifndef __OSP_ASAN_H
#define __OSP_ASAN_H

#ifdef CFG_ASAN_SHADOW_OFFSET

#include <inner/osp_inner_asan.h>

#ifdef __cplusplus
extern "C" {
#endif


#define OSP_ASAN_USER_DATA_READ_ZONE INNER_OSP_ASAN_USER_DATA_READ_ZONE
#define OSP_ASAN_USER_HEAP_READ_ZONE INNER_OSP_ASAN_USER_HEAP_READ_ZONE

///< 用户自定redZone后，可以重新实现ospAsanUnknowReadzoneAnalyzer函数，此函数可以根据redZone 返回字符串描述
///< const char * ospAsanUnknowReadzoneAnalyzer(unsigned char redZone)


/**
 * @brief   解毒begin 到 end的 shadow 区域
 * @param   begin[in] 开始地址，必须8字节对齐
 * @param   end[in] 结束地址
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospAsanTagAccess(const void *begin, const void *end);

/**
 * @brief   使用value 毒化 begin 到 end的 shadow 区域
 * @param   begin[in] 开始地址，必须8字节对齐
 * @param   end[in] 结束地址
 * @param   毒化值
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospAsanTagNoAccessWithRedzone(const void *begin, const void *end,unsigned char  value);

/**
 * @brief   对指定区域手动检测是否访问合法
 * @param   addr [in], 开始地址
 * @param   size[in] , 长度
 * @param   isWrite[in] , 是否是写检测
 * @return  无
 * @warning NA
 * @note    NA
 */
bool ospAsanCheckAccessRange(void* addr, size_t size,bool isWrite);


/**
 * @brief   开启asan 检测出错后 panic, 默认关闭
 * @param
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospAsanPanicOn();

/**
 * @brief   关闭asan 检测出错后 panic, 默认关闭
 * @param
 * @return  无
 * @warning NA
 * @note    NA
 */
void ospAsanPanicOff();


#ifdef __cplusplus
}
#endif

#endif

#endif
