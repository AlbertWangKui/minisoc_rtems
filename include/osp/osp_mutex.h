/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_mutex.h
 * @author  lichenxiang
 * @date    2020.08.04
 * @brief   porting rtems mutex
 * @note    NA
 */

#ifndef __OSP_MUTEX_H
#define __OSP_MUTEX_H

#include <osp_status.h>
#include <osp_types.h>
#include <inner/osp_inner_mutex.h>

//__BEGIN_DECLS
#ifdef __cplusplus
extern "C" {
#endif

#define OSP_MUTEX_INITIALIZER(name)           INNER_OSP_MUTEX_INITIALIZER(name)
#define OSP_RECURSIVE_MUTEX_INITIALIZER(name) INNER_OSP_RECURSIVE_MUTEX_INITIALIZER(name)

typedef InnerOspMutex_t OspMutex_t;
typedef InnerOspRecursiveMutex_t OspRecursiveMutex_t;

/**
 * @brief   初始化互斥锁，不支持递归
 * @param   mutex [in/out], 需要初始化的互斥锁
 * @param   name [in],      互斥锁的名字
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospMutexInit(OspMutex_t *mutex, const char *name);

/**
 * @brief   获取给定mutex句柄的名字
 * @param   mutex [in], 互斥锁句柄
 * @return  返回mutex的名字
 * @warning 无
 * @note    无
 */
const char *ospMutexGetName(const OspMutex_t *mutex);

/**
 * @brief   设置mutex的名字
 * @param   mutex [in/out], 互斥锁句柄
 * @param   name [in],      需要设置的名字
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospMutexSetName(OspMutex_t *mutex, const char *name);

/**
 * @brief   对mutex进行加锁
 * @param   mutex [in/out], 互斥锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospMutexLock(OspMutex_t *mutex);

/**
 * @brief   对mutex进行加锁
 * @param   mutex [in/out], 互斥锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
int ospMutexTryLock(OspMutex_t *mutex);

/**
 * @brief   对mutex解锁
 * @param   mutex [in/out], 互斥锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospMutexUnlock(OspMutex_t *mutex);

/**
 * @brief   销毁mutex,释放其内存
 * @param   mutex [in/out], 互斥锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospMutexDestroy(OspMutex_t *mutex);

/**
 * @brief   初始化递归锁，此锁可以在同一task中多次加锁
 * @param   mutex [in/out], 递归锁句柄
 * @param   name [in],      设置的名字
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospRecursiveMutexInit(OspRecursiveMutex_t *mutex, const char *name);

/**
 * @brief   获取递归锁名字
 * @param   mutex [in], 需要获取名字的递归锁句柄
 * @return  返回递归锁名字
 * @warning 无
 * @note    无
 */
const char *ospRecursiveMutexGetName(const OspRecursiveMutex_t *mutex);

/**
 * @brief   设置递归锁名字
 * @param   mutex [in/out], 递归锁句柄
 * @param   name [in],      设置的名字
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospRecursiveMutexSetName(OspRecursiveMutex_t *mutex, const char *name);

/**
 * @brief
 * 对递归锁进行加锁，在统一task中可以重复加锁，但是解锁时需要与解锁接口对应
 * @param   mutex [in/out], 递归锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospRecursiveMutexLock(OspRecursiveMutex_t *mutex);

/**
 * @brief
 * 对递归锁进行加锁，在统一task中可以重复加锁，但是解锁时需要与解锁接口对应
 * @param   mutex [in/out], 递归锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
int ospRecursiveMutexTryLock(OspRecursiveMutex_t *mutex);

/**
 * @brief   对递归锁进行解锁，加锁几次就需要解锁几次
 * @param   mutex [in/out], 递归锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospRecursiveMutexUnlock(OspRecursiveMutex_t *mutex);

/**
 * @brief   销毁递归锁
 * @param   mutex [in/out], 递归锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospRecursiveMutexDestroy(OspRecursiveMutex_t *mutex);

#ifdef __cplusplus
}
#endif
//__END_DECLS
#endif
