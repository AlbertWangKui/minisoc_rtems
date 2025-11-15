/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_mutex.c
 * @author  lichenxiang
 * @date    2020.09.08
 * @brief   封装rtems操作系统mutex对外接口
 * @note    NA
 */

#include <osp_mutex.h>

/**
 * @brief   初始化互斥锁，不支持递归
 * @param   mutex [in/out], 需要初始化的互斥锁
 * @param   name [in],      互斥锁的名字
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospMutexInit(OspMutex_t *mutex, const char *name)
{
    rtems_mutex_init((rtems_mutex *)mutex, name);
}

/**
 * @brief   获取给定mutex句柄的名字
 * @param   mutex [in], 互斥锁句柄
 * @return  返回mutex的名字
 * @warning 无
 * @note    无
 */
const char *ospMutexGetName(const OspMutex_t *mutex)
{
    return rtems_mutex_get_name((const rtems_mutex *)mutex);
}

/**
 * @brief   设置mutex的名字
 * @param   mutex [in/out], 互斥锁句柄
 * @param   name [in],      需要设置的名字
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospMutexSetName(OspMutex_t *mutex, const char *name)
{
    rtems_mutex_set_name((rtems_mutex *)mutex, name);
}

/**
 * @brief   对mutex进行加锁
 * @param   mutex [in/out], 互斥锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospMutexLock(OspMutex_t *mutex)
{
    rtems_mutex_lock((rtems_mutex *)mutex);
}

/**
 * @brief   对mutex进行加锁
 * @param   mutex [in/out], 互斥锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
int ospMutexTryLock(OspMutex_t *mutex)
{
    return _Mutex_Try_acquire((rtems_mutex *)mutex);
}

/**
 * @brief   对mutex解锁
 * @param   mutex [in/out], 互斥锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospMutexUnlock(OspMutex_t *mutex)
{
    rtems_mutex_unlock((rtems_mutex *)mutex);
}

/**
 * @brief   销毁mutex,释放其内存
 * @param   mutex [in/out], 互斥锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospMutexDestroy(OspMutex_t *mutex)
{
    rtems_mutex_destroy((rtems_mutex *)mutex);
}

/**
 * @brief   初始化递归锁，此锁可以在同一task中多次加锁
 * @param   mutex [in/out], 递归锁句柄
 * @param   name [in],      设置的名字
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospRecursiveMutexInit(OspRecursiveMutex_t *mutex, const char *name)
{
    rtems_recursive_mutex_init((rtems_recursive_mutex *)mutex, name);
}

/**
 * @brief   获取递归锁名字
 * @param   mutex [in], 需要获取名字的递归锁句柄
 * @return  返回递归锁名字
 * @warning 无
 * @note    无
 */
const char *ospRecursiveMutexGetName(const OspRecursiveMutex_t *mutex)
{
    return rtems_recursive_mutex_get_name((const rtems_recursive_mutex *)mutex);
}

/**
 * @brief   设置递归锁名字
 * @param   mutex [in/out], 递归锁句柄
 * @param   name [in],      设置的名字
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospRecursiveMutexSetName(OspRecursiveMutex_t *mutex, const char *name)
{
    rtems_recursive_mutex_set_name((rtems_recursive_mutex *)mutex, name);
}

/**
 * @brief
 * 对递归锁进行加锁，在统一task中可以重复加锁，但是解锁时需要与解锁接口对应
 * @param   mutex [in/out], 递归锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospRecursiveMutexLock(OspRecursiveMutex_t *mutex)
{
    rtems_recursive_mutex_lock((rtems_recursive_mutex *)mutex);
}

/**
 * @brief
 * 对递归锁进行加锁，在统一task中可以重复加锁，但是解锁时需要与解锁接口对应
 * @param   mutex [in/out], 递归锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
int ospRecursiveMutexTryLock(OspRecursiveMutex_t *mutex)
{
    return _Mutex_recursive_Try_acquire((rtems_recursive_mutex *)mutex);
}

/**
 * @brief   对递归锁进行解锁，加锁几次就需要解锁几次
 * @param   mutex [in/out], 递归锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospRecursiveMutexUnlock(OspRecursiveMutex_t *mutex)
{
    rtems_recursive_mutex_unlock((rtems_recursive_mutex *)mutex);
}

/**
 * @brief   销毁递归锁
 * @param   mutex [in/out], 递归锁句柄
 * @return  执行结果
 * @warning 无
 * @note    无
 */
void ospRecursiveMutexDestroy(OspRecursiveMutex_t *mutex)
{
    rtems_recursive_mutex_destroy((rtems_recursive_mutex *)mutex);
}
