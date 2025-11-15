/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_condition_variable.h
 * @author  tianye
 * @date    2021.07.01
 * @brief   porting rtems condition variable
 * @note    NA
 */
#ifndef __OSP_CONDITION_VARIABLE_H__
#define __OSP_CONDITION_VARIABLE_H__

#include <osp_status.h>
#include <osp_types.h>
#include <osp_mutex.h>
#include <inner/osp_inner_condition_variable.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OSP_CONDITION_VARIABLE_INITIALIZER(name) INNER_OSP_CONDITION_VARIABLE_INITIALIZER(name)

typedef InnerOspConditonVariable_t OspConditonVariable_t;

/**
 * @brief   初始化条件变量
 * @param   condVar [in/out], 需要初始化的条件变量
 * @param   name [in],        条件变量的名字
 * @return  void
 * @warning 无
 * @note    无
 */
void ospConditionVariableInit(OspConditonVariable_t *condVar, const char *name);

/**
 * @brief   获取给定条件变量句柄的名字
 * @param   condVar [in], 条件变量句柄
 * @return  返回condVar的名字
 * @warning 无
 * @note    无
 */
const char *ospConditionVariableGetName(const OspConditonVariable_t *condVar);

/**
 * @brief   设置条件变量的名字
 * @param   condVar [in/out], 条件变量句柄
 * @param   name [in],      需要设置的名字
 * @return  void
 * @warning 无
 * @note    无
 */
void ospConditionVariableSetName(OspConditonVariable_t *condVar, const char *name);

/**
 * @brief   等待条件变量
 * @param   condVar [in/out], 条件变量句柄
 * @param   mutex [in], mutex锁句柄
 * @return  void
 * @warning 无
 * @note    无
 */
void ospConditionVariableWait(OspConditonVariable_t *condVar, OspMutex_t *mutex);

/**
 * @brief   唤醒等待条件变量的任务
 * @param   condVar [in/out], 条件变量句柄
 * @return  void
 * @warning 无
 * @note    无
 */
void ospConditionVariableSignal(OspConditonVariable_t *condVar);

/**
 * @brief   广播方式唤醒等待条件变量的任务
 * @param   condVar [in/out], 条件变量句柄
 * @return  void
 * @warning 无
 * @note    无
 */
void ospConditionVariableBroadcast(OspConditonVariable_t *condVar);

/**
 * @brief   销毁件变量
 * @param   condVar [in], 条件变量句柄
 * @return  void
 * @warning 无
 * @note    无
 */
void ospConditionVariableDestroy(OspConditonVariable_t *condVar);

#ifdef __cplusplus
}
#endif

#endif
