/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_condition_variable.c
 * @author  tianye
 * @date    2021.07.01
 * @brief   封装rtems操作系统condition variable对外接口
 * @note    NA
 */

#include <osp_condition_variable.h>

/**
 * @brief   初始化条件变量
 * @param   condVar [in/out], 需要初始化的条件变量
 * @param   name [in],        条件变量的名字
 * @return  void
 * @warning 无
 * @note    无
 */
void ospConditionVariableInit(OspConditonVariable_t *condVar, const char *name)
{
    rtems_condition_variable_init((rtems_condition_variable *)condVar, name);
}

/**
 * @brief   获取给定条件变量句柄的名字
 * @param   condVar [in], 条件变量句柄
 * @return  返回condVar的名字
 * @warning 无
 * @note    无
 */
const char *ospConditionVariableGetName(const OspConditonVariable_t *condVar)
{
    return rtems_condition_variable_get_name((const rtems_condition_variable *)condVar);
}

/**
 * @brief   设置条件变量的名字
 * @param   condVar [in/out], 条件变量句柄
 * @param   name [in],      需要设置的名字
 * @return  void
 * @warning 无
 * @note    无
 */
void ospConditionVariableSetName(OspConditonVariable_t *condVar, const char *name)
{
    rtems_condition_variable_set_name((rtems_condition_variable *)condVar, name);
}

/**
 * @brief   等待条件变量
 * @param   condVar [in/out], 条件变量句柄
 * @param   mutex [in], mutex锁句柄
 * @return  void
 * @warning 无
 * @note    无
 */
void ospConditionVariableWait(OspConditonVariable_t *condVar, OspMutex_t *mutex)
{
    rtems_condition_variable_wait((rtems_condition_variable *)condVar, (rtems_mutex *)mutex);
}

/**
 * @brief   唤醒等待条件变量的任务
 * @param   condVar [in/out], 条件变量句柄
 * @return  void
 * @warning 无
 * @note    无
 */
void ospConditionVariableSignal(OspConditonVariable_t *condVar)
{
    rtems_condition_variable_signal((rtems_condition_variable *)condVar);
}

/**
 * @brief   广播方式唤醒等待条件变量的任务
 * @param   condVar [in/out], 条件变量句柄
 * @return  void
 * @warning 无
 * @note    无
 */
void ospConditionVariableBroadcast(OspConditonVariable_t *condVar)
{
    rtems_condition_variable_broadcast((rtems_condition_variable *)condVar);
}

/**
 * @brief   销毁条件变量
 * @param   condVar [in], 条件变量句柄
 * @return  void
 * @warning 无
 * @note    无
 */
void ospConditionVariableDestroy(OspConditonVariable_t *condVar)
{
    rtems_condition_variable_destroy((rtems_condition_variable *)condVar);
}
