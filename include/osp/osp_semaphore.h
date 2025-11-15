/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_semaphore.h
 * @author  lichenxiang
 * @date    2020.08.04
 * @brief   porting rtems semaphore
 * @note    NA
 */

#ifndef __OSP_SEMAPHORE_H
#define __OSP_SEMAPHORE_H

#include <osp_attr.h>
#include <osp_options.h>
#include <osp_status.h>
#include <osp_types.h>
#include <osp_priority.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   创建信号量
 * @param   name ,            信号量名称
 * @param   count ,           初始的信号量计数值
 * @param   attributeSet ,    信号量属性选项，参考 OSP_DEFAULT_ATTRIBUTES
 * @param   priorityCeiling , 优先级天花板
 * @param   id [out],         RTEMS内核分配的信号量ID
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospSemaphoreCreate(OspName name, uint32_t count, OspAttribute attributeSet,
                                   OspTaskPriority_e priorityCeiling, OspID *id);

/**
 * @brief   删除信号量
 * @param   id , 信号量ID
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospSemaphoreDelete(OspID id);

/**
 * @brief   获取信号量
 * @param   id ,        信号量ID
 * @param   optionSet , 选择是否要等待，参考 osp_options.h
 * @param   timeout ,   最大等待(阻塞)时间
 * @return  执行结果
 * @warning 无
 * @note    如果选择 optionSet 为 OSP_NO_WAIT ，那么本接口会立即返回
 */
OspStatusCode_e ospSemaphoreObtain(OspID id, OspOption optionSet, OspInterval timeout);

/**
 * @brief   释放信号量
 * @param   id , 信号量ID
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospSemaphoreRelease(OspID id);

#ifdef __cplusplus
}
#endif

#endif
