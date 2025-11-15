/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_queue.h
 * @author  lichenxiang
 * @date    2020.08.04
 * @brief   porting rtems queue
 * @note    NA
 */

#ifndef __OSP_QUEUE_H
#define __OSP_QUEUE_H

#include <osp_attr.h>
#include <osp_options.h>
#include <osp_status.h>
#include <osp_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief   创建消息队列
* @param   name ,           队列名称
* @param   count ,          队列支持的消息数量
* @param   maxMessageSize , 消息的最大尺寸
* @param   attributeSet ,   队列属性，参考 OSP_DEFAULT_ATTRIBUTES
           OSP_FIFO      接收消息的任务按 FIFO 等待消息
           OSP_PRIORITY  接收消息的任务按优先级等待消息
           OSP_LOCAL     本地消息队列，用于单核
           OSP_GLOBAL    全局消息队列，用于多核(需开启SMP)
* @param   id [out],        队列ID
* @return  执行结果
* @warning 无
* @note    无
*/
OspStatusCode_e ospMessageQueueCreate(OspName name, uint32_t count, size_t maxMessageSize, OspAttribute attributeSet,
                                      OspID *id);

/**
 * @brief   删除消息队列
 * @param   id , 消息队列ID
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospMessageQueueDelete(OspID id);

/**
 * @brief   清空消息队列
 * @param   id ,         消息队列ID
 * @param   count [out], 此次被删除的消息数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospMessageQueueFlush(OspID id, uint32_t *count);

/**
 * @brief   接收消息
 * @param   id ,           消息队列ID
 * @param   buffer [in],   消息的数据入口指针
 * @param   size [in/out], 消息的数据大小
 * @param   optionSet ,    选择是否要等待(阻塞)，参考 osp_options.h
 * @param   timeout ,      最大的等待(阻塞)时间
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospMessageQueueReceive(OspID id, void *buffer, size_t *size, OspOption optionSet, OspInterval timeout);

/**
 * @brief   发送消息(到队尾)
 * @param   id ,         消息队列ID
 * @param   buffer [in], 消息的数据入口指针
 * @param   size ,       消息的大小
 * @return  执行结果
 * @warning 无
 * @note    消息会被放置在队列的尾部
 */
OspStatusCode_e ospMessageQueueSend(OspID id, const void *buffer, size_t size);

/**
 * @brief   发送消息(到队首)
 * @param   id ,         消息队列ID
 * @param   buffer [in], 消息的数据入口指针
 * @param   size ,       消息的大小
 * @return  执行结果
 * @warning 无
 * @note    消息会被放置在队列的首部
 */
OspStatusCode_e ospMessageQueueUrgent(OspID id, const void *buffer, size_t size);

/**
 * @brief   广播发送消息
 * @param   id ,         消息队列ID
 * @param   buffer [in], 消息的数据入口指针
 * @param   size ,       消息的大小
 * @param   count [out], 被解除阻塞的任务数量
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospMessageQueueBroadcast(OspID id, const void *buffer, size_t size, uint32_t *count);

/**
 * @brief   获取消息队列中的消息数量
 * @param   id ,         消息队列ID
 * @param   count [out], 消息队列中待处理的消息数量
 * @return  执行结果
 * @warning 无
 * @note    NA
 */
OspStatusCode_e ospMessageQueueGetNumberPending(OspID id, uint32_t *count);

/**
 * @brief   根据消息队列名称获取消息队列id
 * @param   name ,         消息队列名称
 * @param   node ,         搜索的节点(OSP_SEARCH_ALL_NODES/OSP_SEARCH_LOCAL_NODES)
 * @param   *id [out],     消息队列的id
 * @return  执行结果
 * @warning 无
 * @note    若多个消息队列名称相同，则获取的是首个匹配id
 */
OspStatusCode_e ospMessageQueueIdent(OspName name, uint32_t node, OspID *id);
#ifdef __cplusplus
}
#endif
#endif
