/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_event.h
 * @author  lichenxiang
 * @date    2020.08.04
 * @brief   porting event
 * @note    NA
 */
#ifndef __OSP_EVENT_H
#define __OSP_EVENT_H
#include <osp_options.h>
#include <osp_status.h>
#include <osp_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  @brief Constant used to send or receive all events.
 */
#define OSP_ALL_EVENTS 0xFFFFFFFF

///< @brief Defines the bit in the event set associated with event 0.
#define OSP_EVENT_0 0x00000001
///< @brief Defines the bit in the event set associated with event 1.
#define OSP_EVENT_1 0x00000002
///< @brief Defines the bit in the event set associated with event 2.
#define OSP_EVENT_2 0x00000004
///< @brief Defines the bit in the event set associated with event 3.
#define OSP_EVENT_3 0x00000008
///< @brief Defines the bit in the event set associated with event 4.
#define OSP_EVENT_4 0x00000010
///< @brief Defines the bit in the event set associated with event 5.
#define OSP_EVENT_5 0x00000020
///< @brief Defines the bit in the event set associated with event 6.
#define OSP_EVENT_6 0x00000040
///< @brief Defines the bit in the event set associated with event 7.
#define OSP_EVENT_7 0x00000080
///< @brief Defines the bit in the event set associated with event 8.
#define OSP_EVENT_8 0x00000100
///< @brief Defines the bit in the event set associated with event 9.
#define OSP_EVENT_9 0x00000200
///< @brief Defines the bit in the event set associated with event 10.
#define OSP_EVENT_10 0x00000400
///< @brief Defines the bit in the event set associated with event 11.
#define OSP_EVENT_11 0x00000800
///< @brief Defines the bit in the event set associated with event 12.
#define OSP_EVENT_12 0x00001000
///< @brief Defines the bit in the event set associated with event 13.
#define OSP_EVENT_13 0x00002000
///< @brief Defines the bit in the event set associated with event 14.
#define OSP_EVENT_14 0x00004000
///< @brief Defines the bit in the event set associated with event 15.
#define OSP_EVENT_15 0x00008000
///< @brief Defines the bit in the event set associated with event 16.
#define OSP_EVENT_16 0x00010000
///< @brief Defines the bit in the event set associated with event 17.
#define OSP_EVENT_17 0x00020000
///< @brief Defines the bit in the event set associated with event 18.
#define OSP_EVENT_18 0x00040000
///< @brief Defines the bit in the event set associated with event 19.
#define OSP_EVENT_19 0x00080000
///< @brief Defines the bit in the event set associated with event 20.
#define OSP_EVENT_20 0x00100000
///< @brief Defines the bit in the event set associated with event 21.
#define OSP_EVENT_21 0x00200000
///< @brief Defines the bit in the event set associated with event 22.
#define OSP_EVENT_22 0x00400000
///< @brief Defines the bit in the event set associated with event 23.
#define OSP_EVENT_23 0x00800000
///< @brief Defines the bit in the event set associated with event 24.
#define OSP_EVENT_24 0x01000000
///< @brief Defines the bit in the event set associated with event 25.
#define OSP_EVENT_25 0x02000000
///< @brief Defines the bit in the event set associated with event 26.
#define OSP_EVENT_26 0x04000000
///< @brief Defines the bit in the event set associated with event 27.
#define OSP_EVENT_27 0x08000000
///< @brief Defines the bit in the event set associated with event 29.
#define OSP_EVENT_28 0x10000000
///< @brief Defines the bit in the event set associated with event 29.
#define OSP_EVENT_29 0x20000000
///< @brief Defines the bit in the event set associated with event 30.
#define OSP_EVENT_30 0x40000000
///< @brief Defines the bit in the event set associated with event 31.
#define OSP_EVENT_31 0x80000000

/**
 *  @brief Constant used to receive the set of currently pending events in
 *  ospEventReceive().
 */
#define OSP_PENDING_EVENTS 0

/**
 * @brief Reserved system event for network SBWAIT usage.
 */
#define OSP_EVENT_SYSTEM_NETWORK_SBWAIT OSP_EVENT_24

/**
 * @brief Reserved system event for network SOSLEEP usage.
 */
#define OSP_EVENT_SYSTEM_NETWORK_SOSLEEP OSP_EVENT_25

/**
 * @brief Reserved system event for network socket close.
 */
#define OSP_EVENT_SYSTEM_NETWORK_CLOSE OSP_EVENT_26

/**
 * @brief Reserved system event to resume server threads, e.g timer or
 * interrupt server.
 */
#define OSP_EVENT_SYSTEM_SERVER_RESUME OSP_EVENT_29

/**
 * @brief Reserved system event for the server threads, e.g timer or interrupt
 * server.
 */
#define OSP_EVENT_SYSTEM_SERVER OSP_EVENT_30

/**
 * @brief Reserved system event for transient usage.
 */
#define OSP_EVENT_SYSTEM_TRANSIENT OSP_EVENT_31

/**
 * @brief   接收事件标志组
 * @param   eventIn ,        待接收的事件标志组
 * @param   optionSet ,      是否要等待(阻塞)，参考osp_options.h
 * @param   ticks ,          等待时间
 * @param   eventOut [out],  已接收到的事件标志
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospEventReceive(OspEventSet eventIn, OspOption optionSet, OspInterval ticks, OspEventSet *eventOut);

/**
 * @brief   发送事件标志组
 * @param   id ,     任务的ID
 * @param   eventIn ,需要发送的事件标志组
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospEventSend(OspID id, OspEventSet eventIn);
#ifdef __cplusplus
}
#endif

#endif
