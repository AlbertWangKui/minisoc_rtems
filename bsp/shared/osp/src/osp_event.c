/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_event.c
 * @author  pengqianheng
 * @date    2020.08.29
 * @brief   封装rtems操作系统事件标志组对外接口
 * @note    NA
 */

#include <osp_options.h>
#include <osp_event.h>
#include <rtems.h>
#include <rtems/rtems/status.h>
#include "include/osp_inner_common.h"

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
OspStatusCode_e ospEventReceive(OspEventSet eventIn, OspOption optionSet, OspInterval ticks, OspEventSet *eventOut)
{
    rtems_status_code result;
    OspStatusCode_e ospResult;

    result    = rtems_event_receive(eventIn, optionSet, ticks, eventOut);
    ospResult = ospConvertReturnValue(result);

    return ospResult;
}

/**
 * @brief   发送事件标志组
 * @param   id ,     任务的ID
 * @param   eventIn ,需要发送的事件标志组
 * @return  执行结果
 * @warning 无
 * @note    无
 */
OspStatusCode_e ospEventSend(OspID id, OspEventSet eventIn)
{
    rtems_status_code ret;
    OspStatusCode_e ospResult;

    ret       = rtems_event_send(id, eventIn);
    ospResult = ospConvertReturnValue(ret);

    return ospResult;
}
