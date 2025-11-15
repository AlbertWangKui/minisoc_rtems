/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_timer.h
 * @author  weipeng
 * @date    2020.09.09
 * @brief   timer & clock相关封装接口
 * @note    NA
 */

#ifndef __OSP_TIMER_H
#define __OSP_TIMER_H

#include <sys/time.h>
#include <osp_status.h>
#include <osp_types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ospTimerServiceRoutineEntry)(OspID, void *);

#define OSP_TIMER_CLASS_BIT_TIME_OF_DAY 0x1
#define OSP_TIMER_CLASS_BIT_ON_TASK 0x2
#define OSP_TIMER_CLASS_BIT_NOT_DORMANT 0x4
typedef enum OspTimerClasses {
  /**
   * This value indicates the timer is currently not in use.
   */
  OSP_TIMER_DORMANT,
  /**
   * This value indicates the timer is currently in use as an interval
   * timer which will fire in the clock tick ISR.
   */
  OSP_TIMER_INTERVAL = OSP_TIMER_CLASS_BIT_NOT_DORMANT,
  /**
   * This value indicates the timer is currently in use as an interval
   * timer which will fire in the timer server task.
   */
  OSP_TIMER_INTERVAL_ON_TASK =
    OSP_TIMER_CLASS_BIT_NOT_DORMANT | OSP_TIMER_CLASS_BIT_ON_TASK,
  /**
   * This value indicates the timer is currently in use as an time of day
   * timer which will fire in the clock tick ISR.
   */
  OSP_TIMER_TIME_OF_DAY =
    OSP_TIMER_CLASS_BIT_NOT_DORMANT | OSP_TIMER_CLASS_BIT_TIME_OF_DAY,
  /**
   * This value indicates the timer is currently in use as an time of day
   * timer which will fire in the timer server task.
   */
  OSP_TIMER_TIME_OF_DAY_ON_TASK =
    OSP_TIMER_CLASS_BIT_NOT_DORMANT | OSP_TIMER_CLASS_BIT_TIME_OF_DAY |
    OSP_TIMER_CLASS_BIT_ON_TASK
} OspTimerClasses_s;

/**
 *  This is the structure filled in by the timer get information
 *  service.
 */
typedef struct OspTimerInformation {
    /** This indicates the current type of the timer. */
    OspTimerClasses_s   the_class;
    /** This indicates the initial requested interval. */
    OspInterval         initial;
    /** This indicates the time the timer was initially scheduled. */
    OspInterval         start_time;
    /** This indicates the time the timer is scheduled to fire. */
    OspInterval         stop_time;
} OspTimerInformation_s;

/**
 * @brief   获取系统启动以来的滴答数
 * @param   NA
 * @return  滴答数
 * @warning NA
 * @note    NA
 */
OspInterval ospClockGetTicksSinceBoot(void);

/**
 * @brief   按照标准格式struct timespec设置系统时间
 * @param   *tp [in], struct timespec格式的系统时间
 * @return  状态码
 * @warning NA
 * @note    由于rtems系统时基为19880101,设置时间早于该值将返回OSP_INVALID_NUMBER
 */
OspStatusCode_e ospClockSetTimespec(const struct timespec *tp);

/**
 * @brief   按照标准格式struct timespec获取系统时间
 * @param   *tp [out], struct timespec格式的系统时间
 * @return  状态码
 * @warning NA
 * @note    rtems时基为19880101 00-00-00
 */
OspStatusCode_e ospClockGetTimespec(struct timespec *tp);

/**
 * @brief   设置挂钟时间
 * @param   tod [in],  万年历格式时间
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospClockSet(const OspTimeOfDay_s *tod);

/**
 * @brief   获取挂钟时间
 * @param   tod [in],  万年历格式时间
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospClockGetTod(const OspTimeOfDay_s *tod);

/**
 * @brief   获取系统运行时间
 * @param   uptime [out], 系统运行时间,自系统启动后系统经过的秒和纳秒,struct timespec
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospClockGetUptime(struct timespec *uptime);

/**
 * @brief   获取系统运行时间(us)
 * @param   uptime [out], 系统运行时间,自系统启动后系统经过的秒和微秒,struct timeval
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospClockGetUptimeTimeval(struct timeval *uptime);

/**
 * @brief   获取系统运行时间(ns)
 * @param   nano [out], 系统运行时间,自系统启动后系统经过纳秒,uint64_t
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospClockGetUptimeNanoseconds(uint64_t *nano);

/**
 * @brief   触发定时器
 * @param   id ,           timer的id
 * @param   ticks ,        ticks以后触发回调
 * @param   routine ,      定时器溢出回调
 * @param   userData [in], 传给回调的数据
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimerFireAfter(OspID id, OspInterval ticks, ospTimerServiceRoutineEntry routine, void *userData);

/**
 * @brief   触发定时器
 * @param   id ,           timer的id
 * @param   wallTime [in], 挂钟时间
 * @param   routine ,      定时器溢出回调
 * @param   userData [in], 传给回调的数据
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimerFireWhen(OspID id, OspTimeOfDay_s *wallTime, ospTimerServiceRoutineEntry routine,
                                 void *userData);

/**
 * @brief   创建定时器
 * @param   name ,    timer的名字
 * @param   id [out], timer的id
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimerCreate(OspName name, OspID *id);

/**
 * @brief   取消计时
 * @param   id ,           timer的id
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimerCancel(OspID id);

/**
 * @brief   销毁定时器实例
 * @param   id ,           timer的id
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimerDelete(OspID id);

/**
 * @brief   重置定时器
 * @param   id ,           timer的id
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimerReset(OspID id);

/**
 * @brief   查询timer基本信息
 * @param   id ,                timer的id
 * @param   theInfo [out] ,     timer基本信息
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimerGetInformation(OspID id, OspTimerInformation_s *theInfo);

/**
 * @brief   OspTimeofDay_s时间格式转timespec格式
 * @param   ospTOD ,                要转换的TOD时间
 * @param   ospTimeSpec [out] ,     转换后的timespec时间
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTODToTimeSpec(const OspTimeOfDay_s *ospTOD, struct timespec *ospTimeSpec);

/**
 * @brief   timespec时间格式转TOD格式
 * @param   ospTimeSpec ,     要转换的timespec时间
 * @param   ospTOD ,[out]     转换后的TOD时间
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimeSpecToTOD(const struct timespec *ospTimeSpec, OspTimeOfDay_s *ospTOD);
#ifdef __cplusplus
}
#endif

#endif
