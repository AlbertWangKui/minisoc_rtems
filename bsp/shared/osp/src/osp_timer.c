/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_timer.c
 * @author  weipeng
 * @date    2020.09.09
 * @brief   timer & clock相关封装接口
 * @note    NA
 */

#include <osp_timer.h>
#include <rtems.h>
#include <rtems/score/todimpl.h>
#include "include/osp_inner_common.h"

/*
 * timespec时间格式转换相关宏定义
 */
#define RTEMS_SECS_PER_MINUTE (60UL)
#define RTEMS_MINUTE_PER_HOUR (60UL)
#define RTEMS_SECS_PER_HOUR   (RTEMS_SECS_PER_MINUTE * RTEMS_MINUTE_PER_HOUR)
#define RTEMS_HOURS_PER_DAY   (24UL)
#define RTEMS_SECS_PER_DAY    (RTEMS_SECS_PER_HOUR * RTEMS_HOURS_PER_DAY)
#define RTEMS_DAYS_PER_YEAR   (365UL)
#define RTEMS_YEAR_BASE       (1970UL)

extern const uint16_t _TOD_Days_to_date[2][13];

/*
 * timespec时间格式转换为TOD格式子函数
 *
 */
static bool _Leap_year(
  uint32_t year
)
{
  return (((year % 4) == 0) && ((year % 100) != 0)) || ((year % 400) == 0);
}

static uint32_t _Leap_years_before(
  uint32_t year
)
{
  year -= 1;
  return (year / 4) - (year / 100) + (year / 400);
}

static uint32_t _Leap_years_between(
  uint32_t from, uint32_t to
)
{
  return _Leap_years_before( to ) - _Leap_years_before( from + 1 );
}

static uint32_t _Year_day_as_month(
  uint32_t year, uint32_t *day
)
{
  const uint16_t* days_to_date;
  uint32_t        month = 0;

  if ( _Leap_year( year ) )
    days_to_date = _TOD_Days_to_date[1];
  else
    days_to_date = _TOD_Days_to_date[0];

  days_to_date += 2;

  while (month < 11) {
    if (*day < *days_to_date)
      break;
    ++month;
    ++days_to_date;
  }

  *day -= *(days_to_date - 1);

  return month;
}


/**
 * @brief   获取系统启动以来的滴答数
 * @param   NA
 * @return  滴答数
 * @warning NA
 * @note    NA
 */
OspInterval ospClockGetTicksSinceBoot(void)
{
    return rtems_clock_get_ticks_since_boot( );
}

/**
 * @brief   按照标准格式struct timespec设置系统时间
 * @param   *tp [in], struct timespec格式的系统时间
 * @return  状态码
 * @warning NA
 * @note    由于rtems系统时基为19880101,设置时间早于该值将返回OSP_INVALID_NUMBER
 */
OspStatusCode_e ospClockSetTimespec(const struct timespec *tp)
{
    OspStatusCode_e ret;
    Status_Control status;
    ISR_lock_Context lock_context;

    if (tp == NULL) {
        ret = OSP_INVALID_ADDRESS;
        goto end;
    }

    _TOD_Lock();
    _TOD_Acquire(&lock_context);
    status = _TOD_Set(tp, &lock_context);
    _TOD_Unlock();

    ret = (OspStatusCode_e)STATUS_GET_CLASSIC(status);

end:
    return ret;
}

/**
 * @brief   按照标准格式struct timespec获取系统时间
 * @param   *tp [out], struct timespec格式的系统时间
 * @return  状态码
 * @warning NA
 * @note    rtems时基为19880101 00-00-00
 */
OspStatusCode_e ospClockGetTimespec(struct timespec *tp)
{
    OspStatusCode_e ret;

    if (tp == NULL) {
        ret = OSP_INVALID_ADDRESS;
        goto end;
    }

    _TOD_Get(tp);
    ret = OSP_SUCCESSFUL;

end:
    return ret;
}
/**
 * @brief   设置挂钟时间
 * @param   tod [in],  万年历格式时间
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospClockSet(const OspTimeOfDay_s *tod)
{
    return ospConvertReturnValue(rtems_clock_set((rtems_time_of_day *)tod));
}

/**
 * @brief   获取挂钟时间
 * @param   tod [in],  万年历格式时间
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospClockGetTod(const OspTimeOfDay_s *tod)
{
    return ospConvertReturnValue(rtems_clock_get_tod((rtems_time_of_day *)tod));
}

/**
 * @brief   获取系统运行时间(ns)
 * @param   uptime [out], 系统运行时间,自动后系统经过的秒和纳秒,struct timespec
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospClockGetUptime(struct timespec *uptime)
{
    return ospConvertReturnValue(rtems_clock_get_uptime(uptime));
}

/**
 * @brief   获取系统运行时间(us)
 * @param   uptime [out], 系统运行时间,自动后系统经过的秒和微秒,struct timeval
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospClockGetUptimeTimeval(struct timeval *uptime)
{
    OspStatusCode_e ret;

    if (uptime == NULL) {
        ret = OSP_INVALID_ADDRESS;
        goto exit;
    }

    rtems_clock_get_uptime_timeval(uptime);
    ret = OSP_SUCCESSFUL;
exit:
    return ret;
}

/**
 * @brief   获取系统运行时间(ns)
 * @param   nano [out], 系统运行时间,自系统启动后系统经过纳秒,uint64_t
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospClockGetUptimeNanoseconds(uint64_t *nano)
{
    OspStatusCode_e ret;

    if (nano == NULL) {
        ret = OSP_INVALID_ADDRESS;
        goto exit;
    }

    *nano = rtems_clock_get_uptime_nanoseconds( );
    ret = OSP_SUCCESSFUL;
exit:
    return ret;
}

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
OspStatusCode_e ospTimerFireAfter(OspID id, OspInterval ticks, ospTimerServiceRoutineEntry routine, void *userData)
{
    return ospConvertReturnValue(
        rtems_timer_fire_after(id, ticks, (rtems_timer_service_routine_entry)routine, userData));
}

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
                                 void *userData)
{
    return ospConvertReturnValue(
        rtems_timer_fire_when(id, (rtems_time_of_day *)wallTime, (rtems_timer_service_routine_entry)routine, userData));
}

/**
 * @brief   创建定时器
 * @param   name ,    timer的名字
 * @param   id [out], timer的id
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimerCreate(OspName name, OspID *id)
{
    return ospConvertReturnValue(rtems_timer_create(name, id));
}

/**
 * @brief   取消计时
 * @param   id ,           timer的id
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimerCancel(OspID id)
{
    return ospConvertReturnValue(rtems_timer_cancel(id));
}

/**
 * @brief   销毁定时器实例
 * @param   id ,           timer的id
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimerDelete(OspID id)
{
    return ospConvertReturnValue(rtems_timer_delete(id));
}

/**
 * @brief   重置定时器
 * @param   id ,           timer的id
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimerReset(OspID id)
{
    return ospConvertReturnValue(rtems_timer_reset(id));
}

/**
 * @brief   查询timer基本信息
 * @param   id ,                timer的id
 * @param   theInfo [out] ,     timer基本信息
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimerGetInformation(OspID id, OspTimerInformation_s *theInfo)
{
    return ospConvertReturnValue(rtems_timer_get_information(id, (rtems_timer_information *)theInfo));
}

/**
 * @brief   OspTimeofDay_s时间格式转timespec格式
 * @param   ospTOD ,                要转换的TOD时间
 * @param   ospTimeSpec [out] ,     转换后的timespec时间
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTODToTimeSpec(const OspTimeOfDay_s *ospTOD, struct timespec *ospTimeSpec)
{
    if ( !ospTOD || !ospTimeSpec)
        goto invalid_address;

    if ( _TOD_Validate( (const rtems_time_of_day *)ospTOD ) ) {
        ospTimeSpec->tv_sec = _TOD_To_seconds( (const rtems_time_of_day *)ospTOD );
        ospTimeSpec->tv_nsec = ospTOD->ticks
        * rtems_configuration_get_nanoseconds_per_tick();
    }
    else
        goto invalid_clock;

    return OSP_SUCCESSFUL;
invalid_address:
    return OSP_INVALID_ADDRESS;
invalid_clock:
    return OSP_INVALID_CLOCK;
}


/**
 * @brief   timespec时间格式转TOD格式
 * @param   ospTimeSpec ,     要转换的timespec时间
 * @param   ospTOD ,[out]     转换后的TOD时间
 * @return  状态码
 * @warning NA
 * @note    NA
 */
OspStatusCode_e ospTimeSpecToTOD(const struct timespec *ospTimeSpec, OspTimeOfDay_s *ospTOD)
{
  uint32_t       days;
  uint32_t       day_secs;
  uint32_t       year;
  uint32_t       year_days;
  uint32_t       leap_years;

  if ( !ospTimeSpec || !ospTOD )
    goto invalid_address;


  /* How many days and how many seconds in the day ? */
  days = ospTimeSpec->tv_sec / RTEMS_SECS_PER_DAY;
  day_secs = ospTimeSpec->tv_sec % RTEMS_SECS_PER_DAY;

  /* How many non-leap year years ? */
  year = ( days / RTEMS_DAYS_PER_YEAR ) + RTEMS_YEAR_BASE;

  /* Determine the number of leap years. */
  leap_years = _Leap_years_between( RTEMS_YEAR_BASE, year );

  /* Adjust the remaining number of days based on the leap years. */
  year_days = ( days - leap_years ) % RTEMS_DAYS_PER_YEAR;

  /* Adjust the year and days in the year if in the leap year overflow. */
  if ( leap_years > ( days % RTEMS_DAYS_PER_YEAR ) ) {
    year -= 1;
    if ( _Leap_year( year ) ) {
      year_days += 1;
    }
  }

  ospTOD->year   = year;
  ospTOD->month  = _Year_day_as_month( year, &year_days ) + 1;
  ospTOD->day    = year_days + 1;
  ospTOD->hour   = day_secs / RTEMS_SECS_PER_HOUR;
  ospTOD->minute = day_secs % RTEMS_SECS_PER_HOUR;
  ospTOD->second = ospTOD->minute % RTEMS_SECS_PER_MINUTE;
  ospTOD->minute = ospTOD->minute / RTEMS_SECS_PER_MINUTE;
  ospTOD->ticks  = ospTimeSpec->tv_nsec /
    rtems_configuration_get_nanoseconds_per_tick( );

  return OSP_SUCCESSFUL;

invalid_address:
  return OSP_INVALID_ADDRESS;
}
