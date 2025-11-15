/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_common.c
 * @author  tianye
 * @date    2020.09.03
 * @brief   通用接口
 * @note    NA
 */

#include <osp_types.h>
#include <osp_common.h>
#include <osp_status.h>
#include <rtems/rtems/object.h>
#include <rtems/rtems/clock.h>
#include <rtems/score/objectimpl.h>

static const char *const gStatusCodeText[] = {
    "OSP_SUCCESSFUL",
    "OSP_TASK_EXITTED",
    "OSP_MP_NOT_CONFIGURED",
    "OSP_INVALID_NAME",
    "OSP_INVALID_ID",
    "OSP_TOO_MANY",
    "OSP_TIMEOUT",
    "OSP_OBJECT_WAS_DELETED",
    "OSP_INVALID_SIZE",
    "OSP_INVALID_ADDRESS",
    "OSP_INVALID_NUMBER",
    "OSP_NOT_DEFINED",
    "OSP_RESOURCE_IN_USE",
    "OSP_UNSATISFIED",
    "OSP_INCORRECT_STATE",
    "OSP_ALREADY_SUSPENDED",
    "OSP_ILLEGAL_ON_SELF",
    "OSP_ILLEGAL_ON_REMOTE_OBJECT",
    "OSP_CALLED_FROM_ISR",
    "OSP_INVALID_PRIORITY",
    "OSP_INVALID_CLOCK",
    "OSP_INVALID_NODE",
    "OSP_NOT_CONFIGURED",
    "OSP_NOT_OWNER_OF_RESOURCE",
    "OSP_NOT_IMPLEMENTED",
    "OSP_INTERNAL_ERROR",
    "OSP_NO_MEMORY",
    "OSP_IO_ERROR",
    "OSP_INTERRUPTED",
    "OSP_PROXY_BLOCKING"
};

/**
 * @brief   osp状态码转字符串接口
 * @param   OspStatusCode_e [in], 状态码
 * @return  const char*：转换后的状态码字符串
 * @warning NA
 * @note    NA
 */
const char *ospStatusText(OspStatusCode_e code)
{
  size_t i = code;
  const char *text = "?";

  if ( i < sizeof(gStatusCodeText) / sizeof(gStatusCodeText[0])) {
    text = gStatusCodeText[i];
  }
  return text;
}

/**
 * @brief   封装build name接口
 * @param   char1 [in],   第一个字符
 * @param   char2 [in],   第二个字符
 * @param   char3 [in],   第三个字符
 * @param   char4 [in],   第四个字符
 * @return  OspName： 转换后的OspName
 * @warning NA
 * @note    NA
 */
OspName ospBuildName(char char1, char char2, char char3, char char4)
{
    return (OspName)rtems_build_name(char1, char2, char3, char4); /*lint !e571 */
}

/**
 * @brief   osp name转换为字符串
 * @param   OspName [in], osp name,U32，每8位表示一个字符
 * @param   stringName [out], 转换为字符串形式buffer空间，4个字符加1个结束符
 * @param   size [in], 长度为4个字符加1个结束符
 * @return  OspName： 转换后的os适配层name
 * @warning NA
 * @note    buffer size固定为OSP_NAME_LEN
 */
OspStatusCode_e ospNameToStringName(OspName ospName, char *stringName, uint32_t size)
{
    (void)_Objects_Name_to_string((Objects_Name)ospName, false, stringName, size); /*lint !e69 */
    return OSP_SUCCESSFUL;
}

/**
 * @brief   把1秒换算为系统滴答计数
 * @param   NA
 * @return  OspInterval 系统滴答(tick)计数
 * @warning NA
 * @note    NA
 */
OspInterval ospClockGetTicksPerSecond(void)
{
    return (OspInterval)rtems_clock_get_ticks_per_second( );
}
