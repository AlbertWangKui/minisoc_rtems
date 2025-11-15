/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co., Ltd.
 * @file osp_os_fatal.h
 * @brief os fatal notify
 * @author tianye
 * @version
 * @date 2024-05-08
 */

#ifndef __OSP_OS_FATAL_H__
#define __OSP_OS_FATAL_H__

#include <osp_status.h>

#ifdef __cplusplus
extern "C" {
#endif

///< 通知函数指针定义
typedef void (*OspOsFatalNotify)(void);

/**
 * @file osp_os_fatal.h
 * @brief 业务注册os fatal回调
 * @author tianye
 * @version
 * @date 2024-05-08
 */
OspStatusCode_e ospOsFatalNotifyFuncReg(OspOsFatalNotify func);

#ifdef __cplusplus
}
#endif

#endif
