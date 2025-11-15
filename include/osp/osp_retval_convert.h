
/**
 * Copyright (C), 2023, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_retval_convert.h
 * @author  tianye
 * @date    2023.09.11
 * @brief   return value convert
 * @note    NA
 */

#ifndef __OSP_RETVAL_CONVERT_H__
#define __OSP_RETVAL_CONVERT_H__

#include <osp_status.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief   转换osp错误码为int32_t
 * @param   OspStatusCode_e , 原始错误码
 * @return  转换为带符号的数（>=0表示成功,负数表示失败）
 * @warning 无
 * @note    当前os错误码成功值只有0
 */
static inline int32_t ospRetvalueConvert(OspStatusCode_e ret)
{
    return -ret;
}

#ifdef __cplusplus
}
#endif

#endif
