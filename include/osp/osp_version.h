/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_version.h
 * @author  lichenxiang
 * @date    2020.12.08
 * @brief   porting types
 * @note    NA
 */

#ifndef __OSP_VERSION_H_
#define __OSP_VERSION_H_

#ifdef __cplusplus
extern "C" {
#endif

#define OSP_VERSION "OSP_RELEASE_VERSION"

/**
 * @brief   获取OSP与内核版本号
 * @param   NA
 * @return  版本号
 * @warning NA
 * @note    NA
 */
static __inline const char *ospGetVersion( )
{
    return OSP_VERSION;
}

#ifdef __cplusplus
}
#endif
#endif
