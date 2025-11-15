/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file verify.h
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/08/01
 * @brief 常见校验算法
 */

#ifndef __VERIFY_H__
#define __VERIFY_H__

#include "common_defines.h"

U16 crc16(const U8 *data, U32 length);
U16 checksum16(const U8 *data, U32 length);

#endif /* __VERIFY_H__ */