/**
 * copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file stringto_wrapper.h
 * @author taohb@starsmicrosystem.com
 * @date 2025/05/21
 * @brief wrapper for strtoxxx, for example strtol/strtoul.
 */

#ifndef __STRINGTO_WRAPPER_H__
#define __STRINGTO_WRAPPER_H__

#include "common_defines.h"

/**
 * @brief convert string to long int.
 * @param [in] str source string.
 * @param [out] out out long int value.
 * @return EXIT_SUCCESS or EXIT_FAILURE
 */
S32 str2L(const S8 *str, S32 *out);

/**
 * @brief convert string to unsigned long int.
 * @param [in] str source string.
 * @param [out] out out unsigned int value.
 * @return EXIT_SUCCESS or EXIT_FAILURE
 */
S32 str2Ul(const S8 *str, U32 *out);

#endif // __STRINGTO_WRAPPER_H__
