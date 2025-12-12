/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_trng_api.h
 * @author yangzhl3 (yangzhl3@starsmicrosystem.com)
 * @date 2025/05/19
 * @brief TRNG driver public API
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/05/19   yangzhl3        the first version
 * 2025/09/27   dingwei         Add TH Trng Driver FW Version
 * 2025/11/26   dingwei         Refactor with funcRunBeginHelper
 *
 */

#ifndef __DRV_TRNG_API_H__
#define __DRV_TRNG_API_H__

#include "common_defines.h"
#include "bsp_device.h"

/**
 * @brief Initialize TRNG driver
 * @details Read SBR configuration, allocate driver resources and install interrupt.
 *          After initialization, random data can be read.
 * @param [in] devId Target device ID
 * @return EXIT_SUCCESS on success
 * @return -EINVAL if device mismatch or invalid parameter
 * @return -EBUSY if device already initialized or lock acquisition failed
 * @return -EIO if hardware or installation failed
 * @return -ENOMEM if memory allocation failed
 * @note Blocking call, must not be called from interrupt context
 */
S32 trngInit(DevList_e devId);

/**
 * @brief Deinitialize TRNG driver
 * @details Remove interrupt, reset peripheral and uninstall driver, release all private data
 * @param [in] devId Target device ID
 * @return EXIT_SUCCESS on success
 * @return -EINVAL if device mismatch or not initialized
 * @return -EBUSY if lock acquisition failed
 * @return -EIO if uninstall or hardware operation failed
 * @note Ensure device is not in use before calling this function
 */
S32 trngDeInit(DevList_e devId);

/**
 * @brief Generate random data
 * @details Read len 32-bit random words and write to caller's buffer
 * @param [in] devId Target device ID
 * @param [out] randomOut Output buffer pointer, must accommodate at least len U32 words
 * @param [in] len Number of 32-bit words to read
 * @return EXIT_SUCCESS on success
 * @return -EINVAL if parameter error
 * @return -EBUSY if driver lock unavailable
 * @return -EIO if hardware timeout or read failed
 * @note Function blocks polling hardware, must not be called from interrupt context
 */
S32 trngRandGen(DevList_e devId, U32 *randomOut, U32 len);

#endif /* __DRV_TRNG_API_H__ */