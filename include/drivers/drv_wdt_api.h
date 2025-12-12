/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_wdt_api.h
 * @author yangkl (yangkl@starsmicrosystem.com)
 * @date 2025/10/22
 * @brief watchdog driver API for minisoc
 */

#ifndef __DRV_WDT_API_H__
#define __DRV_WDT_API_H__

#include "common_defines.h"
#include "bsp_device.h"

/**
 * @brief Watchdog work mode enumeration
 */
typedef enum {
    WDT_MODE_RESET = 0,    ///< Reset mode: generate system reset on timeout
    WDT_MODE_INT,          ///< Interrupt mode: generate interrupt on timeout
    WDT_MODE_NR,           ///< Number of watchdog modes
} WorkMode_e;

/**
 * @brief Watchdog callback function type
 */
typedef void (*pWdtCallback)(void);

/**
 * @brief Initialize the watchdog driver
 * @param[in] devId Device ID to initialize
 * @param[in] callback Optional callback function for watchdog interrupt
 * @return EXIT_SUCCESS on success
 * @return -EBUSY if device is already initialized or lock acquisition fails
 * @return -EINVAL if driver ID doesn't match
 * @return -EIO if peripheral operations fail
 * @return -ENOMEM if memory allocation fails
 */
S32 wdtInit(DevList_e devId, pWdtCallback callback);

/**
 * @brief Deinitialize the watchdog driver
 * @param[in] devId Device ID to deinitialize
 * @return EXIT_SUCCESS on success
 * @return -EBUSY if device is not initialized or lock acquisition fails
 * @return -EINVAL if driver ID doesn't match
 * @return -EIO if peripheral operations fail
 */
S32 wdtDeInit(DevList_e devId);

/**
 * @brief Start the watchdog timer
 * @param[in] devId Device ID to start
 * @return EXIT_SUCCESS on success
 * @return -EBUSY if device is not initialized or lock acquisition fails
 * @return -EINVAL if driver ID doesn't match or work mode is invalid
 * @return -EIO if peripheral operations fail
 */
S32 wdtStart(DevList_e devId);

/**
 * @brief Stop the watchdog timer
 * @param[in] devId Device ID to stop
 * @return EXIT_SUCCESS on success
 * @return -EBUSY if device is not initialized or lock acquisition fails
 * @return -EINVAL if driver ID doesn't match
 * @return -EIO if peripheral operations fail
 */
S32 wdtStop(DevList_e devId);

/**
 * @brief Feed the watchdog timer
 * @param[in] devId Device ID to feed
 * @return EXIT_SUCCESS on success
 * @return -EBUSY if device is not initialized or lock acquisition fails
 * @return -EINVAL if driver ID doesn't match
 * @return -EIO if peripheral operations fail
 */
S32 wdtFeed(DevList_e devId);

/**
 * @brief Clear the watchdog interrupt
 * @param[in] devId Device ID to clear interrupt for
 * @return EXIT_SUCCESS on success
 * @return -EBUSY if device is not initialized or lock acquisition fails
 * @return -EINVAL if driver ID doesn't match
 * @return -EIO if peripheral operations fail
 */
S32 wdtIrqClr(DevList_e devId);

/**
 * @brief Set the watchdog timeout value
 * @param[in] devId Device ID to set timeout for
 * @param[in] timeOutsMs Timeout value in milliseconds
 * @return EXIT_SUCCESS on success
 * @return -EBUSY if device is not initialized or lock acquisition fails
 * @return -EINVAL if driver ID doesn't match or timeout value is invalid
 * @return -EIO if peripheral operations fail
 */
S32 wdtSetTimeout(DevList_e devId, U32 timeOutsMs);

/**
 * @brief Set the watchdog work mode
 * @param[in] devId Device ID to set mode for
 * @param[in] mode Work mode to set (WDT_MODE_RESET or WDT_MODE_INT)
 * @return EXIT_SUCCESS on success
 * @return -EBUSY if device is not initialized or lock acquisition fails
 * @return -EINVAL if driver ID doesn't match or mode is invalid
 * @return -EIO if peripheral operations fail
 */
S32 wdtSetMode(DevList_e devId, WorkMode_e mode);

#endif /* __DRV_WDT_API_H__ */
