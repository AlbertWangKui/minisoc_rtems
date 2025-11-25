/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_timer_api.h
 * @author zuomeng1@starsmicrosystem.com
 * @date 2025/08/09
 * @brief Timer driver API definitions
 */

#ifndef __DRV_TIMER_API_H__
#define __DRV_TIMER_API_H__

#include "common_defines.h"
#include "bsp_device.h"

/**
 * @brief Initialize the timer controller
 * @details Initialize the specified timer controller, install interrupt handler, and configure timer settings
 * @param [in] devId Timer controller ID (DEVICE_TIMER0, DEVICE_TIMER1, etc.)
 * @return EXIT_SUCCESS on success
 * @return -EINVAL Invalid parameter (device not match or system timer)
 * @return -EIO I/O error (failed to enable/reset peripheral, get device config, install IRQ handler, or set timer config)
 * @return -EBUSY Device busy (device lock failed or already initialized)
 * @return -ENOMEM Out of memory (failed to allocate driver data)
 */
S32 timerInit(DevList_e devId);

/**
 * @brief Deinitialize the timer controller
 * @details Deinitialize the specified timer controller, remove interrupt handler, and reset peripheral
 * @param [in] devId Timer controller ID (DEVICE_TIMER0, DEVICE_TIMER1, etc.)
 * @return EXIT_SUCCESS on success
 * @return -EINVAL Invalid parameter (device not match, not initialized, or system timer)
 * @return -EIO I/O error (failed to get device driver, remove IRQ handler, reset peripheral, or uninstall driver)
 * @return -EBUSY Device busy (device lock failed)
 */
S32 timerDeInit(DevList_e devId);

/**
 * @brief Set timer configuration
 * @details Configure timer with specified milliseconds and timeout callback function
 * @param [in] devId Timer controller ID (DEVICE_TIMER0, DEVICE_TIMER1, etc.)
 * @param [in] mSec Timer interval in milliseconds
 * @param [in] timeout Callback function for timeout interrupt, NULL to disable interrupt
 * @return EXIT_SUCCESS on success
 * @return -EINVAL Invalid parameter (device not match, not initialized, system timer, or failed to calculate ticks)
 * @return -EIO I/O error (failed to get device driver or invalid register address)
 * @return -EBUSY Device busy (device lock failed)
 */
S32 timerSetCfg(DevList_e devId, U32 mSec, void (*timeout)(void));

/**
 * @brief Start timer
 * @details Start the timer counting
 * @param [in] devId Timer controller ID (DEVICE_TIMER0, DEVICE_TIMER1, etc.)
 * @return EXIT_SUCCESS on success
 * @return -EINVAL Invalid parameter (device not match, not initialized, or system timer)
 * @return -EIO I/O error (failed to get device driver or invalid register address)
 * @return -EBUSY Device busy (device lock failed)
 */
S32 timerStart(DevList_e devId);

/**
 * @brief Stop timer
 * @details Stop the timer counting
 * @param [in] devId Timer controller ID (DEVICE_TIMER0, DEVICE_TIMER1, etc.)
 * @return EXIT_SUCCESS on success
 * @return -EINVAL Invalid parameter (device not match, not initialized, or system timer)
 * @return -EIO I/O error (failed to get device driver or invalid register address)
 * @return -EBUSY Device busy (device lock failed)
 */
S32 timerStop(DevList_e devId);

#endif /* __DRV_TIMER_API_H__ */
