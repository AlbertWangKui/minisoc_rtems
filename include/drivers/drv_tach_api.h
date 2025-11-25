/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_tach_api.h
 * @author yangkl (yangkl@starsmicrosystem.com)
 * @date 2025
 * @brief  Tach driver API definitions
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2024         tach_driver     Created initial version
 */

#ifndef __DRV_TACH_API_H__
#define __DRV_TACH_API_H__

#include "common_defines.h"
#include "bsp_device.h"

/**
 * @brief Tachometer trigger polarity enumeration
 * @details Defines the trigger edge polarity options for tachometer frequency measurement
 */
typedef enum TachPolarity {
    TACH_POSITIVE_EDGE = 0,  ///< Trigger on rising edge of the input signal
    TACH_NEGATIVE_EDGE,      ///< Trigger on falling edge of the input signal
    TACH_POLARITY_MAX,       ///< Maximum polarity value for bounds checking
} TachPolarity_e;

/**
 * @brief Tachometer interrupt callback function type
 * @details Function type for tachometer interrupt callback
 * @param [in] arg User argument passed to callback function
 */
typedef void (*tachIrqCallBack)(void *arg);

/**
 * @brief Initialize the tachometer controller
 * @details Initialize the specified tachometer controller, install interrupt handler, and configure capture settings
 * @param [in] devId Controller ID
 * @return  EXIT_SUCCESS : Success
 *          -EINVAL : Invalid parameter (device not match)
 *          -EIO : IO error (failed to enable/reset peripheral, get clock frequency, get device config, install IRQ handler, set trigger or set capture config)
 *          -EBUSY : Device busy (device lock failed or already initialized)
 *          -ENOMEM : Out of memory (failed to allocate driver data)
 */
S32 tachInit(DevList_e devId);

/**
 * @brief Deinitialize the tachometer controller
 * @details Deinitialize the specified tachometer controller, disable capture, remove interrupt handler, and reset peripheral
 * @param [in] devId Controller ID
 * @return  EXIT_SUCCESS : Success
 *          -EINVAL : Invalid parameter (device not match or not initialized)
 *          -EIO : IO error (failed to get device driver, remove IRQ handler, reset peripheral, uninstall driver or disable peripheral clock)
 *          -EBUSY : Device busy (device lock failed)
 */
S32 tachDeInit(DevList_e devId);

/**
 * @brief Get tachometer frequency
 * @details Get the tachometer frequency measurement from the specified controller
 * @param [in] devId Controller ID
 * @param [in] timeoutSec Timeout in seconds (must be between 1 and TACH_MAX_TIMEOUT_VALUE_S)
 * @param [out] freqBuf Buffer to store the frequency value
 * @return  EXIT_SUCCESS : Success
 *          -EINVAL : Invalid parameter (freqBuf is NULL, timeoutSec out of range, device not initialized, device not enabled, or device not match)
 *          -EIO : IO error (failed to get device driver or invalid register)
 *          -EBUSY : Device busy (device lock failed or device is already measuring)
 *          -ENOMEM : Out of memory (failed to get driver data)
 *          -ETIMEDOUT : Operation timed out (frequency measurement timeout)
 */
S32 tachGetFreq(DevList_e devId, U32 timeoutSec, U32 *freqBuf);

/**
 * @brief Enable tachometer capture
 * @details Enable the tachometer capture functionality for the specified controller
 * @param [in] devId Controller ID
 * @return  EXIT_SUCCESS : Success
 *          -EINVAL : Invalid parameter (device not match or not initialized)
 *          -EIO : IO error (failed to get device driver or invalid register)
 *          -EBUSY : Device busy (device lock failed)
 *          -ENOMEM : Out of memory (failed to get driver data)
 */
S32 tachEnable(DevList_e devId);

/**
 * @brief Disable tachometer capture
 * @details Disable the tachometer capture functionality for the specified controller (idempotent operation)
 * @param [in] devId Controller ID
 * @return  EXIT_SUCCESS : Success
 *          -EINVAL : Invalid parameter (device not match or not initialized)
 *          -EIO : IO error (failed to get device driver or invalid register)
 *          -EBUSY : Device busy (device lock failed)
 *          -ENOMEM : Out of memory (failed to get driver data)
 */
S32 tachDisable(DevList_e devId);

/**
 * @brief Register tachometer interrupt callback
 * @details Register a callback function to be called when tachometer interrupt occurs
 * @param [in] devId Controller ID
 * @param [in] callback Callback function pointer
 * @param [in] arg Argument to pass to the callback function
 * @return  EXIT_SUCCESS : Success
 *          -EINVAL : Invalid parameter (callback is NULL, device not match, or callback already registered)
 *          -EIO : IO error (device not initialized, failed to get device driver, or invalid register)
 *          -EBUSY : Device busy (device lock failed)
 *          -ENOMEM : Out of memory (failed to get driver data)
 */
S32 tachCallbackRegister(DevList_e devId, tachIrqCallBack callback, void *arg);

/**
 * @brief Unregister tachometer interrupt callback
 * @details Unregister the previously registered callback function
 * @param [in] devId Controller ID
 * @return  EXIT_SUCCESS : Success
 *          -EINVAL : Invalid parameter (device not match or invalid register)
 *          -EIO : IO error (device not initialized, failed to get device driver)
 *          -EBUSY : Device busy (device lock failed)
 *          -ENOMEM : Out of memory (failed to get driver data)
 */
S32 tachCallbackUnRegister(DevList_e devId);

/**
 * @brief Enable tachometer interrupt
 * @details Enable the tachometer interrupt for the specified controller
 * @param [in] devId Controller ID
 * @return  EXIT_SUCCESS : Success
 *          -EINVAL : Invalid parameter (device not match)
 *          -EIO : IO error (device not initialized, failed to get device driver, or invalid register)
 *          -EBUSY : Device busy (device lock failed)
 */
S32 tachIrqEnable(DevList_e devId);

/**
 * @brief Disable tachometer interrupt
 * @details Disable the tachometer interrupt for the specified controller
 * @param [in] devId Controller ID
 * @return  EXIT_SUCCESS : Success
 *          -EINVAL : Invalid parameter (device not match)
 *          -EIO : IO error (device not initialized, failed to get device driver, or invalid register)
 *          -EBUSY : Device busy (device lock failed)
 */
S32 tachIrqDisable(DevList_e devId);

/**
 * @brief Set tachometer trigger polarity
 * @details Configure the trigger edge polarity for tachometer frequency measurement
 * @param [in] devId Controller ID
 * @param [in] polarity Trigger polarity (TACH_POSITIVE_EDGE for rising edge, TACH_NEGATIVE_EDGE for falling edge)
 * @return  EXIT_SUCCESS : Success
 *          -EINVAL : Invalid parameter (device not match or not initialized)
 *          -EIO : IO error (failed to get device driver or invalid register)
 *          -EBUSY : Device busy (device lock failed)
 */
S32 tachSetTrigger(DevList_e devId, TachPolarity_e polarity);
#endif /* __DRV_TACH_API_H__ */