/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file    drv_ocm_ecc_api.h
 * @author  zhangxin3@starsmicrosystem.com
 * @date    2025/09/22
 * @brief   OCM ECC driver API definitions
 */

#ifndef __DRV_OCM_ECC_API_H__
#define __DRV_OCM_ECC_API_H__

#include "common_defines.h"
#include "bsp_device.h"

typedef enum ocmEccErrType {
    OCM_ECC_SEC = 0, ///< Single-bit error
    OCM_ECC_DED = 1, ///< Double-bit error
    OCM_ECC_ETYPE_MAX,
} ocmEccErrType_e;

/**
 * @brief ECC error callback function type
 * @details This callback is invoked when an ECC error is detected
 * @param [in] errType Error type (OCM_ECC_SEC or OCM_ECC_DED)
 * @param [in] errAddr Error address
 * @param [in] errData Error data
 */
typedef void (*pOcmEccCallback)(U8 errType, U32 errAddr, U64 errData);

/**
 * @brief Initialize OCM ECC driver
 * @details This function initializes the OCM ECC driver for the specified device.
 *          It allocates driver private data, reads device configuration from SBR,
 *          installs interrupt handler, and enables interrupts.
 * @param [in] devId Device ID
 * @return EXIT_SUCCESS on success
 * @return -EINVAL Invalid parameter (device not match or already initialized)
 * @return -EIO I/O error (failed to get device config, install drv, irq priority)
 * @return -EBUSY Device busy (device lock failed)
 * @return -ENOMEM Out of memory (failed to allocate driver data)
 */
S32 ocmEccInit(DevList_e devId);

/**
 * @brief Deinitialize OCM ECC driver
 * @details This function deinitializes the OCM ECC driver for the specified device.
 *          It disables interrupts, removes interrupt handler, and uninstalls the driver.
 * @param [in] devId Device ID
 * @return EXIT_SUCCESS on success
 * @return -EINVAL Invalid parameter (device not match, not initialized)
 * @return -EIO I/O error (failed to get device driver, drv data is null, or uninstall driver)
 * @return -EBUSY Device busy (device lock failed)
 */
S32 ocmEccDeInit(DevList_e devId);

/**
 * @brief Register interrupt callback function
 * @details This function registers a callback function to be called when an ECC error
 *          interrupt occurs. The callback is invoked with error type, address, and data.
 * @param [in] devId Device ID
 * @param [in] func Interrupt callback function pointer
 * @return EXIT_SUCCESS on success
 * @return -EINVAL Invalid parameter (param error, device not match, not initialized)
 * @return -EIO I/O error (failed to get device driver, drv data is null)
 * @return -EBUSY Device busy (device lock failed)
 */
S32 ocmEccIrqCbRegister(DevList_e devId, pOcmEccCallback func);

/**
 * @brief Deregister interrupt callback function
 * @details This function removes the previously registered interrupt callback function.
 * @param [in] devId Device ID
 * @return EXIT_SUCCESS on success
 * @return -EINVAL Invalid parameter (param error, device not match, not initialized)
 * @return -EIO I/O error (failed to get device driver, drv data is null)
 * @return -EBUSY Device busy (device lock failed)
 */
S32 ocmEccIrqCbUnregister(DevList_e devId);

/**
 * @brief Get ECC error count
 * @details This function retrieves the error count for the specified error type
 *          (single-bit or double-bit errors).
 * @param [in] devId Device ID
 * @param [in] eType Error type (OCM_ECC_SEC for single-bit, OCM_ECC_DED for double-bit)
 * @return >= 0 Error count on success
 * @return -EINVAL Invalid parameter (param error, device not match, not initialized)
 * @return -EIO I/O error (failed to get device driver, drv data is null)
 * @return -EBUSY Device busy (device lock failed)
 */
S32 ocmEccErrCntGet(DevList_e devId, ocmEccErrType_e eType);

/**
 * @brief Clear ECC error count
 * @details This function clears the error count for the specified error type.
 * @param [in] devId Device ID
 * @param [in] eType Error type (OCM_ECC_SEC for single-bit, OCM_ECC_DED for double-bit)
 * @return EXIT_SUCCESS on success
 * @return -EINVAL Invalid parameter (param error, device not match, not initialized)
 * @return -EIO I/O error (failed to get device driver, drv data is null)
 * @return -EBUSY Device busy (device lock failed)
 */
S32 ocmEccErrCntClear(DevList_e devId, ocmEccErrType_e eType);

/**
 * @brief Get last error address and data
 * @details This function retrieves the address and data of the last detected error
 *          for the specified error type.
 * @param [in] devId Device ID
 * @param [in] eType Error type (OCM_ECC_SEC for single-bit, OCM_ECC_DED for double-bit)
 * @param [out] pErrAddr Pointer to store the error address
 * @param [out] pErrData Pointer to store the error data
 * @return EXIT_SUCCESS on success
 * @return -EINVAL Invalid parameter (param error, device not match, not initialized)
 * @return -EIO I/O error (failed to get device driver, drv data is null)
 * @return -EBUSY Device busy (device lock failed)
 */
S32 ocmEccLastErrGet(DevList_e devId, ocmEccErrType_e eType, U32 *pErrAddr, U64 *pErrData);

/**
 * @brief Software trigger ECC error interrupt
 * @details This function software-triggers a single-bit or double-bit error interrupt
 *          for testing purposes.
 * @param [in] devId Device ID
 * @param [in] eType Error type (OCM_ECC_SEC for single-bit, OCM_ECC_DED for double-bit)
 * @return EXIT_SUCCESS on success
 * @return -EINVAL Invalid parameter (param error, device not match, not initialized)
 * @return -EIO I/O error (failed to get device driver, drv data is null)
 * @return -EBUSY Device busy (device lock failed)
 */
S32 ocmEccErrIrqTrigger(DevList_e devId, ocmEccErrType_e eType);

#endif /* __DRV_OCM_ECC_API_H__ */
