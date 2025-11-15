/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_sm3_api.h
 * @author baibch
 * @date 2025/09/30
 * @brief SM3 hash algorithm driver header
 */

#ifndef __DRV_SM3_API_H__
#define __DRV_SM3_API_H__

#include "common_defines.h"
#include "bsp_device.h"

#define SM3_FINISH_FLAGS     (1U)
#define SM3_ADDR_OVER_FLAGS  (1U)
#define SM3_NUM_BITS_PER_GRP (512)
#define SM3_LOCK_TIMEOUT_MS  (1000)

/* SM3 Mode Definitions */
typedef enum {
    SM3_MODE_SW = 0,
    SM3_MODE_HW,
    SM3_MODE_RESERVED
} Sm3Mode_e;

/* SM3 Status Definitions */
typedef enum {
    SM3_STATE_IDLE = 0,     /* Idle state */
    SM3_STATE_BUSY,         /* Busy processing */
    SM3_STATE_COMPLETE,     /* Calculation complete */
    SM3_STATE_ERROR         /* Error occurred */
} Sm3State_e;

/* Function Prototypes */

/**
 * @brief  Initialize SM3 module: register device to system, get device information
 * @details This function initializes the SM3 hardware module, registers interrupt handlers,
 *          and prepares the driver for use. It should be called before any other SM3 functions.
 * @param [in] devId Device enumeration ID
 * @return  EXIT_SUCCESS : 0 on success
 *          -EINVAL : Invalid device ID
 *          -EBUSY : Device already initialized
 *          -EIO : Hardware initialization error
 *          -ENOMEM : Memory allocation error
 */
S32 sm3Init(DevList_e devId);

/**
 * @brief  Calculate hash value using SM3 algorithm
 * @details This function performs SM3 hash calculation on the input data. The operation
 *          mode (hardware or software) depends on the current configuration.
 * @param [in] devId Device enumeration ID
 * @param [in] dataIn Pointer to input data buffer
 * @param [in] len Length of input data in 32-bit words
 * @param [out] hashBuf Pointer to output buffer for hash result (32 bytes)
 * @return  EXIT_SUCCESS : 0 on success
 *          -EINVAL : Invalid parameters
 *          -EIO : Device not initialized or hardware error
 *          -EBUSY : Device busy
 */
S32 sm3HashCalc(DevList_e devId, U32 *dataIn, U32 len, U32 *hashBuf);

/**
 * @brief  Get current state of SM3 module
 * @details This function returns the current operational state of the SM3 module.
 * @param [in] devId Device enumeration ID
 * @return  SM3_STATE_IDLE : Module is idle
 *          SM3_STATE_BUSY : Module is busy processing
 *          SM3_STATE_COMPLETE : Calculation completed
 *          SM3_STATE_ERROR : Error occurred
 */
Sm3State_e sm3StateGet(DevList_e devId);

/**
 * @brief  Select SM3 operation mode
 * @details This function sets the SM3 module to operate in either hardware or software mode.
 * @param [in] devId Device enumeration ID
 * @param [in] mode Operation mode (SM3_MODE_SW or SM3_MODE_HW)
 * @return  EXIT_SUCCESS : 0 on success
 *          -EINVAL : Invalid device ID or mode
 *          -EIO : Device not initialized
 *          -EBUSY : Device busy
 */
S32 sm3ModeSelect(DevList_e devId, Sm3Mode_e mode);

/**
 * @brief  Get hash result in software mode
 * @details This function retrieves the hash calculation result when operating in software mode.
 * @param [in] devId Device enumeration ID
 * @param [out] hashBuf Pointer to output buffer for hash result (32 bytes)
 * @param [in] timeout Timeout value for operation (milliseconds)
 * @return  EXIT_SUCCESS : 0 on success
 *          -EINVAL : Invalid device ID or parameters
 *          -EIO : Device not initialized
 *          -EBUSY : Device busy
 */
S32 sm3SoftResultGet(DevList_e devId, U32 *hashBuf, U32 timeout);

/**
 * @brief  Get hash result in hardware mode
 * @details This function retrieves the hash calculation result when operating in hardware mode.
 * @param [in] devId Device enumeration ID
 * @param [out] hashBuf Pointer to output buffer for hash result (32 bytes)
 * @param [in] timeout Timeout value for operation (milliseconds)
 * @return  EXIT_SUCCESS : 0 on success
 *          -EINVAL : Invalid device ID or parameters
 *          -EIO : Device not initialized or hardware error
 *          -EBUSY : Device busy
 */
S32 sm3HwResultGet(DevList_e devId, U32 *hashBuf, U32 timeout);

/**
 * @brief  Set group count for SM3 calculation
 * @details This function configures the number of data groups for the SM3 calculation.
 * @param [in] devId Device enumeration ID
 * @param [in] grpNum Number of groups
 * @return  EXIT_SUCCESS : 0 on success
 *          -EINVAL : Invalid device ID
 *          -EIO : Device not initialized
 *          -EBUSY : Device busy
 */
S32 sm3GroupSet(DevList_e devId, U32 grpNum);

/**
 * @brief  Set CHGIV (Change Initial Value) flag
 * @details This function sets the CHGIV flag to indicate that the initial value should be changed.
 * @param [in] devId Device enumeration ID
 * @return  EXIT_SUCCESS : 0 on success
 *          -EINVAL : Invalid device ID
 *          -EIO : Device not initialized
 *          -EBUSY : Device busy
 */
S32 sm3ChgivSet(DevList_e devId);

/**
 * @brief  Perform hash calculation in hardware mode
 * @details This function executes the SM3 hash calculation using hardware acceleration.
 * @param [in] devId Device enumeration ID
 * @param [in] dataIn Pointer to input data buffer
 * @param [in] len Length of input data in 32-bit words
 * @param [in] timeout Timeout value for operation (milliseconds)
 * @return  EXIT_SUCCESS : 0 on success
 *          -EINVAL : Invalid device ID or parameters
 *          -EIO : Device not initialized or hardware error
 *          -EBUSY : Device busy
 */
S32 sm3HwModeCalc(DevList_e devId, U32 *dataIn, U32 len, U32 timeout);

/**
 * @brief  Perform hash calculation in software mode with loop processing
 * @details This function executes the SM3 hash calculation using software implementation
 *          with loop processing for large data sets.
 * @param [in] devId Device enumeration ID
 * @param [in] dataIn Pointer to input data buffer
 * @param [in] len Length of input data in 32-bit words
 * @param [in] timeout Timeout value for operation (milliseconds)
 * @return  EXIT_SUCCESS : 0 on success
 *          -EINVAL : Invalid device ID or parameters
 *          -EIO : Device not initialized or processing error
 *          -EBUSY : Device busy
 */
S32 sm3SoftModeLoopCalc(DevList_e devId, U32 *dataIn, U32 len, U32 timeout);

/**
 * @brief  Deinitialize SM3 module: unregister device from system, release resources
 * @details This function deinitializes the SM3 hardware module, unregisters interrupt handlers,
 *          and releases all allocated resources.
 * @param [in] devId Device enumeration ID
 * @return  EXIT_SUCCESS : 0 on success
 *          -EINVAL : Invalid device ID
 *          -EBUSY : Device not initialized or busy
 */
S32 sm3DeInit(DevList_e devId);
#endif
