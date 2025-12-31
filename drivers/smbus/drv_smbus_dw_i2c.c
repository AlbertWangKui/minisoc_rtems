/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_smbus_dw_i2c.c
 * @author wangkui (wangkui@starsmicrosystem.com)
 * @date 2025/10/20
 * @brief SMBus Hardware Abstraction Layer implementation
 *
 * @par ChangeLog:
 * Date         Author          Description
 * 2025/10/20   wangkui         Initial version
 * 2025/12/01   wangkui         refactor and simplify the code as for core function
 * 2025/12/11   wangkui         correct the issue in AI review
 * 2025/12/15   wangkui         replace slave with target naming
 * @note This file implements the I2C Hardware register Layer, providing
 *       direct register access functions and low-level hardware operations.
 *       These functions operate directly on the DesignWare SMBus controller
 *       registers and provide the foundation for higher-level protocols.
 */

#include <rtems.h>
#include <rtems/counter.h>
#include <rtems/bspIo.h>
#include "bsp_config.h"
#include "log_msg.h"
#include "udelay.h"
#include "osp_interrupt.h"
#include "osp_barrier.h"
#include "drv_smbus_dw_i2c.h"
#include "drv_smbus_dw.h"

/* ======================================================================== */
/*                    Function Forward Declarations                         */
/* ======================================================================== */
static S32 smbusDwXferPoll(SmbusDev_s *dev, SmbusMsg_s *msgs, U32 num);
static S32 smbusWaitBusNotBusy(SmbusDev_s *dev);
static bool smbusIsControllerActive(SmbusDev_s *dev);
static S32 smbusHandleTxAbort(SmbusDev_s *dev);

/**
 * @brief Check if TX FIFO is ready (not full)
 */
static inline S32 smbusCheckTxready(volatile SmbusRegMap_s *regBase)
{
    SmbusIcStatusReg_u status;
    U32 timeout = SMBUS_TX_READY_TIMEOUT_US;
    while (timeout--) {
        status.value = smbusReadReg(&regBase->icStatus.value);
        if (status.fields.tfnf) {  /* TX FIFO not full */
            return 0;
        }
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(1));
    }
    return -ETIMEDOUT;
}

/**
 * @brief Check if RX FIFO has data
 */
static inline S32 smbusCheckRxready(volatile SmbusRegMap_s *regBase)
{
    SmbusIcStatusReg_u status;
    U32 timeout = SMBUS_RX_READY_TIMEOUT_US;
    while (timeout--) {
        status.value = smbusReadReg(&regBase->icStatus.value);
        if (status.fields.rfne) {  /* RX FIFO not empty */
            return 0;
        }
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(1));
    }
    return -ETIMEDOUT;
}

/**
 * @brief Disable SMBus controller
 * @details Safely disables the SMBus controller and waits for the
 *          disable operation to complete. This function should be called
 *          before reconfiguring the controller or during shutdown.
 * @param[in] dev Pointer to the SMBus device structure
 * @return void
 *
 * @note The function waits up to 1ms for the controller to be disabled
 * @note Clears the STATUS_ACTIVE flag in device status
 * @warning If timeout occurs, a warning is logged but the function continues
 */
static inline void smbusDisable(SmbusDev_s *dev)
{
    U32 timeout = 100;
    SmbusIcEnableReg_u enable;
    SmbusIcStatusReg_u status;

    SMBUS_CHECK_PARAM_VOID(dev == NULL || dev->regBase == NULL,
                          "SMBus: Invalid device or register base\n");

    /* Disable controller */
    enable.value = smbusReadReg(&dev->regBase->icEnable.value);
    if (enable.fields.enable) {
        enable.fields.enable = 0;
        smbusWriteReg(&dev->regBase->icEnable.value, enable.value);
    }

    /* Wait for controller to be disabled */
    while (timeout--) {
        status.value = smbusReadReg(&dev->regBase->icEnableStatus);
        if ((status.value & 0x1) == 0) {
            dev->status &= ~SMBUS_STATUS_ACTIVE;  /* Clear STATUS_ACTIVE */
            return;
        }
        udelay(10);
    }

    LOGW("SMBus: timeout waiting for disable\n");

    return;
}

/**
 * @brief Enable SMBus controller
 * @details Enables the SMBus controller and sets the device active status.
 *          This function should be called after the controller has been
 *          properly configured.
 * @param[in] dev Pointer to the SMBus device structure
 * @return void
 *
 * @note Sets the STATUS_ACTIVE flag in device status
 */
static inline void smbusEnable(SmbusDev_s *dev)
{
    SmbusIcEnableReg_u enable;

    SMBUS_CHECK_PARAM_VOID(dev == NULL || dev->regBase == NULL,
                          "SMBus: Invalid device or register base\n");

    enable.value = smbusReadReg(&dev->regBase->icEnable.value);
    enable.fields.enable = 1;
    smbusWriteReg(&dev->regBase->icEnable.value, enable.value);
    dev->status |= SMBUS_STATUS_ACTIVE;  /* Set STATUS_ACTIVE */

    return;
}

/**
 * @brief Configure SMBus interrupts for transfer initialization
 * @details Configures interrupt masks based on device mode and settings.
 *          This function is optimized for use in transfer initialization
 *          and provides comprehensive interrupt configuration.
 * @param[in] dev Pointer to SMBus device structure
 * @param[in] regBase Pointer to SMBus register map base address
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Handles both master and target mode configurations
 * @note Configures SMBus-specific interrupts when SMBus mode is enabled
 * @note Clears pending interrupts before configuration
 */
static S32 smbusConfigTransferInterrupts(SmbusDev_s *dev, volatile SmbusRegMap_s *regBase)
{
    SMBUS_CHECK_PARAM_RETURN(dev == NULL || regBase == NULL, -EINVAL,
                            "dev or regBase is NULL");

    /* Clear any pending interrupts */
    regBase->icClrIntr = 0x7FFF;

    /* Clear SMBus interrupt register */
    regBase->icClrSmbusIntr = 0x7FFF;

    /* Configure interrupts based on work mode */
    if (dev->workMode == 1) {
        /* Polling mode - DISABLE all interrupts to let CPU handle data polling */
        regBase->icIntrMask.value = 0;
        LOGD("SMBus: Polling mode - interrupts disabled for CPU polling\n");
    } else {
        /* Async interrupt mode - enable interrupts based on mode */
        if (dev->mode == SMBUS_MODE_TARGET) {
            /*
             * target mode: Configure optimal interrupts for target operations
             * - Write transfer: WR_REQ, RX_FULL for receiving data
             * - Read transfer: RD_REQ, TX_EMPTY for sending data
             * - Common: TX_ABRT, STOP_DET, RESTART_DET for error handling
             */
            regBase->icIntrMask.value = SMBUS_TARGET_OPTIMAL_CONFIG;
            dev->status = SMBUS_STATUS_IDLE;
            LOGD("SMBus: target mode interrupts configured (mask: 0x%08X)\n", SMBUS_TARGET_OPTIMAL_CONFIG);
        } else {
            /*
             * Master mode: Configure essential interrupts for master operations
             * - TX_EMPTY: For sending data to target
             * - RX_FULL: For receiving data from target
             * - TX_ABRT: Handle transfer aborts
             * - STOP_DET/START_DET: Detect bus conditions
             * - ACTIVITY: Monitor bus activity status
             */
            regBase->icIntrMask.value = SMBUS_MASTER_INTERRUPT_CONFIG;
            LOGD("SMBus: Master mode interrupts enabled (0x%08X)\n", SMBUS_MASTER_INTERRUPT_CONFIG);
        }
    }
    
    /* Enable critical error-related interrupts for both modes */
    regBase->icIntrMask.value = regBase->icIntrMask.value | SMBUS_ERROR_INTERRUPT_CONFIG;
    return EXIT_SUCCESS;
}

/**
 * @brief Initialize SMBus transfer
 * @details Prepares the SMBus controller for a transfer by configuring
 *          necessary registers such as IC_CON and IC_TAR. This function
 *          also sets up interrupts based on the device mode.
 * @param[in] dev Pointer to SMBus device structure
 * @param[in] msgs Array of SMBus messages to be transferred
 * @param[in] msgIdx Index of the current message in the msgs array
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Configures IC_CON for master mode and addressing mode
 * @note Sets IC_TAR based on message address and device addressing mode
 * @note Calls smbusConfigTransferInterrupts to set up interrupts
 * @note Enables the controller to start the transfer
 */
static S32 smbusDwXferInit(SmbusDev_s *dev, SmbusMsg_s msgs[], U32 msgIdx)
{
    SmbusIcConReg_u icCon;
    SmbusIcTarReg_u icTar;
    volatile SmbusRegMap_s *regBase;
    S32 ret;

    /* Basic parameter validation */
    SMBUS_CHECK_PARAM_RETURN(
        dev == NULL || dev->regBase == NULL || msgs == NULL,
        -EINVAL,
        "SMBus: Invalid parameters for xfer init"
    );

    regBase = (volatile SmbusRegMap_s *)dev->regBase;

    /* Disable controller before configuring registers */
    smbusDisable(dev);

    /* ============================================================
     * 1. Configure IC_CON (master mode, restart, address mode)
     * ============================================================*/
    icCon.value = regBase->icCon.value;

    /* Master mode */
    icCon.fields.masterMode = 1;
    icCon.fields.ictargetDisable = 1;  ///< host notify used

    /* Addressing Mode */
    if (dev->addrMode == 1) {
        icCon.fields.ic10bitaddrMaster = 1;
        LOGD("SMBus: Using 10-bit addressing\n");
    } else {
        icCon.fields.ic10bitaddrMaster = 0;
        LOGD("SMBus: Using 7-bit addressing\n");
    }
    
    if (smbusIsControllerActive(dev)) {  ///< prevent re-config when controller is active
        LOGW("SMBus: Controller still active during xfer init\n");
        return -EBUSY;
    }
    /* Restart enable (mandatory for SMBus block read and most reads) */
    icCon.fields.icRestartEn = dev->restartEnb ? 1 : 0;
    regBase->icCon.value = icCon.value;
    LOGE("SMBus: IC_CON set to 0x%X\n", regBase->icCon.value);
    /* ============================================================
     * 2. Set IC_TAR (normal transfer)
     * ============================================================*/
    icTar.value = 0;

    if (dev->addrMode == 1) {
        icTar.fields.icTar = msgs[msgIdx].addr & 0x3FF;
        icTar.fields.ic10bitaddrMaster = 1;
    } else {
        icTar.fields.icTar = msgs[msgIdx].addr & 0x7F;
        icTar.fields.ic10bitaddrMaster = 0;
    }
    regBase->icTar.value = icTar.value;
    LOGD("SMBus: IC_TAR set to 0x%X\n", regBase->icTar.value);

    /* ============================================================
     * 3. Configure interrupts (Polling or IRQ)
     * ============================================================*/
    ret = smbusConfigTransferInterrupts(dev, regBase);

    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: Interrupt configuration failed (%d)\n", ret);
        return ret;
    }

    /* ============================================================
     * 4. Enable adapter to start transfer
     * ============================================================*/
    smbusEnable(dev);

    LOGD("SMBus: Transfer init complete. Target=0x%02X, mode=%u, intr_mask=0x%08X\n",
         msgs[msgIdx].addr,
         dev->addrMode,
         regBase->icIntrMask.value);

    return EXIT_SUCCESS;
}

/**
 * @brief Check for transfer errors
 * @details This function checks for various error conditions during
 *          SMBus transfer and returns appropriate error codes.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Checks for abort conditions and FIFO errors
 */
static S32 smbusDwCheckErrors(SmbusDev_s *dev)
{
    SmbusIcTxAbrtSourceReg_u abrtSource;
    volatile SmbusRegMap_s *regBase;

    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL, -EINVAL,
                            "SMBus: Invalid parameters for error check");

    regBase = dev->regBase;

    /* Check for TX abort in raw interrupt status */
    SmbusIcRawIntrStatReg_u rawIntr;
    rawIntr.value = smbusReadReg(&regBase->icRawIntrStat.value);
    if (rawIntr.fields.txAbrt) {
        /* Read and log abort source */
        abrtSource.value = smbusReadReg(&regBase->icTxAbrtSource.value);
        LOGE("SMBus: Transfer abort detected, source=0x%08X\n", abrtSource.value);

        /* Clear abort condition */
        (void)regBase->icClrTxAbrt;

        /* Return specific error based on abort source */
        if (abrtSource.value & SMBUS_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_MASK) {
            return -ENXIO;  /* No acknowledge from target */
        } else if (abrtSource.value & SMBUS_IC_TX_ABRT_SOURCE_ABRT_ARB_LOST_MASK) {
            return -EAGAIN;  /* Arbitration lost */
        } else if (abrtSource.value & SMBUS_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_MASK) {
            return -EIO;  /* Master disabled */
        } else {
            return -EIO;  /* Generic I/O error */
        }
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Check for stop bit detection
 * @details This function waits for the stop bit to be detected,
 *          indicating the completion of a transfer.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Timeout is based on SMBUS_TRANSACTION_TIMEOUT_US
 */
static S32 smbusDwCheckStopBit(SmbusDev_s *dev)
{
    U32 timeout = SMBUS_TRANSACTION_TIMEOUT_US;

    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL, -EINVAL,
                            "SMBus: Invalid parameters for stop bit check");

    while (timeout > 0) {
        SmbusIcRawIntrStatReg_u rawIntr = dev->regBase->icRawIntrStat;
        if (rawIntr.fields.stopDet) {
            /* Clear stop detect interrupt */
            (void)dev->regBase->icClrIntr;
            return EXIT_SUCCESS;
        }
        udelay(1);
        timeout--;
    }
    LOGE("SMBus: Stop bit detection timeout\n");
    return -ETIMEDOUT;
}

/**
 * @brief SMBus I2C-compatible transfer function
 * @details This function implements SMBus data transfer using the same
 *          interface as the I2C DesignWare master driver. It provides
 *          compatibility with existing I2C applications while using
 *          the SMBus hardware and HAL layer. The function supports both
 *          read and write operations with proper error handling.
 * @param[in] dev Pointer to SMBus device structure
 * @param[in] msgs Array of SMBus messages to transfer
 * @param[in] num Number of messages in the array
 * @return Number of messages transferred on success, negative error code on failure
 *
 * @note This function is compatible with i2c_dw_xfer interface
 * @note Supports both 7-bit and 10-bit addressing
 * @note Uses polling mode for simplicity
 * @note Implements proper error handling and timeout management
 *
 * @warning Ensure device is properly initialized before calling
 * @warning Function modifies device status during transfer
 */
static U32 smbusDwXfer(SmbusDev_s *dev, void *mg, U32 num)
{
    S32 ret = 0;

    SMBUS_CHECK_PARAM(dev == NULL || dev->regBase == NULL || mg == NULL || num == 0,
                      -EINVAL, "SMBus: Invalid parameters for dw xfer");

    /* Check for potential buffer overflow */
    if (num > (SMBUS_MAX_BLOCK_LEN + 1)) {
        LOGE("SMBus: Too many messages (%d > %d)\n", num, SMBUS_MAX_BLOCK_LEN + 1);
        return -EINVAL;
    }

    SmbusMsg_s *msgs = (SmbusMsg_s *)mg;

    ///< Polling mode (original implementation)
    if (dev->workMode == 1) {
        ret = smbusDwXferPoll(dev, msgs, num);
        return (U32)ret;
    }

    ///< Async interrupt mode
    while(RTEMS_SUCCESSFUL == rtems_semaphore_obtain(dev->semaphoreId, RTEMS_NO_WAIT, 0)) {
        ///< Clear any pending semaphore
        ;
    }

    LOGD("SMBus: BEFORE Waiting for async transfer completion, timeout=%dms\n", dev->transferTimeout);
    ///< Initialize async transfer state
    /* CRITICAL FIX: Copy ALL messages, not just one */
    memcpy(&dev->msgs, msgs, sizeof(SmbusMsg_s) * num);
    dev->msgsNum = num;

    /* DEBUG: Verify message copy */
    for (int i = 0; i < num; i++) {
        LOGD("SMBus: Copied msg[%d] - addr=0x%02X, flags=0x%04X, len=%d\n",
             i, dev->msgs[i].addr, dev->msgs[i].flags, dev->msgs[i].len);
    }
    dev->msgWriteIdx = 0;
    dev->msgReadIdx = 0;
    dev->msgErr = 0;
    dev->status = SMBUS_STATUS_ACTIVE;
    dev->cmdErr = 0;
    dev->abortSource = 0;
    dev->rxOutstanding = 0;

    /* Record transfer start time for timeout protection */
    dev->transferStartTime = rtems_clock_get_ticks_since_boot();

    ///< Wait for bus not busy
    ret = smbusWaitBusNotBusy(dev);
    if (ret < 0) {
        LOGE("SMBus: Bus busy error: %d\n", ret);
        goto exit;
    }

    /* Start the async transfer, race condition/timeout 
     * issue where the stack-allocated msg pointer is passed to 
     * smbusDwXferInit instead of the persistent copy in dev->msgs
     */
    ret = smbusDwXferInit(dev, dev->msgs, 0);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: Transfer initialization failed: %d\n", ret);
        goto exit;
    }

    ///< Wait for transfer completion - let ISR handle timeout protection
    LOGD("SMBus: Waiting for transfer completion - semId=%d, status=0x%08X, timeout=%d\n",
         dev->semaphoreId, dev->status, dev->transferTimeout);

    S32 semResult = rtems_semaphore_obtain(dev->semaphoreId, RTEMS_WAIT, dev->transferTimeout);
    if(semResult != RTEMS_SUCCESSFUL) {
        LOGE("SMBus: Semaphore obtain failed unexpectedly: semResult=%d, semaphoreId=%d\n",
            semResult, dev->semaphoreId);
        LOGE("SMBus: Current device state - status=0x%08X, rxOutstanding=%d, msgWriteIdx=%d, msgReadIdx=%d, msgsNum=%d\n",
             dev->status, dev->rxOutstanding, dev->msgWriteIdx, dev->msgReadIdx, dev->msgsNum);

        /* CRITICAL: Force completion to prevent hanging */
        LOGE("SMBus: Forcing transfer completion due to semaphore failure\n");
        dev->status &= ~(SMBUS_STATUS_READ_IN_PROGRESS | SMBUS_STATUS_WRITE_IN_PROGRESS);
        dev->rxOutstanding = 0;
        ret = -ETIMEDOUT; /* Return timeout error */
        goto exit;
    }

    LOGD("SMBus: Transfer completed - semaphore released by ISR\n");

    U16 activeRetry = 50000;
    while (smbusIsControllerActive(dev) && activeRetry > 0) {
        udelay(1);
        activeRetry--;
    }

    /* Only report error if still active after retry and not normal completion state */
    if (smbusIsControllerActive(dev)) {
        ///< Lower log level, or ignore this error when ret == SUCCESS
        if (dev->errorType == 0) {
            LOGW("SMBus: Controller active bit sticky, but transfer seems success. Ignoring.\n");
        } else {
            LOGE("SMBus: Controller still active after transfer completion\n");
            dev->errorType = SMBUS_ERR_TYPE_UNKNOWN; // or specific timeout error
        }
    }

    if (dev->cmdErr || dev->errorType > 0 || dev->msgErr) {
        if (dev->msgErr) {
            /* Ensure msgErr is also negative */
            ret = (dev->msgErr < 0) ? dev->msgErr : -EIO;
            LOGE("SMBus: Message error: %d\n", ret);
            goto exit;
        }
        ret = smbusHandleTxAbort(dev);
        ///< Log enhanced error information with device context
        LOGE("SMBus: Transfer failed - Device context:\n");
        LOGE("  - set target address: 0x%02X\n", dev->regBase->icTar.fields.icTar);
        LOGE("  - Error type: %d (%s)\n", dev->errorType,
             (dev->errorType == SMBUS_ERR_TYPE_NACK_7BIT) ? "NACK_7BIT" :
             (dev->errorType == SMBUS_ERR_TYPE_NACK_10BIT) ? "NACK_10BIT" :
             (dev->errorType == SMBUS_ERR_TYPE_NACK_DATA) ? "NACK_DATA" :
             (dev->errorType == SMBUS_ERR_TYPE_NACK_GCALL) ? "NACK_GCALL" :
             (dev->errorType == SMBUS_ERR_TYPE_ARB_LOST) ? "ARB_LOST" :
             (dev->errorType == SMBUS_ERR_TYPE_SDA_STUCK) ? "SDA_STUCK" :
             (dev->errorType == SMBUS_ERR_TYPE_MASTER_DISABLED) ? "MASTER_DISABLED" : "UNKNOWN");
        LOGE("  - Abort source: 0x%08X\n", dev->abortSource);
        LOGE("  - Return code: %d\n", ret);
        goto exit;
    }

    ///< Check for status errors
    if (dev->status & ~SMBUS_STATUS_MASK) {
        LOGE("SMBus: Transfer terminated early - interrupt latency too high?\n");
        ret = -EFAULT;
        goto exit;
    }
    ///< Success - return number of messages transferred
    ret = num;
    LOGD("SMBus: Async dw xfer completed successfully\n");
exit:
    return (U32)ret;
}

/**
 * @brief SMBus polling mode transfer function
 * @details Performs SMBus transfer using polling mode (original implementation).
 *          This function is used when workMode = 1.
 * @param[in] dev Pointer to SMBus device structure
 * @param[in] msgs Pointer to message array
 * @param[in] num Number of messages to transfer
 * @return Number of messages transferred on success, negative error code on failure
 *
 * @note This is the original polling implementation
 */
static S32 smbusDwXferPoll(SmbusDev_s *dev, SmbusMsg_s *msgs, U32 num)
{
    S32 status = 0;
    U32 msgWrtIdx, msgItrLmt, bufLen;
    U8 *buf;
    U32 val;
    U32 checkErrRetry = 0;

    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL || msgs == NULL || num == 0,
                            -EINVAL, "SMBus: Invalid parameters for poll xfer");

    LOGT("SMBus: Starting poll dw xfer, %u messages\n", num);

    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)dev->regBase;

    /* Store message information */
    dev->status |= SMBUS_STATUS_ACTIVE;
    dev->cmdErr = 0;

    /* Clear any pending interrupts before starting transfer */
    (void)regBase->icClrIntr;
    (void)regBase->icClrTxAbrt;
    (void)regBase->icClrStopDet;
    (void)regBase->icClrActivity;

    
    /* Initialize transfer */
    status = smbusDwXferInit(dev, msgs, 0);
    if (status != EXIT_SUCCESS) {
        LOGE("SMBus: Poll transfer initialization failed: %d\n", status);
        return status;
    }

    /* Initiate messages read/write transaction */
    for (msgWrtIdx = 0; msgWrtIdx < num; msgWrtIdx++) {
        SmbusMsg_s *pMsg = &msgs[msgWrtIdx];

        buf = pMsg->buf;
        bufLen = pMsg->len;
        LOGD("SMBus Poll Msg[%d]: Addr=0x%02X, Len=%d, Flags=0x%04X, IsRead=%d\n", 
             msgWrtIdx, pMsg->addr, bufLen, pMsg->flags, (pMsg->flags & SMBUS_M_RD) ? 1 : 0);

        for (msgItrLmt = bufLen; msgItrLmt > 0; msgItrLmt--) {
            U32 cmd = 0;
            U32 dataCmd = 0;

            /* Check if this is a read operation */
            if (msgs[msgWrtIdx].flags & SMBUS_M_RD) {
                /* Read operation */
                status = smbusCheckTxready(regBase);
                if (status != EXIT_SUCCESS) {
                    LOGE("SMBus: TX ready timeout in read: %d\n", status);
                    return status;
                }

                /* Set stop bit for last byte of last message */
                if (msgWrtIdx == num - 1 && msgItrLmt == 1) {
                    cmd |= SMBUS_IC_DATA_CMD_STOP;
                }

                /* Send read command - format: READ_CMD for read + stop bit if needed */
                dataCmd = SMBUS_IC_DATA_CMD_READ_CMD | cmd;
                LOGD("DEBUG: Read dataCmd = 0x%08X\n", dataCmd);
                smbusWriteReg(&regBase->icDataCmd.value, dataCmd);

                /* Check for errors after command */
                checkErrRetry = 0;
                while (checkErrRetry < 10) {
                    status = smbusDwCheckErrors(dev);
                    if (status == EXIT_SUCCESS) {
                        break;
                    }
                    checkErrRetry++;
                    rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(10));
                }
                if (status != EXIT_SUCCESS) {
                    LOGE("SMBus: Error detected in read command: %d\n", status);
                    return status;
                }

                /* Wait for data ready */
                status = smbusCheckRxready(dev->regBase);
                if (status != EXIT_SUCCESS) {
                    LOGE("SMBus: RX ready timeout in read: %d\n", status);
                    return status;
                }

                /* Read data */
                val = smbusReadReg(&regBase->icDataCmd.value);

                /* Check for errors after read */
                checkErrRetry = 0;
                while (checkErrRetry < 10) {
                    status = smbusDwCheckErrors(dev);
                    if (status == EXIT_SUCCESS) {
                        break;
                    }
                    checkErrRetry++;
                    rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(10));
                }
                if (status != EXIT_SUCCESS) {
                    LOGE("SMBus: Error detected after read: %d\n", status);
                    return status;
                }

                *buf++ = (U8)(val & 0xFF);

            } else {
                /* Write operation */
                status = smbusCheckTxready(dev->regBase);
                if (status != EXIT_SUCCESS) {
                    LOGE("SMBus: TX ready timeout in write: %d\n", status);
                    return status;
                }

                /* Set stop bit for last byte of last message */
                if (msgWrtIdx == num - 1 && msgItrLmt == 1) {
                    cmd |= SMBUS_IC_DATA_CMD_STOP;
                }

                /* Write data */
                U8 dataByte = (buf) ? *buf++ : 0;
                dataCmd = dataByte | cmd;
                smbusWriteReg(&regBase->icDataCmd.value, dataCmd);

                /* Check for errors after write */
                checkErrRetry = 0;
                while (checkErrRetry < 10) {
                    status = smbusDwCheckErrors(dev);
                    if (status == EXIT_SUCCESS) {
                        break;
                    } else {
                        LOGE("SMBus: Error detected in write: %d\n", status);
                        return status;
                    }
                    checkErrRetry++;
                    rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(10));
                }
            }
        }

        /* Check for stop bit detection at the end of transaction */
        if (msgWrtIdx == num - 1) {
             status = smbusDwCheckStopBit(dev);
             if (status != EXIT_SUCCESS) {
                 LOGE("SMBus: Stop bit check failed: %d\n", status);
                 return status;
             }
        }
    }

    LOGD("SMBus: Poll dw xfer completed successfully\n");
    return num;
}

static S32 smbusPerformHardwareRecovery(volatile SmbusRegMap_s *regBase)
{
    U32 timeout;
    U32 currentVal;

    LOGW("SMBus: Triggering Hardware SDA Stuck Recovery (Bit 3)...\n");

    /* 1. Ensure controller is enabled (ENABLE = 1)
     * Hardware needs clock to send recovery pulses, so must enable first.
     */
    if (!(smbusReadReg(&regBase->icEnable.value) & SMBUS_IC_ENABLE_ENABLE_MASK)) {
        smbusWriteReg(&regBase->icEnable.value, smbusReadReg(&regBase->icEnable.value) | SMBUS_IC_ENABLE_ENABLE_MASK);
        /* Simple wait for enable to take effect */
        rtems_task_wake_after(RTEMS_MICROSECONDS_TO_TICKS(1));
    }

    /* 2. Set SDA_STUCK_RECOVERY_ENABLE (Bit 3) */
    /* Use read-modify-write to protect other bits */
    currentVal = smbusReadReg(&regBase->icEnable.value);
    smbusWriteReg(&regBase->icEnable.value, currentVal | SMBUS_IC_ENABLE_RECOVERY_MASK);

    /* 3. Wait for hardware to automatically clear this bit (recovery action complete) */
    /* Usually requires a few I2C cycles, 1ms timeout is more than sufficient */
    timeout = 100;
    while (timeout--) {
        if (!(regBase->icEnable.value & SMBUS_IC_ENABLE_RECOVERY_MASK)) {
            /* Bit 3 has been auto-cleared, recovery action complete */
            LOGI("SMBus: Hardware Recovery sequence finished.\n");
            return EXIT_SUCCESS;
        }
        rtems_task_wake_after(RTEMS_MICROSECONDS_TO_TICKS(1) > 0 ? 1 : 1);
    }

    LOGE("SMBus: Hardware Recovery TIMEOUT! Controller stuck.\n");
    return -ETIMEDOUT;
}

/**
 * @brief Wait for SMBus bus to become not busy
 * @details Checks if the SMBus controller is idle and ready for new transfer.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note This function polls the IC_STATUS register
 */
static inline S32 smbusWaitBusNotBusy(SmbusDev_s *dev)
{
    U32 timeout = 1000;  ///< 1 second timeout
    S32 ret = EXIT_SUCCESS;

    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL, -EINVAL,
                            "SMBus: Invalid parameters for wait bus not busy");

    volatile SmbusRegMap_s *regBase = dev->regBase;

    if (!(smbusReadReg(&regBase->icStatus.value) & SMBUS_IC_STATUS_ACTIVITY_MASK)) {
        return EXIT_SUCCESS;
    }

    /* Give previous transfer some completion time (2ms) */
    timeout = 200;
    while (timeout--) {
        if (!(smbusReadReg(&regBase->icStatus.value) & SMBUS_IC_STATUS_ACTIVITY_MASK)) {
            return EXIT_SUCCESS;
        }
        rtems_task_wake_after(RTEMS_MICROSECONDS_TO_TICKS(10));
    }

    LOGW("SMBus: Bus stuck BUSY (Stat=0x%08X), starting recovery procedure...\n", regBase->icStatus.value);

    /* ---------------------------------------------------------
     * 2. Perform hardware bus recovery (SDA Stuck Recovery)
     * --------------------------------------------------------- */
    ret = smbusPerformHardwareRecovery(regBase);
    if (ret != EXIT_SUCCESS) {
        /* If hardware recovery times out, IP may be stuck, try force reset */
        LOGE("SMBus: Hardware recovery failed, proceeding to force reset.\n");
    }

    /* ---------------------------------------------------------
     * 3. State machine reset process (Abort -> Disable -> Enable)
     * Even if bus is electrically recovered, IP state machine may still think it's in transfer
     * --------------------------------------------------------- */
    /* 3.1 Try sending ABORT signal (terminate any pending transfers) */
    /* Abort is only effective when Enable=1 */
    if (regBase->icEnable.value & SMBUS_IC_ENABLE_ENABLE_MASK) {
        regBase->icEnable.value |= SMBUS_IC_ENABLE_ABORT_MASK;

        /* Wait for ABORT completion (TX_ABRT interrupt bit set) */
        timeout = 1000;
        while (timeout--) {
            if (regBase->icRawIntrStat.value & SMBUS_INTR_TX_ABRT_MASK) {
                (void)regBase->icClrTxAbrt; /* Clear interrupt */
                break;
            }
            rtems_task_wake_after(RTEMS_MICROSECONDS_TO_TICKS(10));
        }
    }

    /* 3.2 Force disable controller (Disable) - complete state machine reset */
    smbusWriteReg(&regBase->icEnable.value, 0);
    /* Wait for Enable bit to truly clear */
    timeout = 2000; /* 20ms */
    while (timeout--) {
        if (!(regBase->icEnableStatus & SMBUS_IC_ENABLE_ENABLE_MASK)) {
            break;
        }
        rtems_task_wake_after(RTEMS_MICROSECONDS_TO_TICKS(10));
    }

    /* 3.3 Re-enable controller (Enable) */
    smbusWriteReg(&regBase->icEnable.value, SMBUS_IC_ENABLE_ENABLE_MASK);

    /* ---------------------------------------------------------
     * 4. Final status check
     * --------------------------------------------------------- */
    /* Bus should be idle at this point */
    if (regBase->icStatus.value & SMBUS_IC_STATUS_ACTIVITY_MASK) {
        LOGE("SMBus: Critical Failure - Bus still BUSY after HW Recovery & Reset. (Stat=0x%08X)\n",
             regBase->icStatus.value);
        return -EIO;
    }
    LOGI("SMBus: Bus recovery SUCCESSFUL. Ready for transfer.\n");
    return EXIT_SUCCESS;
}

/**
 * @brief Check if SMBus controller is still active
 * @details Checks if the SMBus controller is still active after transfer.
 * @param[in] dev Pointer to SMBus device structure
 * @return true if controller is active, false otherwise
 *
 * @note This function checks the IC_STATUS register
 */
static bool smbusIsControllerActive(SmbusDev_s *dev)
{
    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL, false,
                            "SMBus: Invalid parameters for controller active check");

    volatile SmbusRegMap_s *regBase = dev->regBase;
    U32 status = smbusReadReg(&regBase->icStatus.value);

    return (status & 0x01) != 0;  ///< Check ACTIVITY bit
}

/**
 * @brief Handle SMBus TX abort condition
 * @details Handles TX abort errors and returns appropriate error code.
 * @param[in] dev Pointer to SMBus device structure
 * @return Error code based on abort source
 *
 * @note This function reads IC_TX_ABRT_SOURCE register
 */
static S32 smbusHandleTxAbort(SmbusDev_s *dev)
{
    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL, -EINVAL,
                            "SMBus: Invalid parameters for handle tx abort");

    ///< Store abort source and error type in device structure
    U32 abortSource = dev->abortSource;
    ///< Classify the specific error type and log detailed information
    if (abortSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_MASK) {
        dev->errorType = SMBUS_ERR_TYPE_NACK_7BIT;
        LOGE("SMBus: 7-bit address NACK (0x%02X) - target device not responding or not present\n",
             dev->targetAddr);
        LOGE("SMBus: Abort source=0x%08X, errorType=NACK_7BIT\n", abortSource);
        return -ENXIO;  ///< No such device or address
    } else if (abortSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_10B_ADDR_NOACK_MASK) {
        dev->errorType = SMBUS_ERR_TYPE_NACK_10BIT;
        LOGE("SMBus: 10-bit address NACK (0x%03X) - target device not responding\n",
             dev->targetAddr);
        LOGE("SMBus: Abort source=0x%08X, errorType=NACK_10BIT\n", abortSource);
        return -ENXIO;  ///< No such device or address
    } else if (abortSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_MASK) {
        dev->errorType = SMBUS_ERR_TYPE_NACK_DATA;
        LOGE("SMBus: Data NACK - target rejected data during transfer\n");
        LOGE("SMBus: Abort source=0x%08X, errorType=NACK_DATA\n", abortSource);
        return -EIO;   ///< I/O error during data transfer
    } else if (abortSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK_MASK) {
        dev->errorType = SMBUS_ERR_TYPE_NACK_GCALL;
        LOGE("SMBus: General Call NACK - No devices responding to broadcast\n");
        LOGE("SMBus: Abort source=0x%08X, errorType=NACK_GCALL\n", abortSource);
        return -ENXIO;  ///< No devices responding
    } else if (abortSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_ARB_LOST_MASK) {
        dev->errorType = SMBUS_ERR_TYPE_ARB_LOST;
        LOGE("SMBus: Arbitration lost - Bus contention with another master\n");
        LOGE("SMBus: Abort source=0x%08X, errorType=ARB_LOST\n", abortSource);
        return -EAGAIN; ///< Retry may succeed
    } else if (abortSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_SDA_STUCK_LOW_MASK) {
        dev->errorType = SMBUS_ERR_TYPE_SDA_STUCK;
        LOGE("SMBus: SDA line stuck low - Hardware fault or short circuit\n");
        LOGE("SMBus: Abort source=0x%08X, errorType=SDA_STUCK\n", abortSource);
        return -EIO;   ///< Hardware I/O error
    } else if (abortSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_MASK) {
        dev->errorType = SMBUS_ERR_TYPE_MASTER_DISABLED;
        LOGE("SMBus: Master disabled - Controller not properly configured\n");
        LOGE("SMBus: Abort source=0x%08X, errorType=MASTER_DISABLED\n", abortSource);
        return -EIO;   ///< Controller I/O error
    } else {
        dev->errorType = SMBUS_ERR_TYPE_UNKNOWN;
        LOGE("SMBus: Unknown abort source=0x%08X - Check hardware configuration\n", abortSource);
        LOGE("SMBus: errorType=UNKNOWN\n");
        return -EIO;   ///< Generic I/O error
    }
}

/**
 * @brief Handle SMBus Quick Command hardware operations
 * @details Implements hardware-specific handling for Quick Command (msg.len == 0)
 *          This function operates at the hardware abstraction layer and handles
 *          the special register configuration required for Quick Commands.
 * @param[in] dev SMBus device handle
 * @param[in] msg I2C message with len == 0 for Quick Command
 * @return EXIT_SUCCESS on success, negative error code on failure
 * @note This function maintains clean protocol/hardware layer separation
 */
static S32 smbusHandleQuickCommand(SmbusDev_s *dev, SmbusMsg_s *msg)
{
    volatile SmbusRegMap_s *regBase = dev->regBase;
    SmbusIcTarReg_u icTar = {0};
    SmbusIcDataCmdReg_u icDataCmd = {0};
    S32 ret;

    dev->isQuick = 1;

    if (msg->addr > 0x7F) return -EINVAL;
    smbusDisable(dev);

    /* 2. Configure IC_TAR (Master mode target address) */
    icTar.fields.icTar = msg->addr & 0x7F;
    icTar.fields.special = 1;       ///< Enable special command
    icTar.fields.smbusQuickCmd = 1; ///< Mark as Quick Command
    smbusWriteReg(&regBase->icTar.value, icTar.value);

    smbusEnable(dev);

    /* 3. Prepare synchronization mechanism */
    dev->xferStatus = -ETIMEDOUT; ///< Default state is timeout
    dev->isQuick = 1;             ///< Tell ISR: This is Quick Command, please handle STOP signal

    /* Clear any pending interrupt status */
    (void)regBase->icClrStopDet;
    (void)regBase->icClrTxAbrt;

    /* 4. Enable interrupts (critical!) */
    /* Must enable STOP_DET and TX_ABRT, otherwise ISR will never trigger */
    U32 intrMask = smbusReadReg(&regBase->icIntrMask.value);
    smbusWriteReg(&regBase->icIntrMask.value, intrMask | SMBUS_INTR_STOP_DET | SMBUS_INTR_TX_ABRT);
    U32 intrSmbusMask = smbusReadReg(&regBase->icSmbusIntrMask.value);
    smbusWriteReg(&regBase->icSmbusIntrMask.value, intrSmbusMask | SMBUS_QUICK_CMD_DET_BIT);

    /* 5. Write command word to trigger hardware transfer */
    icDataCmd.fields.cmd = (msg->flags & SMBUS_M_RD) ? 1 : 0; ///< Read/Write
    icDataCmd.fields.stop = 1;                                ///< Must generate STOP
    icDataCmd.fields.restart = 0;
    smbusWriteReg(&regBase->icDataCmd.value, icDataCmd.value);

    /* 6. Sleep waiting for ISR to wake up */
    if (rtems_semaphore_obtain(dev->semaphoreId, RTEMS_WAIT, dev->transferTimeout) == RTEMS_SUCCESSFUL) {
         /* Woken up by ISR, return result set by ISR */
         ret = dev->xferStatus;
    } else {
        /* Timeout: hardware did not respond */
        LOGE("SMBus: Quick Command Timed Out!\n");
        ret = -ETIMEDOUT;
        /* Need to do cleanup here to prevent late interrupts from corrupting state */
        dev->isQuick = 0;
    }
    /* 7. Restore interrupt mask (optional) */
    smbusWriteReg(&regBase->icIntrMask.value, intrMask);
    return ret;
}

/**
 * @brief Master read data from RX FIFO
 * @details Reads data from RX FIFO and stores in driver data structure
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @return void
 *
 */
void smbusMasterReadData(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase)
{
    U32 rxValid = 0;
    U32 i = 0;

    SMBUS_CHECK_PARAM_VOID(pDrvData == NULL || regBase == NULL,
                          "SMBus: NULL pointer in smbusMasterReadData");

    /* CRITICAL: Check if rxBuffer is properly allocated */
    SMBUS_CHECK_PARAM_VOID(pDrvData->rxBuffer == NULL,
                          "SMBus: CRITICAL - rxBuffer is NULL in smbusMasterReadData! pDrvData=%p, rxBuffer=%p, rxLength=%d, interruptMode=%d",
                          pDrvData, pDrvData->rxBuffer, pDrvData->rxLength, pDrvData->sbrCfg.interruptMode);

    rxValid = smbusReadReg(&regBase->icRxflr);

    /* CRITICAL FIX: Correct boundary check - use <= instead of < to prevent overflow */
    /* Also check rxLength initialization to prevent negative indexing */
    if (pDrvData->rxLength >= SMBUS_MAX_BUFFER_SIZE) {
        LOGE("SMBus: CRITICAL - rxLength (%d) >= buffer size (%d), preventing overflow!\n",
             pDrvData->rxLength, SMBUS_MAX_BUFFER_SIZE);
        return;
    }

    LOGD("SMBus: Read data - rxValid=%d, current rxLength=%d, buffer_size=%d\n",
         rxValid, pDrvData->rxLength, SMBUS_MAX_BUFFER_SIZE);

    for (i = 0; i < rxValid && pDrvData->rxLength < SMBUS_MAX_BUFFER_SIZE; i++) {
        /* SAFETY: Store to valid array index (0 to SMBUS_MAX_BUFFER_SIZE-1) */
        pDrvData->rxBuffer[pDrvData->rxLength] = (U8)(regBase->icDataCmd.value & 0xFF);

        LOGD("SMBus: Stored data[0x%02X] at index [%d]\n",
             pDrvData->rxBuffer[pDrvData->rxLength], pDrvData->rxLength);

        pDrvData->rxLength++;

        /* Double-check after increment to prevent overflow */
        if (pDrvData->rxLength >= SMBUS_MAX_BUFFER_SIZE) {
            LOGW("SMBus: Buffer full, stopping read at %d bytes\n", pDrvData->rxLength);
            break;
        }
    }

    LOGD("SMBus: Read completed - total read=%d, rxValid=%d\n", pDrvData->rxLength, rxValid);
    pDrvData->rxComplete = 1;
}

/**
 * @brief Master transfer data to TX FIFO
 * @details Transfers data from TX buffer to TX FIFO
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @return void
 *
 */
void smbusMasterTransferData(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase)
{
    SmbusDev_s *dev = &pDrvData->pSmbusDev;
    U32 txAvailable, tx_limit;
    SmbusMsg_s *msg;
    U32 cmd = 0;
    U8 *buf = NULL;
    U32 buf_len = 0;

    if (!dev || dev->msgWriteIdx >= dev->msgsNum) {
        /* No more data to send, disable TX_EMPTY interrupt */
        LOGD("SMBus: No more data, disabling TX_EMPTY interrupt\n");
        LOGD("Smbus:msgWriteIdx:%d, msgNum:%d\r\n", dev->msgWriteIdx, dev->msgsNum);
        pDrvData->txComplete = 1;
        return;
    }

    /* CRITICAL FIX: Check if device is still in active transfer state */
    if (!(dev->status & SMBUS_STATUS_ACTIVE)) {
        /* Device is not active but we're getting TX_EMPTY interrupts */
        /* This indicates a race condition - disable TX_EMPTY interrupt */
        LOGW("SMBus: TX_EMPTY interrupt received but device not active (status=0x%08X) - disabling interrupt\n",
             dev->status);
        U32 currentMask = smbusReadReg(&regBase->icIntrMask.value);
        currentMask &= ~SMBUS_INTR_TX_EMPTY;
        smbusWriteReg(&regBase->icIntrMask.value, currentMask);
        pDrvData->txComplete = 1;
        return;
    }

    txAvailable = smbusReadReg(&regBase->icTxflr);
    tx_limit = dev->txFifoDepth - txAvailable;
    msg = &dev->msgs[dev->msgWriteIdx];

    /* Process current message */
    if (msg->flags == 0) { /* Write operation */
        /* Simple approach: send all data at once */
        buf = msg->buf;
        buf_len = msg->len;

        while (buf_len > 0 && tx_limit > 0) {
            cmd = *buf++;
            buf_len--;
            tx_limit--;

            /* Set STOP condition for last byte of last message */
            if (dev->msgWriteIdx == dev->msgsNum - 1 && buf_len == 0) {
                cmd |= SMBUS_IC_DATA_CMD_STOP;
                LOGD("SMBus: Setting STOP condition for last byte\n");
            }

            smbusWriteReg(&regBase->icDataCmd.value, cmd);
            LOGD("SMBus: Sent byte 0x%02X, remaining: %d\n", cmd & 0xFF, buf_len);
        }

        /* Mark current message as complete */
        dev->msgWriteIdx++;

        if (dev->msgWriteIdx >= dev->msgsNum) {
            /* All messages processed */
            pDrvData->txComplete = 1;
            LOGD("SMBus: All messages transferred, disabling TX_EMPTY\n");

            /* Disable TX_EMPTY interrupt to prevent continuous triggering */
            U32 currentMask = smbusReadReg(&regBase->icIntrMask.value);
            currentMask &= ~SMBUS_INTR_TX_EMPTY;
            smbusWriteReg(&regBase->icIntrMask.value, currentMask);
        }
    } else {
        /* Read operation - not implemented in this context */
        LOGE("SMBus: Read operation not supported in legacy TX handler\n");
        pDrvData->txComplete = 1;
    }
}

/**
 * @brief Handle master specific SMBus interrupts
 * @details Processes SMBus-specific interrupts in master mode
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @param[in] smbusIntrStat SMBus interrupt status
 * @return void
 *
 */
void smbusHandleMasterSpecificInterrupts(SmbusDrvData_s *pDrvData,
                                         volatile SmbusRegMap_s *regBase,
                                         U32 smbusIntrStat)
{

    /* Handle SMBus timeout interrupts */
    if (smbusIntrStat & (SMBUS_MST_CLOCK_EXTND_TIMEOUT_BIT | SMBUS_QUICK_CMD_DET_BIT)) {
        SmbusTimeoutType_e timeoutType = (smbusIntrStat & SMBUS_MST_CLOCK_EXTND_TIMEOUT_BIT) ?
                                       SMBUS_TIMEOUT_TYPE_CLOCK_EXTEND : SMBUS_TIMEOUT_TYPE_CLOCK_LOW;

        /* Trigger timeout event callback */
        U32 event = (timeoutType == SMBUS_TIMEOUT_TYPE_CLOCK_EXTEND) ?
                    SMBUS_EVENT_MST_CLK_EXT_TIMEOUT : SMBUS_EVENT_MST_CLK_LOW_TIMEOUT; 
        LOGW("SMBus Master timeout: type=%d, event=%d\n", timeoutType, event);
    }

    /* Clear SMBus interrupts */
    smbusWriteReg(&regBase->icClrSmbusIntr, smbusIntrStat);
}

/* ======================================================================== */
/*                    HAL Layer - I2C Compatible Probe Functions               */
/* ======================================================================== */

/**
 * @brief Calculate SMBus controller FIFO size
 * @details Detects and configures the TX and RX FIFO sizes by reading
 *          the component parameters. This function is compatible with
 *          I2C initialization and configures FIFO parameters for both
 *          master and target modes.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Compatible with I2C's i2c_dw_calc_fifo_size interface
 * @note Reads IC_COMP_PARAM_1 register to determine FIFO depths
 * @note Configures TX and RX FIFO depth in device structure
 * @note Supports FIFO depths from 2 to 256 per hardware specification
 * @note Required for proper interrupt threshold configuration
 */
S32 smbusCalcFifoSize(SmbusDev_s *dev)
{
    U32 param, tx_fifo_depth, rx_fifo_depth;

    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL, -EINVAL,
                            "SMBus: Invalid device structure");

    /*
     * Try to detect the FIFO depth if not set by interface driver,
     * the depth could be from 2 to 256 from HW spec.
     */
    param = smbusReadReg(&dev->regBase->icCompParam1);

    /* Extract FIFO depths from component parameters */
    tx_fifo_depth = ((param >> 16) & 0xff) + 1;
    rx_fifo_depth = ((param >> 8)  & 0xff) + 1;

    /* Configure device structure with detected FIFO depths */
    dev->txFifoDepth = tx_fifo_depth;
    dev->rxFifoDepth = rx_fifo_depth;

    LOGD("SMBus: FIFO size detected - TX: %u, RX: %u\n",
         tx_fifo_depth, rx_fifo_depth);

    return EXIT_SUCCESS;
}

/**
 * @brief Calculate SMBus master timing parameters
 * @details Calculates SCL high/low count and SDA hold time for master mode
 *          based on the configured clock speed. Uses DW I2C recommended formula.
 * @param[in] dev Pointer to SMBus device structure
 * @return void
 *
 * @note SDA TX hold time formula: 0.15 × ic_clk / (2 × speed)
 * @note ic_clk is the internal peripheral clock (e.g., 12.5MHz)
 * @note This is derived from: base_lcnt × correction_factor / 2
 *       where base_lcnt = ic_clk/(2×speed), correction = 6/(5×4) = 0.3
 *       final: 0.3/2 = 0.15
 */
static void smbusCalcTimingsMaster(SmbusDev_s *dev)
{
    U32 ic_clk;  /* Internal peripheral clock frequency (12.5MHz) */
    U32 speedInHz = dev->clkRate;

    SMBUS_CHECK_PARAM_VOID(dev == NULL, "SMBus: Invalid device for timings calculation");
    
    /* Initialize all timing counters to zero */
    SMBUS_INIT_TIMING_COUNTERS(dev);
    ic_clk = 12500000;  /* Fixed internal clock frequency for timing calculations */
    LOGW("SMBus: clock rate %u Hz, smbusCalcTimingsMaster\n", dev->clkRate);

    peripsClockFreqGet(dev->channelNum, &dev->clkRate);  /* Use actual clock rate from device configuration */
    U32 sclLcnt;
    U32 temp;
    U32 lowDutyPercent = 50; /* Default 50% duty cycle */
    U16 fsSpklen;
    
    /* Set spike suppression length based on speed */
    if (speedInHz > SMBUS_HS_MIN_SPEED) {
        fsSpklen = SMBUS_HS_DEFAULT_SPKLEN;  /* 1 */
    } else if (speedInHz > SMBUS_FS_MIN_SPEED) {
        fsSpklen = SMBUS_FS_DEFAULT_SPKLEN;  /* 2 */
    } else {
        fsSpklen = SMBUS_SS_DEFAULT_SPKLEN;  /* 11 */
    }

    /* Store spike suppression values */
    dev->fsSpklen = fsSpklen;
    dev->hsSpklen = (speedInHz > SMBUS_HS_MIN_SPEED) ? SMBUS_HS_DEFAULT_SPKLEN : SMBUS_FS_DEFAULT_SPKLEN;

    /* Calculate SCL low count: <lcount> = <internal clock> / <speed, Hz> */
    sclLcnt = dev->clkRate / speedInHz;
    LOGE("SMBus: Calculating timings for speed %u Hz, ic_clk=%u, sclhnt=%u Hz\n", speedInHz, ic_clk, sclLcnt); 
    /*
     * IC_CLK_FREQ_OPTIMIZATION = 0
     * SCL_Low_time = [(LCNT + 1) * ic_clk] - SCL_Fall_time + SCL_Rise_time
     * LCNT = (ic_clk * duty) / speedInHz -1;
     * ignore SCL_Fall_time and SCL_Rise_time
     * assume SCL_Fall_time = 300 ns, SCL_Rise_time = 300ns
     */
    temp = (lowDutyPercent * sclLcnt) / 100 - 1;
    if (temp < (fsSpklen + 7)) {
        temp = fsSpklen + 7;
    }

    /* Set low count for all modes based on speed */
    if (speedInHz <= SMBUS_SS_MAX_SPEED) {
        dev->ssLcnt = temp;
    }
    if (speedInHz >= SMBUS_FS_MIN_SPEED && speedInHz <= SMBUS_FS_MAX_SPEED) {
        dev->fsLcnt = temp;
    }
    if (speedInHz >= SMBUS_HS_MIN_SPEED) {
        dev->hsLcnt = temp;
    }
    if (speedInHz > SMBUS_FS_MAX_SPEED) {
        dev->fpLcnt = temp;
    }

    /*
     * IC_CLK_FREQ_OPTIMIZATION = 0
     * SCL_High_time = [(HCNT + IC_*_SPKLEN + 7) * ic_clk] + SCL_Fall_time
     * HCNT = (ic_clk * duty) / speedInHz - 7 - SPKLEN ;
     */
    temp = ((100 - lowDutyPercent) * sclLcnt) / 100 - 7 - fsSpklen;
    if (temp < (fsSpklen + 5)) {
        temp = fsSpklen + 5;
    }

    /* Set high count for all modes based on speed */
    if (speedInHz <= SMBUS_SS_MAX_SPEED) {
        dev->ssHcnt = temp;
    }
    if (speedInHz >= SMBUS_FS_MIN_SPEED && speedInHz <= SMBUS_FS_MAX_SPEED) {
        dev->fsHcnt = temp;
    }
    if (speedInHz >= SMBUS_HS_MIN_SPEED) {
        dev->hsHcnt = temp;
    }
    if (speedInHz > SMBUS_FS_MAX_SPEED) {
        dev->fpHcnt = temp;
    }

    /* For compatibility with higher speeds, also configure lower speed modes */
    if (speedInHz > SMBUS_FS_MAX_SPEED) {
        /* Fast mode plus or high speed - configure fast mode as fallback */
        U32 fallbackSpeed = SMBUS_FS_MAX_SPEED;
        U32 fallbackSclLcnt = ic_clk / fallbackSpeed;
        U16 fallbackSpklen = SMBUS_FS_DEFAULT_SPKLEN;

        /* Calculate fallback low count */
        temp = (lowDutyPercent * fallbackSclLcnt) / 100 - 1;
        if (temp < (fallbackSpklen + 7)) {
            temp = fallbackSpklen + 7;
        }
        dev->fsLcnt = temp;

        /* Calculate fallback high count */
        temp = ((100 - lowDutyPercent) * fallbackSclLcnt) / 100 - 7 - fallbackSpklen;
        if (temp < (fallbackSpklen + 5)) {
            temp = fallbackSpklen + 5;
        }
        dev->fsHcnt = temp;
    }
    /* 
     * Calculate SDA TX hold time using formula:
     * IC_SDA_TX_HOLD = 0.15 × ic_clk / (2 × speed)
     * 
     * Derivation:
     * - Base SCL low count = ic_clk / (2 × speed)
     * - Correction factor = 6/(5×4) = 0.3
     * - Final hold time = base × 0.3 / 2 = base × 0.15
     * 
     * Example with ic_clk=12.5MHz, speed=100kHz:
     * sdaHoldTime = (15 × 12,500,000) / (200 × 100,000) = 9 cycles
     * Time = 9 / 12.5MHz = 720ns
     */
    
    if (speedInHz > 0 && dev->clkRate > 0) {
        /* 
         * Formula: sdaHoldTime = (0.15 × ic_clk) / (2 × speed)
         * Use fixed-point: (15 × ic_clk) / (200 × speed) to avoid float
         * Add rounding: + (100 × speed) before division
         */
        dev->sdaHoldTime = (15 * dev->clkRate + 100 * speedInHz) / (200 * speedInHz);
        
        /* Ensure minimum value of 1 */
        if (dev->sdaHoldTime == 0) {
            dev->sdaHoldTime = 1;
        }
        
        LOGD("SMBus: SDA TX hold = (15x%u)/(200x%u) = %u cycles (~%u ns)\n",
             ic_clk, dev->clkRate, dev->sdaHoldTime, 
             (dev->sdaHoldTime * 1000) / (ic_clk / 1000000));
    } else {
        /* Fallback: use minimal safe value */
        dev->sdaHoldTime = 1;
        LOGW("SMBus: IC_CLK or clkRate not configured, using minimal SDA hold time\n");
    }
    
    LOGD("SMBus: Timing - SS:%u/%u FS:%u/%u FP:%u/%u HS:%u/%u SDA_HOLD:%u\n",
         dev->ssHcnt, dev->ssLcnt, dev->fsHcnt, dev->fsLcnt,
         dev->fpHcnt, dev->fpLcnt, dev->hsHcnt, dev->hsLcnt, dev->sdaHoldTime);
}

/**
 * @brief Configure SMBus master mode
 * @details Configures the SMBus controller for master mode operation.
 *          This function sets up the master configuration register with
 *          appropriate speed, addressing mode, and control bits.
 * @param[in] dev Pointer to SMBus device structure
 * @return void
 *
 * @note Compatible with I2C's i2c_dw_configure_master interface
 * @note Sets master mode, disables target mode, enables restart
 * @note Configures speed based on clock rate setting
 * @note Sets addressing mode (7-bit or 10-bit)
 */
static void smbusConfigureMaster(SmbusDev_s *dev)
{
    U32 masterCfg = 0;

    SMBUS_CHECK_PARAM_VOID(dev == NULL || dev->regBase == NULL,
                          "SMBus: Invalid device for master configuration");

    /* Configure master mode - compatible with I2C DW_IC_CON settings */
    masterCfg |= (1U << SMBUS_IC_CON_MASTER_MODE_EN_BIT);  /* Master mode enable */
    masterCfg |= (1U << SMBUS_IC_CON_TARGET_DISABLE_BIT);   /* target disable */
    masterCfg |= (1U << SMBUS_IC_CON_RESTART_EN_BIT);      /* Restart enable */

    /* Set addressing mode */
    if (dev->addrMode == 1) {
        masterCfg |= (1U << SMBUS_IC_CON_10BIT_MASTER_ADDR_BIT);  /* 10-bit master addressing */
    }

    /* Set speed based on clock rate using proper macro definitions */
    if (dev->clkRate >= SMBUS_SPEED_HIGH_THRESHOLD) {
        masterCfg |= SMBUS_SPEED_HIGH_CFG;      /* High speed (3.4 MHz) */
    } else if (dev->clkRate >= SMBUS_SPEED_FAST_PLUS_THRESHOLD) {
        masterCfg |= SMBUS_SPEED_FAST_PLUS_CFG; /* Fast Plus speed (1 MHz) */
    } else if (dev->clkRate >= SMBUS_SPEED_FAST_THRESHOLD) {
        masterCfg |= SMBUS_SPEED_FAST_CFG;      /* Fast speed (400 kHz) */
    } else {
        masterCfg |= SMBUS_SPEED_STANDARD_CFG;  /* Standard speed (100 kHz) */
    }

    /* Calculate and configure SMBus timing parameters */
    smbusCalcTimingsMaster(dev);

    /* Store configuration in device structure */
    dev->masterCfg = masterCfg;
    dev->mode = DW_SMBUS_MODE_MASTER;

    LOGD("SMBus: Master configuration: 0x%08X, speed: %u Hz\n",
         masterCfg, dev->clkRate);
}

/**
 * @brief Configure SMBus target mode
 * @details Configures the SMBus controller for target mode operation.
 *          This function sets up the target configuration register with
 *          appropriate control bits and addressing mode.
 * @param[in] dev Pointer to SMBus device structure
 * @return void
 *
 * @note Compatible with I2C's i2c_dw_configure_TARGET interface
 * @note Enables target mode, configures FIFO hold control
 * @note Sets addressing mode and restart support
 * @note Configures stop detection when addressed
 */
static void smbusConfigureTarget(SmbusDev_s *dev)
{
    U32 targetCfg = 0;

    SMBUS_CHECK_PARAM_VOID(dev == NULL || dev->regBase == NULL,
                          "SMBus: Invalid device for target configuration");

    /* Configure target mode - compatible with smbusProbeTarget settings */
    targetCfg &= ~(1U << SMBUS_IC_CON_MASTER_MODE_EN_BIT);  /* Master mode disable */
    targetCfg &= ~(1U << SMBUS_IC_CON_TARGET_DISABLE_BIT);   /* target enable */
    targetCfg |= (1U << SMBUS_IC_CON_RESTART_EN_BIT);      /* Restart enable for combined transactions */

    /* Set speed based on clock rate using proper macro definitions */
    if (dev->clkRate >= SMBUS_SPEED_HIGH_THRESHOLD) {
        targetCfg |= SMBUS_SPEED_HIGH_CFG;      /* High speed (3.4 MHz) */
    } else if (dev->clkRate >= SMBUS_SPEED_FAST_PLUS_THRESHOLD) {
        targetCfg |= SMBUS_SPEED_FAST_PLUS_CFG; /* Fast Plus speed (1 MHz) */
    } else if (dev->clkRate >= SMBUS_SPEED_FAST_THRESHOLD) {
        targetCfg |= SMBUS_SPEED_FAST_CFG;      /* Fast speed (400 kHz) */
    } else {
        targetCfg |= SMBUS_SPEED_STANDARD_CFG;  /* Standard speed (100 kHz) */
    }

    /* Set address mode */
    if (dev->addrMode == 1) {
        targetCfg |= (1U << SMBUS_IC_CON_10BIT_TARGET_ADDR_BIT);  /* 10-bit target addressing */
    }

    /* Configure SMBus features for target mode */
    targetCfg |= (1U << SMBUS_IC_CON_RX_FIFO_HOLD_BIT);         /* RX FIFO full hold control */
    targetCfg |= (1U << SMBUS_IC_CON_STOP_DET_IFADDRESSED_BIT); /* STOP detection when addressed */

    targetCfg |= (dev->smbFeatures.arpEnb << SMBUS_IC_CON_ARP_ENABLE_BIT);     /* Bit 18 = 1 (ARP enabled) */
    targetCfg |= (dev->smbFeatures.quickCmdEnb << SMBUS_IC_CON_QUICK_CMD_BIT);  /* Bit 17 = 1 (Quick command enable) */
    /* Store configuration in device structure */
    dev->targetCfg = targetCfg;
    dev->mode = SMBUS_MODE_TARGET;

    LOGD("SMBus: target configuration: 0x%08X, address: 0x%02X\n",
         targetCfg, dev->targetAddr);
}

/**
 * @brief SMBus master probe function compatible with I2C initialization
 * @details Configures SMBus controller in master mode, calculates timing,
 *          detects FIFO size, and enables interrupts. This function is
 *          compatible with I2C initialization interface and can be called
 *          from SMBus device initialization to enable I2C functionality.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Compatible with I2C's i2c_dw_probe_master interface
 * @note Configures master mode timing and FIFO parameters
 * @note Sets up interrupt handling for master operations
 * @note Enables controller after configuration
 * @warning This function should only be called in Master mode
 */
S32 smbusProbeMaster(SmbusDev_s *dev)
{
    S32 ret;
    U32 ic_con;

    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL, -EINVAL,
                            "SMBus: Invalid device structure");
    
    /* Step 1: Configure master mode parameters (like i2c_dw_configure_master) */
    smbusConfigureMaster(dev);

    /* Step 2: Calculate FIFO size (like i2c_dw_calc_fifo_size) */
    ret = smbusCalcFifoSize(dev);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: FIFO size calculation failed: %d\n", ret);
        return ret;
    }
    
    smbusDisable(dev);
    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)dev->regBase;
    /*
     * On AMD platforms BIOS advertises the bus clear feature
     * and enables the SCL/SDA stuck low. SMU FW does the
     * bus recovery process. Driver should not ignore this BIOS
     * advertisement of bus clear feature.
     */
    ic_con = smbusReadReg(&regBase->icCon.value);
    if (ic_con & (1U << 11)) {  /* DW_IC_CON_BUS_CLEAR_CTRL */
        dev->masterCfg |= (1U << 11);  /* DW_IC_CON_BUS_CLEAR_CTRL */
    }

    /* Step 3: Apply master configuration to hardware */
    smbusWriteReg(&regBase->icCon.value, dev->masterCfg);

    /* Step 4: Configure SMBus-specific features for master mode */
    ic_con = regBase->icCon.value & ~0x60000U;  /* Clear SMBus feature bits */

    ic_con |= (dev->smbFeatures.arpEnb         << SMBUS_IC_CON_ARP_ENABLE_BIT) |   /* ARP */
              (dev->smbFeatures.quickCmdEnb    << SMBUS_IC_CON_QUICK_CMD_BIT);    /* Quick Command */            
  
    smbusWriteReg(&regBase->icCon.value, ic_con);

    /* Step 5: Configure timing parameters in hardware registers */
    /* Write standard speed timing parameters */
    if (dev->ssHcnt && dev->ssLcnt) {
        smbusWriteReg(&regBase->icSsSclHcnt, dev->ssHcnt);
        smbusWriteReg(&regBase->icSsSclLcnt, dev->ssLcnt);
        LOGD("SMBus: Standard timing set - HCNT:%u, LCNT:%u\n", dev->ssHcnt, dev->ssLcnt);
    }

    /* Write fast mode/fast mode plus timing parameters */
    if (dev->fsHcnt && dev->fsLcnt) {
        smbusWriteReg(&regBase->icFsSclHcnt, dev->fsHcnt);
        smbusWriteReg(&regBase->icFsSclLcnt, dev->fsLcnt);
        LOGD("SMBus: Fast timing set - HCNT:%u, LCNT:%u\n", dev->fsHcnt, dev->fsLcnt);
    }

    /* Write high speed timing parameters */
    if (dev->hsHcnt && dev->hsLcnt) {
        smbusWriteReg(&regBase->icHsSclHcnt, dev->hsHcnt);
        smbusWriteReg(&regBase->icHsSclLcnt, dev->hsLcnt);
        LOGD("SMBus: High speed timing set - HCNT:%u, LCNT:%u\n", dev->hsHcnt, dev->hsLcnt);
    }

    /* Write SDA hold time if supported */
    if (dev->sdaHoldTime) {
        smbusWriteReg(&regBase->icSdaHold, dev->sdaHoldTime);
        LOGD("SMBus: SDA hold time set: %u\n", dev->sdaHoldTime);
    }

    /* Step 6: Set TX/RX FIFO thresholds for interrupt generation */
    smbusWriteReg(&regBase->icTxTl, 0);  /* TX threshold: generate interrupt when TX FIFO is empty */
    smbusWriteReg(&regBase->icRxTl, 0);  /* RX threshold: generate interrupt when RX FIFO has 1+ bytes */

    /* Step 7: Clear interrupt mask initially */
    smbusWriteReg(&regBase->icIntrMask.value, 0);

    /* Step 8: Enable controller */
    smbusEnable(dev);

    LOGD("SMBus: Master probe completed successfully\n");
    return EXIT_SUCCESS;
}

/**
 * @brief SMBus target probe function compatible with I2C initialization
 * @details Configures SMBus controller in target mode, calculates timing,
 *          detects FIFO size, and enables interrupts. This function is
 *          compatible with I2C initialization interface and can be called
 *          from SMBus device initialization to enable I2C functionality.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Compatible with I2C's i2c_dw_probe_TARGET interface
 * @note Configures target mode timing and FIFO parameters
 * @note Sets up interrupt handling for target operations
 * @note Allocates target buffer memory
 * @note Enables controller after configuration
 * @warning This function should only be called in target mode
 */
S32 smbusProbeTarget(SmbusDev_s *dev)
{
    S32 ret;

    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL, -EINVAL,
                            "SMBus: Invalid device structure");

    /* Calculate FIFO size for target operations */
    ret = smbusCalcFifoSize(dev);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: FIFO size calculation failed: %d\n", ret);
        return ret;
    }
    
    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)dev->regBase;

    /* target mode initialization */
    if (dev->mode == SMBUS_MODE_TARGET) {
        LOGD("=== Starting target Mode Initialization ===\n");

        /* Verify if targetCfg has been properly configured */
        if (dev->targetCfg == 0) {
            LOGW("SMBus: targetCfg not configured, using default target settings\n");
            /* Use default target mode configuration consistent with smbusConfigureTarget */
            dev->targetCfg = (0U << SMBUS_IC_CON_MASTER_MODE_EN_BIT) |      /* Bit 0 = 0 (target mode) */
                           (2U << 1) |                                      /* Bit 1:2 = 10 (Fast mode 400kHz) */
                           (0U << SMBUS_IC_CON_TARGET_DISABLE_BIT) |        /* Bit 6 = 0 (target enabled) */
                           (1U << SMBUS_IC_CON_RESTART_EN_BIT) |           /* Bit 5 = 1 (Restart enable) */
                           (1U << SMBUS_IC_CON_RX_FIFO_HOLD_BIT) |         /* Bit 9 = 1 (RX FIFO hold control) */
                           (1U << SMBUS_IC_CON_STOP_DET_IFADDRESSED_BIT) | /* Bit 7 = 1 (STOP detection when addressed) */
                           (dev->smbFeatures.arpEnb << SMBUS_IC_CON_ARP_ENABLE_BIT) |     /* Bit 18 = 1 (ARP enabled) */
                           (dev->smbFeatures.quickCmdEnb << SMBUS_IC_CON_QUICK_CMD_BIT);  /* Bit 17 = 1 (Quick command enable) */
            LOGD("Applied default target configuration: targetCfg = 0x%08X\n", dev->targetCfg);
        } else {
            LOGD("Using pre-configured targetCfg = 0x%08X\n", dev->targetCfg);
        }

        /* 1. Must disable device first before modifying CON and SAR */
        smbusWriteReg(&regBase->icEnable.value, 0);
        LOGD("Device disabled (IC_ENABLE = 0x%08X)\n", regBase->icEnable.value);

        /* Poll waiting for device to be actually disabled (this is Synopsys recommended practice to prevent active transfers) */
        int timeout = 1000;
        while ((regBase->icEnableStatus & 0x1) && --timeout > 0);
        if (timeout == 0) LOGW("Warning: Timeout waiting for IC disable\n");

        /* 2. Configure all registers while disabled */
        smbusWriteReg(&regBase->icCon.value, dev->targetCfg);
        LOGD("IC_CON configured using pre-configured targetCfg = 0x%08X\n", regBase->icCon.value);

        /* Set slave address - prioritize dynamic parameters, use default when not set */
        if (dev->targetAddr != 0) {
            SmbusIcSarReg_u sar;
            sar.value = smbusReadReg(&regBase->icSar.value);
            sar.fields.icSar = dev->targetAddr;
            smbusWriteReg(&regBase->icSar.value, sar.value);
            LOGI("IC_SAR set to dynamic address: 0x%02X\n", regBase->icSar.fields.icSar);
        } else {
            regBase->icSar.fields.icSar = SMBUS_TARGET_DEFAULT_ADDR;
            LOGD("IC_SAR set to default address: 0x%02X\n", regBase->icSar.fields.icSar);
        }

        /* Set TX/RX FIFO thresholds for interrupt generation */
        smbusWriteReg(&regBase->icTxTl, 0);  /* TX threshold: generate interrupt when TX FIFO is empty */
        smbusWriteReg(&regBase->icRxTl, 0);  /* RX threshold: generate interrupt when RX FIFO has 1+ bytes */
        LOGD("FIFO thresholds set: TX_TL=%d, RX_TL=%d\n", regBase->icTxTl, regBase->icRxTl);

        /* Configure target mode interrupt mask - configure before enabling device */
        /* TX_EMPTY will be dynamically enabled as needed during WR_REQ processing */
        U32 intrMask = SMBUS_TARGET_INIT_INTR_MASK;
        smbusWriteReg(&regBase->icIntrMask.value, intrMask);
        U32 smbusIntrMask = 0;
        if (dev->smbFeatures.arpEnb) {
            smbusIntrMask |= SMBUS_ARP_INTR_MASK; /* Include ARP_DET etc */
            regBase->icAckGeneralCall |= SMBUS_DEFAULT_GC_ACK;
            LOGD("ARP Enabled: Setting SMBus Intr Mask for ARP\n");
        }
        if (dev->smbFeatures.smbAlertEnb) {
            smbusIntrMask |= SMBUS_ALERT_DET_BIT;
        }
        if (dev->smbFeatures.hostNotifyEnb) {
            smbusIntrMask |= SMBUS_HOST_NOTIFY_MST_DET_BIT;
        }
        if (dev->smbFeatures.quickCmdEnb) {
            /* Quick Command is usually part of protocol interrupts */
            smbusIntrMask |= SMBUS_QUICK_CMD_DET_BIT;
        }
        /* Apply SMBus specific interrupt mask */
        if (smbusIntrMask != 0) {
            smbusWriteReg(&regBase->icSmbusIntrMask.value, smbusIntrMask);
        } else {
             /* Default behavior, or keep as 0 */
             smbusWriteReg(&regBase->icSmbusIntrMask.value, 0);
        }

        LOGE("FINAL OVERRIDE: target interrupt mask FORCED to 0x%08X (TX_EMPTY initially disabled)\n",
             regBase->icIntrMask.value);
        LOGD("target mode smbus initialized with TX_EMPTY disabled - will be enabled on demand:%08X\n", 
              regBase->icSmbusIntrMask.value );
    
        if (dev->smbFeatures.arpEnb) regBase->icSmbusIntrMask.value |= SMBUS_ARP_INTR_MASK;
        LOGD("✓ target interrupt mask configured: 0x%08X (WR_REQ|RX_FULL|RD_REQ|TX_ABRT|RX_DONE|STOP_DET)\n",
             regBase->icIntrMask.value);

        /* 3. Finally: uniformly enable device */
        U32 icEnable = 0;
        icEnable |= SMBUS_IC_ENABLE_MASK;   /* Enable */
        icEnable |= SMBUS_IC_SAR_ENABLE_MASK;  /* SMBus target TX Enable */

        smbusWriteReg(&regBase->icEnable.value, icEnable);
        LOGD("SMBus: target probe completed, Device Enabled=0x%x\n", regBase->icEnable.value);

        /* Verify if configuration succeeded */
        udelay(100);  /* Short delay to ensure hardware stability */

        LOGD("=== target Initialization Verification ===\n");
        LOGD("Final IC_CON    = 0x%08X\n", regBase->icCon.value);
        LOGD("Final IC_SAR    = 0x%02X\n", regBase->icSar.fields.icSar);
        LOGD("Final IC_ENABLE = 0x%08X\n", regBase->icEnable.value);
        LOGD("Final IC_INTR_MASK = 0x%08X\n", regBase->icIntrMask.value);
        LOGD("Final IC_INTR_SMBUS_MASK = 0x%08X\n", regBase->icSmbusIntrMask.value);
        LOGD("Final IC_TX_TL  = %d\n", regBase->icTxTl);
        LOGD("Final IC_RX_TL  = %d\n", regBase->icRxTl);
    } 
    LOGD("SMBus: target probe completed successfully\n");
    return EXIT_SUCCESS;
}

/**
 * @brief SMBus master unprobe function for cleanup
 * @details Disables SMBus controller, clears configuration, and removes
 *          interrupt handlers for master mode. This function performs
 *          cleanup operations when shutting down master mode.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Compatible with I2C's i2c_dw_unprobe_master interface
 * @note Disables controller and clears master configuration
 * @note Removes interrupt handlers and synchronization objects
 * @note Performs graceful shutdown of master operations
 * @warning This function should only be called in Master mode
 */
S32 smbusUnprobeMaster(SmbusDev_s *dev)
{
    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL, -EINVAL,
                            "SMBus: Invalid device structure");

    /* Disable controller */
    smbusDisable(dev);

    /* Clear master configuration */
    dev->masterCfg = 0;

    /* Remove synchronization objects if they exist */
    smbusDeleteSemaphore(dev, "SMBus Master Unprobe");

    LOGD("SMBus: Master unprobe completed successfully\n");
    return EXIT_SUCCESS;
}
/**
 * @brief SMBus target unprobe function for cleanup
 * @details Disables SMBus controller, clears configuration, and removes
 *          interrupt handlers for target mode. This function performs
 *          cleanup operations when shutting down target mode.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Compatible with I2C's i2c_dw_unprobe_TARGET interface
 * @note Disables controller and clears target configuration
 * @note Removes interrupt handlers and frees target buffers
 * @note Performs graceful shutdown of target operations
 * @warning This function should only be called in target mode

 */
S32 smbusUnprobeTarget(SmbusDev_s *dev)
{
    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL, -EINVAL,
                            "SMBus: Invalid device structure");

    /* Disable controller */
    smbusDisable(dev);

    /* Clear target configuration */
    dev->targetCfg = 0;
    /* Note: target buffer deallocation is handled in smbusDeInit function */

    LOGD("SMBus: target unprobe completed successfully\n");
    return EXIT_SUCCESS;
}

/**
 * @brief Calculate timing parameters and disable controller for mode switch
 * @param[in] dev Pointer to SMBus device structure
 * @param[out] ti2cPoll Polling delay in microseconds
 * @param[out] maxTPollCount Maximum polling count for timeout
 * @return 0 on success, negative error code on failure
 * @retval 0 Success
 * @retval -EINVAL Invalid speed setting
 * @retval -ETIMEDOUT Controller disable timeout
 */
static S32 smbusCalculateTimingAndDisable(SmbusDev_s *dev, U32 *ti2cPoll, U32 *maxTPollCount)
{
    U32 speedVal;
    U32 enableSts;
    U32 i;

    /* Read current speed configuration and calculate timing parameters */
    speedVal = dev->regBase->icCon.value & SMBUS_IC_CON_SPEED_MASK;

    /* Calculate delay parameters based on current speed */
    switch (speedVal) {
        case SMBUS_IC_CON_SPEED_STD:
            *ti2cPoll = SMBUS_MODE_SWITCH_STD_SPEED_DELAY_US;
            *maxTPollCount = SMBUS_MODE_SWITCH_STD_SPEED_TIMEOUT_CNT;
            break;
        case SMBUS_IC_CON_SPEED_FAST:
            *ti2cPoll = SMBUS_MODE_SWITCH_FAST_SPEED_DELAY_US;
            *maxTPollCount = SMBUS_MODE_SWITCH_FAST_SPEED_TIMEOUT_CNT;
            break;
        case SMBUS_IC_CON_SPEED_HIGH:
            *ti2cPoll = SMBUS_MODE_SWITCH_HIGH_SPEED_DELAY_US;
            *maxTPollCount = SMBUS_MODE_SWITCH_HIGH_SPEED_TIMEOUT_CNT;
            break;
        default:
            LOGE("SMBus: Invalid speed setting 0x%X for mode switch\n", speedVal);
            return -EINVAL;
    }

    /* Disable controller */
    smbusDisable(dev);

    /* Wait for controller to be fully disabled with timing based on speed */
    for (i = 0; i < *maxTPollCount; i++) {
        enableSts = smbusReadReg(&dev->regBase->icEnableStatus);
        enableSts = enableSts & SMBUS_IC_ENABLE_STATUS_IC_EN;
        if (!enableSts) {
            LOGD("SMBus: Controller disabled after %u attempts (delay=%u us)\n", i + 1, *ti2cPoll);
            break;
        }
        /* Delay based on speed setting */
        udelay(*ti2cPoll);
    }

    /* Check if disable failed */
    if (i == *maxTPollCount) {
        LOGE("SMBus: Failed to disable controller within timeout\n");
        /* Restore enable state and return error */
        smbusEnable(dev);
        return -ETIMEDOUT;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief SMBus Master/target mode switch core function
 * @details Implements the core LOGEc for switching between Master and target modes
 *          at the HAL layer. This function handles the hardware reconfiguration
 *          required to safely switch between operational modes.
 * @param[in] dev Pointer to SMBus device structure
 * @param[in] targetMode Target mode to switch to (SMBUS_MODE_MASTER or SMBUS_MODE_TARGET)
 * @return EXIT_SUCCESS on successful mode switch, negative error code on failure:
 *         -EINVAL: Invalid parameters (NULL dev or invalid target mode)
 *         -EBUSY: Bus is busy and cannot be switched
 *         -EIO: Hardware configuration failed
 *         -ETIMEDOUT: Bus idle wait timeout
 *
 * @note Disables controller before reconfiguration
 * @note Waits for bus to be idle before mode switch
 * @note Configures controller for target mode (master or target)
 * @note Re-enables controller after successful reconfiguration
 * @note Handles both master-to-target and target-to-master transitions
 * @warning This function should be called with proper device locking
 * @warning Controller is temporarily disabled during mode switch
 * @warning All ongoing transfers will be aborted during mode switch
 */
static S32 smbusModeSwitchCore(SmbusDev_s *dev, SmbusMode_e targetMode)
{
    S32 ret = EXIT_SUCCESS;
    U32 ti2cPoll, maxTPollCount;

    SMBUS_CHECK_PARAM(dev == NULL || dev->regBase == NULL, -EINVAL,
                    "SMBus: Invalid device structure for mode switch");

    SMBUS_CHECK_PARAM(targetMode != DW_SMBUS_MODE_MASTER && targetMode != DW_SMBUS_MODE_TARGET,
                      -EINVAL, "SMBus: Invalid target mode %d for mode switch", targetMode);

    /* Check if already in target mode */
    if (dev->mode == targetMode) {
        LOGD("SMBus: Already in %s mode, no switch needed\n",
             targetMode == DW_SMBUS_MODE_MASTER ? "master" : "target");
        ret = EXIT_SUCCESS;
        goto exit;
    }

    LOGD("SMBus: Switching from %s to %s mode\n",
         dev->mode == DW_SMBUS_MODE_MASTER ? "master" : "target",
         targetMode == DW_SMBUS_MODE_MASTER ? "master" : "target");

    /* Step 1: Wait for bus to be idle before disable */
    ret = smbusWaitBusNotBusy(dev);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: Bus busy during mode switch\n");
        ret = -EBUSY;
        goto exit;
    }

    /* Step 2-5: Calculate timing parameters and disable controller */
    ret = smbusCalculateTimingAndDisable(dev, &ti2cPoll, &maxTPollCount);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* Step 6: Switch mode in software */
    dev->mode = targetMode;

    /* Step 7: Configure hardware for the target mode */
    if (targetMode == DW_SMBUS_MODE_MASTER) {
        /* Switch to master mode */
        smbusConfigureMaster(dev);

        /* Apply master configuration to hardware */
        smbusWriteReg(&dev->regBase->icCon.value, dev->masterCfg);
        LOGD("SMBus: Configured for master mode, cfg=0x%08X\n", dev->masterCfg);
            /* Step 9: Configure proper interrupt mask for the target mode */
        /* CRITICAL FIX: Ensure RX_FULL is always enabled for read operations */
        U32 masterMask = SMBUS_INTR_TX_ABRT |      /* TX abort */
                           SMBUS_INTR_STOP_DET |     /* Stop detection */
                           SMBUS_INTR_RX_FULL;       /* RX FIFO Full - CRITICAL FIX */

        smbusWriteReg(&dev->regBase->icIntrMask.value, masterMask);
        LOGD("SMBus: Set Master mode interrupt mask=0x%08X\n", dev->regBase->icIntrMask.value);
    } else {
        /* Switch to target mode */
        smbusConfigureTarget(dev);

        /* Apply target configuration to hardware */
        smbusWriteReg(&dev->regBase->icCon.value, dev->targetCfg);

        /* Set target address for target mode 
         * Set target Address DIRECTLY (Do not use helper that enables HW) 
         */
        SmbusIcSarReg_u sar;
        sar.value = smbusReadReg(&dev->regBase->icSar.value);
        sar.fields.icSar = dev->targetAddr;
        smbusWriteReg(&dev->regBase->icSar.value, sar.value);
        
        LOGD("SMBus: Configured for target mode, cfg=0x%08X, addr=0x%02X\n",
             dev->targetCfg, dev->targetAddr);
        /* target mode: Enable only interrupts needed for target operations */
        smbusWriteReg(&dev->regBase->icIntrMask.value, SMBUS_TARGET_INTR_MASK);

        LOGE("✓ target mode interrupt mask set: 0x%08X (expected: 0x2F4)\n", dev->regBase->icIntrMask.value);

        /* Verify the write was successful */
        U32 readBack = smbusReadReg(&dev->regBase->icIntrMask.value);
        if (readBack != SMBUS_TARGET_INTR_MASK) {
            LOGE("✗ target interrupt mask write failed! Expected: 0x%08X, Read: 0x%08X\n", SMBUS_TARGET_INTR_MASK, readBack);
        }
    }
    /* Step 8: Re-enable controller */
    smbusEnable(dev);
    LOGE("SMBus: Successfully switched to %s mode\n",
         targetMode == DW_SMBUS_MODE_MASTER ? "master" : "target");

exit:
    return ret;
}

/**
 * @brief SMBus read handler (hardware-driven flow)
 * @details Strictly follows DW I2C Databook flow:
 * RX_FULL interrupt -> drain hardware FIFO -> handle I2C_M_RECV_LEN -> continue
 * @param[in] pDrvData Pointer to driver data
 * @param[in] buf      Pointer to the RX accumulation buffer (pDrvData->rxBuffer)
 * @param[in] maxLen   Size of the RX buffer (pDrvData->rxBufferSize)
 * @return void
 */
static void smbusDwRead(SmbusDrvData_s *pDrvData, void *buf, U32 maxLen)
{
    volatile SmbusRegMap_s *regBase;
    SmbusDev_s *dev;
    SmbusMsg_s *msgs;
    U32 rxValid;
    U8 *rxBufPtr = (U8 *)buf;

    if (pDrvData == NULL || buf == NULL) return;

    dev = &pDrvData->pSmbusDev;
    regBase = dev->regBase;
    msgs = dev->msgs;

    SMBUS_CHECK_PARAM_VOID(regBase == NULL || msgs == NULL,
                      "regBase or msgs is NULL");

    /* Iterate through all read messages (Outer Loop) */
    for (; dev->msgReadIdx < dev->msgsNum; dev->msgReadIdx++) {
        U32 tmp;
        U32 flags = msgs[dev->msgReadIdx].flags;
        U32 msgTotalLen;

        /* Skip write messages */
        if (!(flags & SMBUS_M_RD)) {
            continue;
        }

        /* Get current accumulated length for this message */
        U32 currentRxOffset = pDrvData->rxLength;
        msgTotalLen = msgs[dev->msgReadIdx].len;

        /* Check HW FIFO level */
        rxValid = smbusReadReg(&regBase->icRxflr);

        /* Drain FIFO while data is available (Inner Loop) */
        while (rxValid > 0) {
            /* Safety check for buffer overflow */
            if (currentRxOffset >= maxLen) {
                dev->status |= SMBUS_ERR_RX_OVER;
                return;
            }

            /* Read data from hardware FIFO */
            tmp = smbusReadReg(&regBase->icDataCmd.value);
            tmp &= 0xFF;  /* Extract data byte */
            
            rxValid--; /* Local tracker decrement */
            
            /* Handle Block Read Length (Inlined smbusDwRecvLen logic) */
            if ((flags & SMBUS_M_RECV_LEN) && (currentRxOffset == 0)) {
                U8 len = (U8)tmp;
                
                /* Validate length */
                if (len == 0 || len > SMBUS_BLOCK_MAXLEN) {
                    len = 1; /* Fallback to prevent stall */
                    /* Log error but let it continue to finish transaction cleanly if possible */
                    LOGE("SMBus: Invalid Block Count %d\n", (U8)tmp);
                }

                /* Calculate total message length: LenByte + Data + (PEC) */
                msgTotalLen = len + ((flags & SMBUS_M_PEC) ? 2 : 1);
                
                /* Update message state */
                msgs[dev->msgReadIdx].len = msgTotalLen;
                msgs[dev->msgReadIdx].flags &= ~SMBUS_M_RECV_LEN;
                
                /* Update TX buffer len for the Resume logic in XferMsg */
                dev->masterTxBufLen = msgTotalLen - 1;

                /* Re-enable TX_EMPTY to pump remaining READ CMDs */
                if (!(regBase->icIntrMask.value & SMBUS_INTR_TX_EMPTY)) {
                    regBase->icIntrMask.value |= SMBUS_INTR_TX_EMPTY;
                }
            }

            /* Store received byte */
            rxBufPtr[currentRxOffset++] = (U8)tmp;
            
            /* Update global tracking */
            pDrvData->rxLength = currentRxOffset;
            if (dev->rxOutstanding > 0) {
                dev->rxOutstanding--;
            }

            /* Check if this specific message is fully received */
            if (currentRxOffset >= msgTotalLen) {
                /* Message complete! Copy to user buffer */
                if (msgs[dev->msgReadIdx].buf != NULL) {
                    memcpy(msgs[dev->msgReadIdx].buf, rxBufPtr, msgTotalLen);
                }
                
                /* Reset state for next message */
                dev->status &= ~SMBUS_STATUS_READ_IN_PROGRESS;
                pDrvData->rxLength = 0;
                
                /* Break inner loop to advance outer loop (dev->msgReadIdx++) */
                goto nextMsg;
            }
        }

        /* If we are here, FIFO is empty but message is NOT complete */
        dev->status |= SMBUS_STATUS_READ_IN_PROGRESS;
        return; /* Exit ISR, wait for more data */

nextMsg:
        /* Continue to next iteration of Outer Loop */
        continue;
    }

    /* Handle Ghost Bytes (Inlined smbusDrainGhostBytes logic) */
    if (dev->msgReadIdx >= dev->msgsNum) {
        if (regBase->icRxflr > 0) {
            while (regBase->icRxflr > 0) {
                (void)regBase->icDataCmd.value;
                if (dev->rxOutstanding > 0) dev->rxOutstanding--;
            }
        }
    }
}

/**
 * @brief Initiate and continue low level SMBus master read/write transaction
 * @details This function is called from smbusDwIsr to pump SmbusMsg_s messages
 *          into the TX buffer. Handles messages longer than FIFO depth by
 *          processing them in multiple interrupt cycles.
 * @param[in] dev Pointer to SMBus device structure
 * @return void
 *
 * @note Based on Linux i2c_dw_xfer_msg with SMBus-specific enhancements
 * @note Follows DW_apb_i2c 2.03a databook recommendations
 */
static void smbusDwXferMsg(SmbusDev_s *dev)
{
    volatile SmbusRegMap_s *regBase;
    SmbusMsg_s *msg;
    U32 txLimit, rxLimit;
    U32 addr;
    U32 bufLen;
    U8 *buf;
    Bool needRestart = FALSE;
    U32 cmd;
    U32 flr;

    if (!dev || !dev->regBase || !dev->msgs) return;

    regBase = dev->regBase;
    addr = dev->msgs[dev->msgWriteIdx].addr;

    /* Loop through messages */
    for (; dev->msgWriteIdx < dev->msgsNum; dev->msgWriteIdx++) {
        msg = &dev->msgs[dev->msgWriteIdx];

        /* Address consistency check */
        if (msg->addr != addr) {
            dev->msgErr = -EINVAL;
            break;
        }

        /* Initialize buffer pointers */
        if (!(dev->status & SMBUS_STATUS_WRITE_IN_PROGRESS)) {
            buf = msg->buf;
            bufLen = msg->len;
            if ((dev->masterCfg & SMBUS_IC_CON_RESTART_EN) && (dev->msgWriteIdx > 0)) {
                needRestart = TRUE;
            }
        } else {
            buf = dev->masterTxBuf;
            bufLen = dev->masterTxBufLen;
            /* Handle RECV_LEN resume adjustment */
            if ((msg->flags & SMBUS_M_RECV_LEN) && (msg->len < SMBUS_MAX_BLOCK_LEN)) {
                bufLen = (msg->len >= 1) ? (msg->len - 1) : 0;
            }
        }

        /* Dynamic FIFO limit calculation */
        flr = smbusReadReg(&regBase->icTxflr);
        txLimit = dev->txFifoDepth - flr;
        flr = smbusReadReg(&regBase->icRxflr);
        rxLimit = dev->rxFifoDepth - flr;

        /* Fill FIFO Loop */
        while (bufLen > 0 && txLimit > 0 && rxLimit > 0) {
            cmd = 0;

            /* Check Block Read Pause */
            if ((msg->flags & SMBUS_M_RECV_LEN) && (dev->rxOutstanding >= 1)) {
                /* Waiting for length byte - Pause TX and STOP INTERRUPT STORM */
                regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;
                goto saveState;
            }

            /* Stop Bit Logic */
            if (((dev->msgWriteIdx == dev->msgsNum - 1) || (msg->flags & SMBUS_M_STOP)) &&
                (bufLen == 1) && !(msg->flags & SMBUS_M_RECV_LEN)) {
                cmd |= SMBUS_IC_DATA_CMD_STOP;
            }

            /* Restart Bit Logic */
            if (needRestart) {
                cmd |= SMBUS_IC_DATA_CMD_RESTART;
                needRestart = FALSE;
            }

            /* Write to Hardware */
            if (msg->flags & SMBUS_M_RD) {
                if (dev->rxOutstanding >= dev->rxFifoDepth) break; 
                smbusWriteReg(&regBase->icDataCmd.value, cmd | SMBUS_IC_DATA_CMD_READ_CMD);
                dev->rxOutstanding++;
                rxLimit--;
            } else {
                cmd |= (*buf & 0xFF);
                smbusWriteReg(&regBase->icDataCmd.value, cmd);
                buf++;
            }
            txLimit--;
            bufLen--;
        }

saveState:
        /* Save state if message incomplete */
        if (bufLen > 0) {
            dev->status |= SMBUS_STATUS_WRITE_IN_PROGRESS;
            dev->masterTxBufLen = bufLen;
            if (buf != dev->masterTxBuf && buf != NULL) {
                U32 copyLen = (bufLen < sizeof(dev->masterTxBuf)) ? bufLen : sizeof(dev->masterTxBuf);
                memcpy(dev->masterTxBuf, buf, copyLen);
            }
            return;
        } else {
            dev->status &= ~SMBUS_STATUS_WRITE_IN_PROGRESS;
            dev->masterTxBufLen = 0;
        }
    }

    /* Disable TX_EMPTY when all messages queued */
    if (dev->msgWriteIdx >= dev->msgsNum) {
        regBase->icIntrMask.value &= ~SMBUS_INTR_TX_EMPTY;
    }
}
/**
 * @brief Unified I2C Transfer Interface
 * @details Handles locking, parameter validation, and calls low-level HAL.
 *          Supports array of messages for repeated-start sequences.
 * @param[in] dev SMBus device handle
 * @param[in] msgs Array of I2C messages
 * @param[in] num Number of messages in array
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
static S32 smbusI2cTransfer(SmbusDev_s *dev, SmbusMsg_s *msgs, S32 num)
{
    S32 ret = EXIT_SUCCESS;

    /* 1. Parameter Validation */
    SMBUS_CHECK_PARAM_RETURN(dev == NULL || msgs == NULL || num <= 0, -EINVAL,
                            "SMBus: Invalid parameters for I2C transfer");

    /* ============================================================
     * BRANCH: target MODE HANDLING
     * target mode: Simply operate on memory buffers (Producer/Consumer model)
     * No need to operate hardware registers to initiate transfers or wait for bus idle
     * ============================================================*/
    if (dev->mode == SMBUS_MODE_TARGET) {
        S32 i;
        LOGD("SMBus: Processing target transfer request (%d msgs)\n", num);

        for (i = 0; i < num; i++) {
            SmbusMsg_s *msg = &msgs[i];

            /* Parameter Check for Buffer */
            SMBUS_CHECK_PARAM_RETURN(msg->len == 0 || msg->buf == NULL, -EINVAL,
                            "SMBus: target msg[%d] invalid len/buf", i);

            if (msg->flags & SMBUS_M_RD) {
                /* * [target READ] = User retrieves data received by ISR
                 * Logic: Copy RX Buffer -> User Buffer -> Clear RX Buffer
                 */
                if (dev->targetValidRxLen > 0) {
                    U32 copyLen = (msg->len < dev->targetValidRxLen) ? msg->len : dev->targetValidRxLen;

                    if (dev->targetRxBuf != NULL) {
                        memcpy(msg->buf, dev->targetRxBuf, copyLen);

                        /* Clear receive buffer status */
                        memset(dev->targetRxBuf, 0, dev->targetValidRxLen);
                        dev->targetValidRxLen = 0;
                        /* Could record actual read length here, but standard I2C Msg structure doesn't usually fill back len */
                    }
                } else {
                    /* No data available */
                    /* In target Read, no data is usually not an error, but returns 0 length */
                    /* To let caller know no data was read, can clear buffer */
                    memset(msg->buf, 0, msg->len);
                }

            } else {
                /* * [target WRITE] = User prepares data for ISR to send
                 * Logic: User Buffer -> TX Buffer -> Set TX Valid Length
                 */
                if (dev->targetTxBuf != NULL) {
                    /* First mark data as invalid to prevent ISR from reading incorrectly during copy */
                    dev->targetValidTxLen = 0;

                    /* Limit copy length to hardware/buffer maximum limit */
                    U32 writeLen = (msg->len > SMBUS_MAX_BLOCK_LEN) ? SMBUS_MAX_BLOCK_LEN : msg->len;

                    memset(dev->targetTxBuf, 0, SMBUS_MAX_BLOCK_LEN);
                    memcpy(dev->targetTxBuf, msg->buf, writeLen);

                    /* Commit data: ISR can now see the valid length */
                    dev->targetValidTxLen = writeLen;
                } else {
                    LOGE("SMBus: target TX buffer is NULL\n");
                    return -EINVAL;
                }
            }

        }

        /* In target mode, directly return number of processed messages to indicate success */
        return num;
    }
    /* 3. Wait for Bus Idle */
    ret = smbusWaitBusNotBusy(dev);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: Bus busy before transfer\n");
        goto exit;
    }
    /* ============================================================
     * 4. Quick Command handling
     * ============================================================*/
    if ((num == 1 && msgs[0].len == 0 && msgs[0].flags & SMBUS_FLAG_QUICK_CMD)) {
        LOGI("SMBus: Quick Command detected during xfer init\n");

        ret = smbusHandleQuickCommand(dev, &msgs[0]);
        if (ret != EXIT_SUCCESS) {
            LOGE("SMBus: Quick Command failed, ret=%d\n", ret);
        } else {
            LOGD("SMBus: Quick Command completed successfully\n");
        }
        goto exit;  ///< Quick Command handled, exit early 
    }
    /* 5. Call Low-Level Driver (where Register Ops happen) */
    /* This function must handle len=0 logic for QuickCmd */
    ret = smbusDwXfer(dev, msgs, num);

    if (ret < EXIT_SUCCESS) {
        LOGE("SMBus: Transfer failed, ret=%d\n", ret);
        /* Update error statistics if available */
        if (dev) {
            dev->cmdErr++;
        }
    } else {
        /* Optional: Update success stats */
        LOGD("SMBus: Transfer complete, processed %d msgs\n", num);
    }

exit:
    return ret;
}

/* Handle SAR (Slave Address Register) related commands */
static S32 smbusHalHandleSar(SmbusDev_s *dev, SmbusCmd_e cmd, SmbusParam_u *p)
{
    volatile SmbusRegMap_s *reg = dev->regBase;
    volatile U32 *pSarReg = NULL;
    SmbusIcEnableReg_u enReg;
    S32 ret = EXIT_SUCCESS;

    if (p->sarConfig.sarId >= SMBUS_MAX_DEVICE_NUM) return -EINVAL;

    /* Get SAR register address */
    switch (p->sarConfig.sarId) {
        case 0: pSarReg = &reg->icSar.value; break;
        case 1: pSarReg = &reg->icSar2.value; break;
        case 2: pSarReg = &reg->icSar3.value; break;
        case 3: pSarReg = &reg->icSar4.value; break;
        default: 
            ret = -ENOTSUP;
            LOGE("SMBus: Unsupported SAR ID %d\n", ret);
            goto exit;
    }

    if (cmd == SMBUS_CMD_SAR_SET_ADDR) {
        /* Validate SMBus SAR address range according to SMBus specification
         * Valid range: 0x08 - 0x77 (reserved 0x00-0x07, 0x78-0x7F) */
        U8 addr = p->sarConfig.slaveAddr & 0x7F; /* Extract 7-bit address */
        if (addr < SMBUS_MIN_VALID_ADDRESS || addr > SMBUS_MAX_VALID_ADDRESS) {
            LOGE("SMBus: Invalid SAR address 0x%02X (valid range: 0x%02X - 0x%02X)\n",
                 addr, SMBUS_MIN_VALID_ADDRESS, SMBUS_MAX_VALID_ADDRESS);
            ret = -EINVAL;
            goto exit;
        }

        smbusDisable(dev); /* Must Disable before modifying address */
        smbusWriteReg(pSarReg, p->sarConfig.slaveAddr & 0x3FF);
        smbusEnable(dev);
        ret = EXIT_SUCCESS;
        goto exit;
    } else if (cmd == SMBUS_CMD_SAR_GET_ADDR) {
        p->sarConfig.slaveAddr = (U8)(smbusReadReg(pSarReg) & 0x3FF);
        ret = EXIT_SUCCESS;
        goto exit;
    }

    /* Enable/Disable SAR */
    enReg.value = smbusReadReg(&reg->icEnable.value);
    bool enable = (cmd == SMBUS_CMD_SAR_ENABLE);
    switch (p->sarConfig.sarId) {
        case 0: enReg.fields.icSarEn = enable; break;
        case 1: enReg.fields.icSar2En = enable; break;
        case 2: enReg.fields.icSar3En = enable; break;
        case 3: enReg.fields.icSar4En = enable; break;
    }
    smbusWriteReg(&reg->icEnable.value, enReg.value);

exit:
    return ret;
}

/* Handle ARP UDID read/write */
static void smbusHalHandleArpUdid(volatile SmbusRegMap_s *reg, SmbusCmd_e cmd, SmbusParam_u *p)
{
    /* Note: SmbusUdid_s structure is assumed to be defined and size matched */
    /* Here param->arp.udid is SmbusUdid_s type, may need cast or msg for byte access */
    /* For generality, here assume udid is also stored or converted as byte stream */
    U8 *d = (U8 *)&p->arp.udid; 
    
    if (cmd == SMBUS_CMD_ARP_SET_UDID) {
        smbusWriteReg(&reg->icSmbusUdidWord0.value, (U32)d[0] | ((U32)d[1]<<8) | ((U32)d[2]<<16) | ((U32)d[3]<<24));
        smbusWriteReg(&reg->icSmbusUdidWord1.value, (U32)d[4] | ((U32)d[5]<<8) | ((U32)d[6]<<16) | ((U32)d[7]<<24));
        smbusWriteReg(&reg->icSmbusUdidWord2.value, (U32)d[8] | ((U32)d[9]<<8) | ((U32)d[10]<<16) | ((U32)d[11]<<24));
        smbusWriteReg(&reg->icSmbusUdidWord3.value, (U32)d[12] | ((U32)d[13]<<8) | ((U32)d[14]<<16) | ((U32)d[15]<<24));
    } else {
        U32 w[4];
        w[0] = smbusReadReg(&reg->icSmbusUdidWord0.value);
        w[1] = smbusReadReg(&reg->icSmbusUdidWord1.value);
        w[2] = smbusReadReg(&reg->icSmbusUdidWord2.value);
        w[3] = smbusReadReg(&reg->icSmbusUdidWord3.value);
        memcpy(d, w, 16);
    }
    LOGE("SMBus: ARP UDID %s: %02X%02X%02X%02X-%02X%02X%02X%02X-%02X%02X%02X%02X-%02X%02X%02X%02X\n",
         (cmd == SMBUS_CMD_ARP_SET_UDID) ? "set" : "get",
         d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7],
         d[8], d[9], d[10], d[11], d[12], d[13], d[14], d[15]);
}

/* Handle ARP related commands */
static S32 smbusHalHandleArp(SmbusDev_s *dev, SmbusCmd_e cmd, SmbusParam_u *p)
{
    volatile SmbusRegMap_s *reg = dev->regBase;

    switch (cmd) {
        case SMBUS_CMD_ARP_ENABLE:
        case SMBUS_CMD_ARP_DISABLE: {
            smbusDisable(dev);
            SmbusIcConReg_u con;
            con.value = smbusReadReg(&reg->icCon.value);
            con.fields.smbusArpEn = (cmd == SMBUS_CMD_ARP_ENABLE);
            smbusWriteReg(&reg->icCon.value, con.value);
            smbusEnable(dev);
            return EXIT_SUCCESS;
        }
        case SMBUS_CMD_ARP_SET_UDID:
        case SMBUS_CMD_ARP_GET_UDID:
            smbusHalHandleArpUdid(reg, cmd, p);
            return EXIT_SUCCESS;
        case SMBUS_CMD_ARP_GET_ADDR_VALID:
            /* Bit 17: SMBUS_SLV_ADDR_VALID */
            p->arp.addrStatus.isValid = (smbusReadReg(&reg->icStatus.value) >> 17) & 1;
            return EXIT_SUCCESS;
        case SMBUS_CMD_ARP_GET_ADDR_RESOLVED:
            /* Bit 18: SMBUS_SLV_ADDR_RESOLVED */
            p->arp.addrStatus.isResolved = (smbusReadReg(&reg->icStatus.value) >> 18) & 1;
            return EXIT_SUCCESS;
        case SMBUS_CMD_ARP_IS_ADDR_USED: {
            SmbusMsg_s msg = { .addr = p->arp.checkAddr, .flags = SMBUS_FLAG_QUICK_CMD, .len = 0 };
            S32 ret = smbusDwXfer(dev, &msg, 1);
            p->arp.addrStatus.isInUse = (ret >= 0); /* Success(ACK)=Used, ENXIO(NACK)=Unused */
            return (ret == -ENXIO) ? EXIT_SUCCESS : ret;
        }
        default: return -EINVAL;
    }
}

/* Handle Host Notify and Alert */
static S32 smbusHalHandleNotifyAlert(SmbusDev_s *dev, SmbusCmd_e cmd, SmbusParam_u *p)
{
    volatile SmbusRegMap_s *reg = dev->regBase;
    S32 ret = EXIT_SUCCESS;
    
    if (cmd == SMBUS_CMD_HOST_NOTIFY_ENABLE || cmd == SMBUS_CMD_HOST_NOTIFY_DISABLE) {

        IcSmbusIntrMask_u mask;
        mask.value = smbusReadReg(&reg->icSmbusIntrMask.value);
        mask.fields.hostNotifyMstDet = (cmd == SMBUS_CMD_HOST_NOTIFY_ENABLE);
        smbusWriteReg(&reg->icSmbusIntrMask.value, mask.value);
        return EXIT_SUCCESS;

    } else if (cmd == SMBUS_CMD_HOST_NOTIFY) {
        Bool wasTarget = (dev->mode == SMBUS_MODE_TARGET);
        /* Use dev->notifyBuf to avoid dependency on dev->masterTxBuf
         * which may be freed when switching to target mode */
        dev->notifyBuf[0] = (p->hostNotify.slaveAddr << 1);
        dev->notifyBuf[1] = (U8)p->hostNotify.data;
        dev->notifyBuf[2] = (U8)(p->hostNotify.data >> 8);

        SmbusMsg_s msg = {
            .addr = SMBUS_HOST_NOTIFY_ADDR,
            .len = 3,
            .buf = dev->notifyBuf,
        };
        LOGE("SMBus: Sending Host Notify to 0x%02X with data 0x%04X\n",
             SMBUS_HOST_NOTIFY_ADDR, p->hostNotify.data);
        
        /* 如果当前是 Target 模式，必须临时将软件状态改为 MASTER。
         * 否则 ISR 会进入 Target 分支，把 Host Notify 的 TX_EMPTY 中断
         * 当作误报 (Spurious) 处理并屏蔽，导致死锁/超时。
         */
        if (wasTarget) {
            dev->mode = SMBUS_MODE_MASTER; 
            LOGD("SMBus: Host Notify - Temp switch SW state to Master\n");
        }
        ret = smbusDwXfer(dev, &msg, 1);
        if (wasTarget) {
            LOGD("SMBus: Notify sent, switching back to Target mode...\n");
            smbusDisable(dev); 
        
            SmbusIcConReg_u icCon;
            icCon.value = smbusReadReg(&reg->icCon.value);
            icCon.fields.masterMode = 0;      
            icCon.fields.ictargetDisable = 0;     
            icCon.fields.icRestartEn = 1;     
            smbusWriteReg(&reg->icCon.value, icCon.value);
            
            dev->mode = SMBUS_MODE_TARGET;
            smbusEnable(dev); ///< Re-enable controller of target mode
            dev->mode = SMBUS_MODE_TARGET;
            LOGD("SMBus: Switched back to Target mode after Notify\n");
        }
    
        return ret >= 0 ? EXIT_SUCCESS : -EIO;

    } else if (cmd == SMBUS_CMD_ALERT_ENABLE || cmd == SMBUS_CMD_ALERT_DISABLE) {

        SmbusIcEnableReg_u en;
        en.value = smbusReadReg(&reg->icEnable.value);
        en.fields.smbusAlertEn = (cmd == SMBUS_CMD_ALERT_ENABLE);
        smbusWriteReg(&reg->icEnable.value, en.value);
        return EXIT_SUCCESS;

    } else if (cmd == SMBUS_CMD_ALERT_RESPOND) {  ///< Get Alert Response Master mode
        /* Use dev->alertBuf to avoid dependency on dev->masterTxBuf
         * which may be freed when switching to target mode */

        SmbusMsg_s msg = {
            .addr = SMBUS_ALERT_RESPONSE_ADDR,
            .flags = SMBUS_M_RD,
            .len = 1,
            .buf = dev->alertBuf,
        };
        if (smbusDwXfer(dev, &msg, 1) >= 0) {
            p->alertResponse.respondingAddr = dev->alertBuf[0] >> 1;
            p->alertResponse.status = dev->alertBuf[0] & 1;
            return EXIT_SUCCESS;
        }
        return -EIO;
    }
    return -EINVAL;
}

/* Handle bus recovery */
static S32 smbusHalBusRecovery(SmbusDev_s *dev, SmbusParam_u *param)
{
    volatile SmbusRegMap_s *reg = dev->regBase;
    SmbusIcEnableReg_u en;
    U32 timeout;
    U32 activityStatus;

    /* If forceRecovery is false, check if bus is actually stuck */
    if (!param->busRecovery.forceRecovery) {
        /* Read bus activity status */
        activityStatus = smbusReadReg(&reg->icStatus.value);

        /* If bus is not busy (no activity), skip recovery to avoid NACK errors */
        if (!(activityStatus & SMBUS_IC_STATUS_ACTIVITY_MASK)) {
            LOGD("SMBus: Bus healthy (no activity), skipping recovery (forceRecovery=false)\n");
            return EXIT_SUCCESS;
        }

        LOGW("SMBus: Bus busy detected (Stat=0x%08X), proceeding with recovery\n", activityStatus);

        /* Give bus some time to complete ongoing transfer */
        timeout = 200;  /* 2ms */
        while (timeout--) {
            activityStatus = smbusReadReg(&reg->icStatus.value);
            if (!(activityStatus & SMBUS_IC_STATUS_ACTIVITY_MASK)) {
                LOGD("SMBus: Bus cleared during wait, skipping recovery\n");
                return EXIT_SUCCESS;
            }
            udelay(10);
        }
    }

    /* Force recovery or bus is stuck - proceed with SDA stuck recovery */
    LOGI("SMBus: Starting bus recovery (sclCount=%u, timeout=%u)\n",
         param->busRecovery.sclRecoveryCount, param->busRecovery.timeoutMs);

    smbusEnable(dev); /* Must enable first */
    en.value = smbusReadReg(&reg->icEnable.value);
    en.fields.sdaStuckRecoveryEnable = 1;
    smbusWriteReg(&reg->icEnable.value, en.value);

    /* Use timeout from parameter, with default fallback */
    timeout = (param->busRecovery.timeoutMs > 0) ?
              (param->busRecovery.timeoutMs * 100) : 100000;  /* Convert ms to 10us units */

    while (timeout--) {
        en.value = smbusReadReg(&reg->icEnable.value);
        if (!en.fields.sdaStuckRecoveryEnable) {
            LOGI("SMBus: Bus recovery completed successfully\n");
            return EXIT_SUCCESS;
        }
        udelay(10);
    }
    LOGE("SMBus: Bus recovery timed out (recovery bit still set)\n");
    return -ETIMEDOUT;
}

/**
 * @brief HAL Control unified dispatch entry
 * @note Length meets requirements, logic is simple, only dispatches
 */
static S32 smbusHalControl(SmbusDev_s *dev, SmbusCmd_e cmd, void *arg)
{
    SmbusParam_u *p = (SmbusParam_u *)arg;

    /* Handle basic commands directly */
    switch (cmd) {
        case SMBUS_CMD_HW_ENABLE:
            smbusEnable(dev);
            return EXIT_SUCCESS;
        case SMBUS_CMD_HW_DISABLE:
            smbusDisable(dev);
            return EXIT_SUCCESS;
        case SMBUS_CMD_BUS_RECOVERY:
            return smbusHalBusRecovery(dev, p);
        default: break; /* Continue to groups */
    }

    /* Group processing */
    if (cmd >= SMBUS_CMD_SAR_ENABLE && cmd <= SMBUS_CMD_SAR_GET_ADDR)
        return smbusHalHandleSar(dev, cmd, p);

    if (cmd >= SMBUS_CMD_ARP_ENABLE && cmd <= SMBUS_CMD_ARP_GET_ADDR_RESOLVED)
        return smbusHalHandleArp(dev, cmd, p);

    if ((cmd >= SMBUS_CMD_HOST_NOTIFY && cmd <= SMBUS_CMD_HOST_NOTIFY_DISABLE) ||
        (cmd >= SMBUS_CMD_ALERT_RESPOND && cmd <= SMBUS_CMD_ALERT_DISABLE))
        return smbusHalHandleNotifyAlert(dev, cmd, p);

    return -EINVAL;
}

/* ======================================================================== */
/*                    HAL Layer - Export Functions                            */
/* ======================================================================== */
/**
 * @brief Get HAL function pointers for external use
 */
SmbusHalOps_s* smbusGetHalOps(void)
{
    static SmbusHalOps_s halOps = {
        .checkTxReady = smbusCheckTxready,
        .checkRxReady = smbusCheckRxready,
        .enable = smbusEnable,
        .disable = smbusDisable,
        .modeSwitchCore = smbusModeSwitchCore,
        .i2cTransfer = smbusI2cTransfer,
        .smbusDwRead = smbusDwRead,
        .smbusDwXferMsg = smbusDwXferMsg,
        .configureTarget = smbusConfigureTarget,
        .control = smbusHalControl,
    };
    return &halOps;
}
