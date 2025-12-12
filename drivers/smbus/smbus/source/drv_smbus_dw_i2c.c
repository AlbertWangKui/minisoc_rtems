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
 *
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
static S32 smbusHandleQuickCommand(SmbusDev_s *dev, SmbusMsg_s *msg);
/**
 * @brief Check if TX FIFO is ready (not full)
 */
static inline S32 smbusCheckTxready(volatile SmbusRegMap_s *regBase)
{
    SmbusIcStatusReg_u status;
    U32 timeout = SMBUS_TX_READY_TIMEOUT_US;
    while (timeout--) {
        status.value = regBase->icStatus.value;
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
        status.value = regBase->icStatus.value;
        if (status.fields.rfne) {  /* RX FIFO not empty */
            return 0;
        }
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(1));
    }
    return -ETIMEDOUT;
}

/**
 * @brief Set SMBus slave address
 * @details Programs the slave address register following the SMBus Address Assignment protocol
 *          This function follows the official SMBus specification for setting slave addresses:
 *          1. Disable IC_ENABLE
 *          2. Set IC_SAR
 *          3. Re-enable IC_ENABLE
 *          4. Wait for address resolution
 * @param[in] dev Pointer to SMBus device structure
 * @return S32 0 on success, -EINVAL on failure
 *
 * @note Follows SMBus Address Assignment protocol per official specification
 * @note IC_SAR[9:0] = slave address, IC_SAR_ENABLE[19] = enable in IC_ENABLE register
 * @note After enabling, system waits for address to be resolved by the master
 */
static S32 smbusSetSlaveAddr(SmbusDev_s *dev)
{
    SmbusIcSarReg_u sar;
    SmbusIcEnableReg_u enable;
    S32 ret = 0;

    SMBUS_CHECK_PARAM(dev == NULL || dev->regBase == NULL, -EINVAL,
                      "SMBus: Invalid device or register base\n");

    LOGD("SMBus: Starting address assignment protocol for address 0x%02X\n", dev->slaveAddr);

    /* Step 1: Disable IC_ENABLE (per SMBus Address Assignment protocol) */
    enable.value = dev->regBase->icEnable.value;
    enable.fields.enable = 0;  /* Disable I2C/SMBus controller */
    dev->regBase->icEnable.value = enable.value;
    rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(1));  /* 1ms delay using RTEMS */
    /* Step 2: Set slave address in IC_SAR register */
    sar.value = dev->regBase->icSar.value;
    sar.fields.icSar = dev->slaveAddr;  /* Slave address (7-bit goes to bits [6:0], 10-bit to bits [9:0]) */
    dev->regBase->icSar.value = sar.value;

    LOGD("SMBus: Address set to 0x%02X in IC_SAR register\n", dev->slaveAddr);

    /* Step 3: Re-enable IC_ENABLE with SAR_ENABLE bit set */
    enable.value = dev->regBase->icEnable.value;
    enable.fields.enable = 1;      /* Enable I2C/SMBus controller */
    enable.fields.icSarEn = 1;      /* Enable slave address register */
    dev->regBase->icEnable.value = enable.value;

    LOGD("SMBus: Controller re-enabled with SAR_EN=1\n");

    /* Step 4: Wait for address resolution to begin */
    rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(10));  /* 10ms delay using RTEMS */

    /* Step 5: Read back to verify both registers */
    SmbusIcSarReg_u sarVerify;
    SmbusIcEnableReg_u enableVerify;
    sarVerify.value = dev->regBase->icSar.value;
    enableVerify.value = dev->regBase->icEnable.value;

    LOGD("SMBus: Address assignment completed - address: 0x%02X, SAR: 0x%03X, ENABLE: %u, SAR_EN: %u\n",
         dev->slaveAddr, sarVerify.fields.icSar, enableVerify.fields.enable, enableVerify.fields.icSarEn);

exit:
    return ret;
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
    enable.value = dev->regBase->icEnable.value;
    enable.fields.enable = 0;
    dev->regBase->icEnable.value = enable.value;

    /* Wait for controller to be disabled */
    while (timeout--) {
        status.value = dev->regBase->icEnableStatus;
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

    enable.value = dev->regBase->icEnable.value;
    enable.fields.enable = 1;
    dev->regBase->icEnable.value = enable.value;
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
 * @note Handles both master and slave mode configurations
 * @note Configures SMBus-specific interrupts when SMBus mode is enabled
 * @note Clears pending interrupts before configuration
 */
static S32 smbusConfigTransferInterrupts(SmbusDev_s *dev, volatile SmbusRegMap_s *regBase)
{
    if (dev == NULL || regBase == NULL) {
        return -EINVAL;
    }

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
        if (dev->mode == SMBUS_MODE_SLAVE) {
            /*
             * Slave mode: Configure optimal interrupts for slave operations
             * - Write transfer: WR_REQ, RX_FULL for receiving data
             * - Read transfer: RD_REQ, TX_EMPTY for sending data
             * - Common: TX_ABRT, STOP_DET, RESTART_DET for error handling
             */
            regBase->icIntrMask.value = SMBUS_SLAVE_OPTIMAL_CONFIG;
            dev->status = SMBUS_STATUS_IDLE;
            LOGD("SMBus: Slave mode interrupts configured (mask: 0x%08X)\n", SMBUS_SLAVE_OPTIMAL_CONFIG);
        } else {
            /*
             * Master mode: Configure essential interrupts for master operations
             * - TX_EMPTY: For sending data to slave
             * - RX_FULL: For receiving data from slave
             * - TX_ABRT: Handle transfer aborts
             * - STOP_DET/START_DET: Detect bus conditions
             * - ACTIVITY: Monitor bus activity status
             */
            regBase->icIntrMask.value = SMBUS_MASTER_INTERRUPT_CONFIG;
            LOGD("SMBus: Master mode interrupts enabled (0x%08X)\n", SMBUS_MASTER_INTERRUPT_CONFIG);
        }
    }
    
    /* Enable critical error-related interrupts for both modes */
    regBase->icIntrMask.value |= SMBUS_ERROR_INTERRUPT_CONFIG;
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

    /* Addressing Mode */
    if (dev->addrMode == 1) {
        icCon.fields.ic10bitaddrMaster = 1;
        LOGD("SMBus: Using 10-bit addressing\n");
    } else {
        icCon.fields.ic10bitaddrMaster = 0;
        LOGD("SMBus: Using 7-bit addressing\n");
    }

    /* Restart enable (mandatory for SMBus block read and most reads) */
    icCon.fields.icRestartEn = dev->restartEnb ? 1 : 0;

    regBase->icCon.value = icCon.value;

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
    SmbusIcRawIntrStatReg_u rawIntr = regBase->icRawIntrStat;
    if (rawIntr.fields.txAbrt) {
        /* Read and log abort source */
        abrtSource.value = regBase->icTxAbrtSource.value;
        LOGE("SMBus: Transfer abort detected, source=0x%08X\n", abrtSource.value);

        /* Clear abort condition */
        (void)regBase->icClrTxAbrt;

        /* Return specific error based on abort source */
        if (abrtSource.value & SMBUS_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_MASK) {
            return -ENXIO;  /* No acknowledge from slave */
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

    ///< Start the async transfer
    ret = smbusDwXferInit(dev, msgs, 0);
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
        ret = -ETIMEDOUT; /* 返回超时错误 */
        goto exit;
    }

    LOGD("SMBus: Transfer completed - semaphore released by ISR\n");

    U16 activeRetry = 50000;
    while (smbusIsControllerActive(dev) && activeRetry > 0) {
        udelay(1);
        activeRetry--;
    }

    /* 只有在重试后仍然 Active，且不是正常的传输完成状态时才报错 */
    if (smbusIsControllerActive(dev)) {
        ///< 降低日志级别，或者在 ret == SUCCESS 时忽略此错误
        if (dev->errorType == 0) {
            LOGW("SMBus: Controller active bit sticky, but transfer seems success. Ignoring.\n");
        } else {
            LOGE("SMBus: Controller still active after transfer completion\n");
            dev->errorType = SMBUS_ERR_TYPE_UNKNOWN; // 或者具体的超时错误
        }
    }

    if (dev->cmdErr || dev->errorType > 0 || dev->msgErr) {
        if (dev->msgErr) {
            /* 确保 msgErr 也是负值 */
            ret = (dev->msgErr < 0) ? dev->msgErr : -EIO;
            LOGE("SMBus: Message error: %d\n", ret);
            goto exit;
        }
        ret = smbusHandleTxAbort(dev);
        ///< Log enhanced error information with device context
        LOGE("SMBus: Transfer failed - Device context:\n");
        LOGE("  - Slave address: 0x%02X\n", dev->slaveAddr);
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
    memcpy(&dev->msgs, msgs, sizeof(SmbusMsg_s));
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
        buf = msgs[msgWrtIdx].buf;
        bufLen = msgs[msgWrtIdx].len;

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
                    cmd |= (1 << 9);
                }

                /* Send read command - format: READ_CMD for read + stop bit if needed */
                dataCmd = SMBUS_IC_DATA_CMD_READ_CMD | cmd;
                dev->regBase->icDataCmd.value = dataCmd;

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
                val = regBase->icDataCmd.value;

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
                    cmd |= (1 << 9);
                }

                /* Write data */
                dataCmd = (*buf++) | cmd;
                regBase->icDataCmd.value = dataCmd;

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

        /* Check for stop bit detection */
        status = smbusDwCheckStopBit(dev);
        if (status != EXIT_SUCCESS) {
            LOGE("SMBus: Stop bit check failed: %d\n", status);
            return status;
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

    /* 1. 确保控制器已启用 (ENABLE = 1) 
     * 硬件需要时钟才能发送恢复脉冲，所以必须先 Enable。
     */
    if (!(regBase->icEnable.value & SMBUS_IC_ENABLE_ENABLE_MASK)) {
        regBase->icEnable.value |= SMBUS_IC_ENABLE_ENABLE_MASK;
        /* 简单等待启用生效 */
        rtems_task_wake_after(RTEMS_MICROSECONDS_TO_TICKS(1)); 
    }

    /* 2. 设置 SDA_STUCK_RECOVERY_ENABLE (Bit 3) */
    /* 使用读-改-写以保护其他位 */
    currentVal = regBase->icEnable.value;
    regBase->icEnable.value = currentVal | SMBUS_IC_ENABLE_RECOVERY_MASK;

    /* 3. 等待硬件自动清除该位 (表示恢复动作完成) */
    /* 通常需要几个 I2C 周期，给予 1ms 超时绰绰有余 */
    timeout = 100; 
    while (timeout--) {
        if (!(regBase->icEnable.value & SMBUS_IC_ENABLE_RECOVERY_MASK)) {
            /* Bit 3 已自动清零，恢复动作完成 */
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

    if (!(regBase->icStatus.value & SMBUS_IC_STATUS_ACTIVITY_MASK)) {
        return EXIT_SUCCESS;
    }

    /* 给上一笔传输一点收尾时间 (2ms) */
    timeout = 200;
    while (timeout--) {
        if (!(regBase->icStatus.value & SMBUS_IC_STATUS_ACTIVITY_MASK)) {
            return EXIT_SUCCESS;
        }
        rtems_task_wake_after(RTEMS_MICROSECONDS_TO_TICKS(10));
    }

    LOGW("SMBus: Bus stuck BUSY (Stat=0x%08X), starting recovery procedure...\n", regBase->icStatus.value);

    /* ---------------------------------------------------------
     * 2. 执行硬件总线恢复 (SDA Stuck Recovery)
     * --------------------------------------------------------- */
    ret = smbusPerformHardwareRecovery(regBase);
    if (ret != EXIT_SUCCESS) {
        /* 如果硬件恢复都超时了，说明 IP 可能挂死了，只能尝试强行复位 */
        LOGE("SMBus: Hardware recovery failed, proceeding to force reset.\n");
    }

    /* ---------------------------------------------------------
     * 3. 状态机复位流程 (Abort -> Disable -> Enable)
     * 即使电气上总线已经恢复，IP 内部状态机可能还认为自己在传输中
     * --------------------------------------------------------- */
    /* 3.1 尝试发送 ABORT 信号 (终止任何挂起的传输) */
    /* 只有在 Enable=1 时 Abort 才有效 */
    if (regBase->icEnable.value & SMBUS_IC_ENABLE_ENABLE_MASK) {
        regBase->icEnable.value |= SMBUS_IC_ENABLE_ABORT_MASK;
        
        /* 等待 ABORT 完成 (TX_ABRT 中断位置位) */
        timeout = 1000;
        while (timeout--) {
            if (regBase->icRawIntrStat.value & SMBUS_INTR_TX_ABRT_MASK) {
                (void)regBase->icClrTxAbrt; /* 清除中断 */
                break;
            }
            rtems_task_wake_after(RTEMS_MICROSECONDS_TO_TICKS(10));
        }
    }

    /* 3.2 强制禁用控制器 (Disable) - 彻底复位状态机 */
    regBase->icEnable.value = 0;
    /* 等待 Enable bit 真正清零 */
    timeout = 2000; /* 20ms */
    while (timeout--) {
        if (!(regBase->icEnableStatus & SMBUS_IC_ENABLE_ENABLE_MASK)) {
            break; 
        }
        rtems_task_wake_after(RTEMS_MICROSECONDS_TO_TICKS(10));
    }

    /* 3.3 重新启用控制器 (Enable) */
    regBase->icEnable.value = SMBUS_IC_ENABLE_ENABLE_MASK;

    /* ---------------------------------------------------------
     * 4. 最终状态检查
     * --------------------------------------------------------- */
    /* 此时总线应该是空闲的 */
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
    U32 status = regBase->icStatus.value;

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
        LOGE("SMBus: 7-bit address NACK (0x%02X) - Slave device not responding or not present\n",
             dev->slaveAddr);
        LOGE("SMBus: Abort source=0x%08X, errorType=NACK_7BIT\n", abortSource);
        return -ENXIO;  ///< No such device or address
    } else if (abortSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_10B_ADDR_NOACK_MASK) {
        dev->errorType = SMBUS_ERR_TYPE_NACK_10BIT;
        LOGE("SMBus: 10-bit address NACK (0x%03X) - Slave device not responding\n",
             dev->slaveAddr);
        LOGE("SMBus: Abort source=0x%08X, errorType=NACK_10BIT\n", abortSource);
        return -ENXIO;  ///< No such device or address
    } else if (abortSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_MASK) {
        dev->errorType = SMBUS_ERR_TYPE_NACK_DATA;
        LOGE("SMBus: Data NACK - Slave rejected data during transfer\n");
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

    /* 2. 配置 IC_TAR (Master 模式目标地址) */
    icTar.fields.icTar = msg->addr & 0x7F;
    icTar.fields.special = 1;       ///< 启用特殊命令
    icTar.fields.smbusQuickCmd = 1; ///< 标记为 Quick Command
    regBase->icTar.value = icTar.value;

    smbusEnable(dev);

    /* 3. 准备同步机制 */
    dev->xferStatus = -ETIMEDOUT; ///< 默认状态为超时
    dev->isQuick = 1;             ///< 告诉 ISR：这是 Quick Command，请处理 STOP 信号
    
    /* 清除可能残留的中断状态 */
    (void)regBase->icClrStopDet;
    (void)regBase->icClrTxAbrt;

    /* 4. 开启中断 (关键！) */
    /* 必须开启 STOP_DET 和 TX_ABRT，否则 ISR 永远不会触发 */
    U32 intrMask = regBase->icIntrMask.value;
    regBase->icIntrMask.value = intrMask | SMBUS_INTR_STOP_DET | SMBUS_INTR_TX_ABRT;
    U32 intrSmbusMask = regBase->icSmbusIntrMask.value;
    regBase->icSmbusIntrMask.value = intrSmbusMask | SMBUS_QUICK_CMD_DET_BIT;

    /* 5. 写入命令字，触发硬件传输 */
    icDataCmd.fields.cmd = (msg->flags & SMBUS_M_RD) ? 1 : 0; ///< 读/写
    icDataCmd.fields.stop = 1;                                ///< 必须产生 STOP
    icDataCmd.fields.restart = 0;
    regBase->icDataCmd.value = icDataCmd.value;

    /* 6. 睡眠等待 ISR 唤醒*/
    if (rtems_semaphore_obtain(dev->semaphoreId, RTEMS_WAIT, dev->transferTimeout) == RTEMS_SUCCESSFUL) {
         /* 被 ISR 唤醒，返回 ISR 设置的结果 */
         ret = dev->xferStatus;
    } else {
        /* 超时：硬件没有响应 */
        LOGE("SMBus: Quick Command Timed Out!\n");
        ret = -ETIMEDOUT;
        /* 此时需要做清理工作，防止迟到的中断破坏状态 */
        dev->isQuick = 0;
    }

    /* 7. 恢复中断掩码 (可选) */
    regBase->icIntrMask.value = intrMask;
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
    if (pDrvData->rxBuffer == NULL) {
        LOGE("SMBus: CRITICAL - rxBuffer is NULL in smbusMasterReadData!\n");
        LOGE("SMBus: pDrvData=%p, rxBuffer=%p, rxLength=%d, interruptMode=%d\n",
             pDrvData, pDrvData->rxBuffer, pDrvData->rxLength, pDrvData->sbrCfg.interruptMode);
        return;
    }

    rxValid = regBase->icRxflr;

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
        U32 currentMask = regBase->icIntrMask.value;
        currentMask &= ~SMBUS_INTR_TX_EMPTY;
        regBase->icIntrMask.value = currentMask;
        pDrvData->txComplete = 1;
        return;
    }

    txAvailable = regBase->icTxflr;
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

            regBase->icDataCmd.value = cmd;
            LOGD("SMBus: Sent byte 0x%02X, remaining: %d\n", cmd & 0xFF, buf_len);
        }

        /* Mark current message as complete */
        dev->msgWriteIdx++;

        if (dev->msgWriteIdx >= dev->msgsNum) {
            /* All messages processed */
            pDrvData->txComplete = 1;
            LOGD("SMBus: All messages transferred, disabling TX_EMPTY\n");

            /* Disable TX_EMPTY interrupt to prevent continuous triggering */
            U32 currentMask = regBase->icIntrMask.value;
            currentMask &= ~SMBUS_INTR_TX_EMPTY;
            regBase->icIntrMask.value = currentMask;
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
    regBase->icClrSmbusIntr = smbusIntrStat;
}

/* ======================================================================== */
/*                    HAL Layer - I2C Compatible Probe Functions               */
/* ======================================================================== */

/**
 * @brief Calculate SMBus controller FIFO size
 * @details Detects and configures the TX and RX FIFO sizes by reading
 *          the component parameters. This function is compatible with
 *          I2C initialization and configures FIFO parameters for both
 *          master and slave modes.
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
    param = dev->regBase->icCompParam1;

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
 * @note Sets master mode, disables slave mode, enables restart
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
    masterCfg |= (1U << SMBUS_IC_CON_SLAVE_DISABLE_BIT);   /* Slave disable */
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
 * @brief Configure SMBus slave mode
 * @details Configures the SMBus controller for slave mode operation.
 *          This function sets up the slave configuration register with
 *          appropriate control bits and addressing mode.
 * @param[in] dev Pointer to SMBus device structure
 * @return void
 *
 * @note Compatible with I2C's i2c_dw_configure_slave interface
 * @note Enables slave mode, configures FIFO hold control
 * @note Sets addressing mode and restart support
 * @note Configures stop detection when addressed
 */
static void smbusConfigureSlave(SmbusDev_s *dev)
{
    U32 slaveCfg = 0;

    SMBUS_CHECK_PARAM_VOID(dev == NULL || dev->regBase == NULL,
                          "SMBus: Invalid device for slave configuration");

    /* Configure slave mode - compatible with smbusProbeSlave settings */
    slaveCfg &= ~(1U << SMBUS_IC_CON_MASTER_MODE_EN_BIT);  /* Master mode disable */
    slaveCfg &= ~(1U << SMBUS_IC_CON_SLAVE_DISABLE_BIT);   /* Slave enable */
    slaveCfg |= (1U << SMBUS_IC_CON_RESTART_EN_BIT);      /* Restart enable for combined transactions */

    /* Set speed based on clock rate using proper macro definitions */
    if (dev->clkRate >= SMBUS_SPEED_HIGH_THRESHOLD) {
        slaveCfg |= SMBUS_SPEED_HIGH_CFG;      /* High speed (3.4 MHz) */
    } else if (dev->clkRate >= SMBUS_SPEED_FAST_PLUS_THRESHOLD) {
        slaveCfg |= SMBUS_SPEED_FAST_PLUS_CFG; /* Fast Plus speed (1 MHz) */
    } else if (dev->clkRate >= SMBUS_SPEED_FAST_THRESHOLD) {
        slaveCfg |= SMBUS_SPEED_FAST_CFG;      /* Fast speed (400 kHz) */
    } else {
        slaveCfg |= SMBUS_SPEED_STANDARD_CFG;  /* Standard speed (100 kHz) */
    }

    /* Set address mode */
    if (dev->addrMode == 1) {
        slaveCfg |= (1U << SMBUS_IC_CON_10BIT_SLAVE_ADDR_BIT);  /* 10-bit slave addressing */
    }

    /* Configure SMBus features for slave mode */
    slaveCfg |= (1U << SMBUS_IC_CON_RX_FIFO_HOLD_BIT);         /* RX FIFO full hold control */
    slaveCfg |= (1U << SMBUS_IC_CON_STOP_DET_IFADDRESSED_BIT); /* STOP detection when addressed */

    slaveCfg |= (dev->smbFeatures.arpEnb << SMBUS_IC_CON_ARP_ENABLE_BIT);     /* 位18 = 1 (ARP enabled) */
    slaveCfg |= (dev->smbFeatures.quickCmdEnb << SMBUS_IC_CON_QUICK_CMD_BIT);  /* 位17 = 1 (Quick command enable) */
    /* Store configuration in device structure */
    dev->slaveCfg = slaveCfg;
    dev->mode = SMBUS_MODE_SLAVE;

    LOGD("SMBus: Slave configuration: 0x%08X, address: 0x%02X\n",
         slaveCfg, dev->slaveAddr);
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
    ic_con = regBase->icCon.value;
    if (ic_con & (1U << 11)) {  /* DW_IC_CON_BUS_CLEAR_CTRL */
        dev->masterCfg |= (1U << 11);  /* DW_IC_CON_BUS_CLEAR_CTRL */
    }

    /* Step 3: Apply master configuration to hardware */
    regBase->icCon.value = dev->masterCfg;

    /* Step 4: Configure SMBus-specific features for master mode */
    ic_con = regBase->icCon.value & ~0x60000U;  /* Clear SMBus feature bits */

    ic_con |= (dev->smbFeatures.arpEnb         << 18) |   /* ARP */
              (dev->smbFeatures.quickCmdEnb    << 17);    /* Quick Command */            
  
    regBase->icCon.value = ic_con;

    /* Step 5: Configure timing parameters in hardware registers */
    /* Write standard speed timing parameters */
    if (dev->ssHcnt && dev->ssLcnt) {
        regBase->icSsSclHcnt = dev->ssHcnt;
        regBase->icSsSclLcnt = dev->ssLcnt;
        LOGD("SMBus: Standard timing set - HCNT:%u, LCNT:%u\n", dev->ssHcnt, dev->ssLcnt);
    }

    /* Write fast mode/fast mode plus timing parameters */
    if (dev->fsHcnt && dev->fsLcnt) {
        regBase->icFsSclHcnt = dev->fsHcnt;
        regBase->icFsSclLcnt = dev->fsLcnt;
        LOGD("SMBus: Fast timing set - HCNT:%u, LCNT:%u\n", dev->fsHcnt, dev->fsLcnt);
    }

    /* Write high speed timing parameters */
    if (dev->hsHcnt && dev->hsLcnt) {
        regBase->icHsSclHcnt = dev->hsHcnt;
        regBase->icHsSclLcnt = dev->hsLcnt;
        LOGD("SMBus: High speed timing set - HCNT:%u, LCNT:%u\n", dev->hsHcnt, dev->hsLcnt);
    }

    /* Write SDA hold time if supported */
    if (dev->sdaHoldTime) {
        regBase->icSdaHold = dev->sdaHoldTime;
        LOGD("SMBus: SDA hold time set: %u\n", dev->sdaHoldTime);
    }

    /* Step 6: Set TX/RX FIFO thresholds for interrupt generation */
    regBase->icTxTl = 0;  /* TX threshold: generate interrupt when TX FIFO is empty */
    regBase->icRxTl = 0;  /* RX threshold: generate interrupt when RX FIFO has 1+ bytes */

    /* Step 7: Clear interrupt mask initially */
    regBase->icIntrMask.value = 0;

    /* Step 8: Enable controller */
    smbusEnable(dev);

    LOGD("SMBus: Master probe completed successfully\n");
    return EXIT_SUCCESS;
}

/**
 * @brief SMBus slave probe function compatible with I2C initialization
 * @details Configures SMBus controller in slave mode, calculates timing,
 *          detects FIFO size, and enables interrupts. This function is
 *          compatible with I2C initialization interface and can be called
 *          from SMBus device initialization to enable I2C functionality.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Compatible with I2C's i2c_dw_probe_slave interface
 * @note Configures slave mode timing and FIFO parameters
 * @note Sets up interrupt handling for slave operations
 * @note Allocates slave buffer memory
 * @note Enables controller after configuration
 * @warning This function should only be called in Slave mode
 */
S32 smbusProbeSlave(SmbusDev_s *dev)
{
    S32 ret;

    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL, -EINVAL,
                            "SMBus: Invalid device structure");

    /* Calculate FIFO size for slave operations */
    ret = smbusCalcFifoSize(dev);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: FIFO size calculation failed: %d\n", ret);
        return ret;
    }
    
    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)dev->regBase;

    /* Slave 模式初始化 */
    if (dev->mode == SMBUS_MODE_SLAVE) {
        LOGD("=== Starting Slave Mode Initialization ===\n");

        /* 验证 slaveCfg 是否已正确配置 */
        if (dev->slaveCfg == 0) {
            LOGW("SMBus: slaveCfg not configured, using default slave settings\n");
            /* 使用与 smbusConfigureSlave 一致的默认 Slave 模式配置 */
            dev->slaveCfg = (0U << SMBUS_IC_CON_MASTER_MODE_EN_BIT) |      /* 位0 = 0 (Slave mode) */
                           (2U << 1) |                                      /* 位1:2 = 10 (Fast mode 400kHz) */
                           (0U << SMBUS_IC_CON_SLAVE_DISABLE_BIT) |        /* 位6 = 0 (Slave enabled) */
                           (1U << SMBUS_IC_CON_RESTART_EN_BIT) |           /* 位5 = 1 (Restart enable) */
                           (1U << SMBUS_IC_CON_RX_FIFO_HOLD_BIT) |         /* 位9 = 1 (RX FIFO hold control) */
                           (1U << SMBUS_IC_CON_STOP_DET_IFADDRESSED_BIT) | /* 位7 = 1 (STOP detection when addressed) */
                           (dev->smbFeatures.arpEnb << SMBUS_IC_CON_ARP_ENABLE_BIT) |     /* 位18 = 1 (ARP enabled) */
                           (dev->smbFeatures.quickCmdEnb << SMBUS_IC_CON_QUICK_CMD_BIT);  /* 位17 = 1 (Quick command enable) */
            LOGD("Applied default slave configuration: slaveCfg = 0x%08X\n", dev->slaveCfg);
        } else {
            LOGD("Using pre-configured slaveCfg = 0x%08X\n", dev->slaveCfg);
        }

        /* 1. 必须首先禁用设备，才能修改 CON 和 SAR */
        regBase->icEnable.value = 0;
        LOGD("Device disabled (IC_ENABLE = 0x%08X)\n", regBase->icEnable.value);

        /* 轮询等待设备确实被禁用 (这是 Synopsys 推荐的做法，防止此时还有活动传输) */
        int timeout = 1000;
        while ((regBase->icEnableStatus & 0x1) && --timeout > 0);
        if (timeout == 0) LOGW("Warning: Timeout waiting for IC disable\n");

        /* 2. 在禁用状态下配置所有寄存器 */
        regBase->icCon.value = dev->slaveCfg;
        LOGD("IC_CON configured using pre-configured slaveCfg = 0x%08X\n", regBase->icCon.value);

        /* 设置从机地址 - 优先使用动态参数，未设置时使用默认值 */
        if (dev->slaveAddr != 0) {
            regBase->icSar.fields.icSar = dev->slaveAddr; 
            if (dev->smbFeatures.arpEnb) {
                regBase->icSar.fields.icSar = SMBUS_SLAVE_DEFAULT_ADDR; 
            }
            LOGD("IC_SAR set to default address: 0x%02X\n", regBase->icSar.fields.icSar);
        } else {
            regBase->icSar.fields.icSar = SMBUS_SLAVE_DEFAULT_ADDR; /* 使用默认地址 */ 
            LOGD("IC_SAR set to dynamic address: 0x%02X\n", regBase->icSar.fields.icSar);
        }

        /* Set TX/RX FIFO thresholds for interrupt generation */
        regBase->icTxTl = 0;  /* TX threshold: generate interrupt when TX FIFO is empty */
        regBase->icRxTl = 0;  /* RX threshold: generate interrupt when RX FIFO has 1+ bytes */
        LOGD("FIFO thresholds set: TX_TL=%d, RX_TL=%d\n", regBase->icTxTl, regBase->icRxTl);

        /* 配置从机模式中断掩码 - 在使能设备之前配置 */
        /* TX_EMPTY将在WR_REQ处理过程中根据需要动态启用 */
        U32 intrMask = SMBUS_SLAVE_INIT_INTR_MASK;
        regBase->icIntrMask.value = intrMask;
        U32 smbusIntrMask = 0;
        if (dev->smbFeatures.arpEnb) {
            smbusIntrMask |= SMBUS_ARP_INTR_MASK; /* 包含 ARP_DET 等 */
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
            /* Quick Command 通常也是协议中断的一部分 */
            smbusIntrMask |= SMBUS_QUICK_CMD_DET_BIT; 
        }    
        /* 应用 SMBus 特定中断掩码 */
        if (smbusIntrMask != 0) {
            regBase->icSmbusIntrMask.value = smbusIntrMask;
        } else {
             /* 默认行为，或者保持为0 */
             regBase->icSmbusIntrMask.value = 0;
        }

        LOGE("FINAL OVERRIDE: Slave interrupt mask FORCED to 0x%08X (TX_EMPTY initially disabled)\n",
             regBase->icIntrMask.value);
        LOGD("Slave mode smbus initialized with TX_EMPTY disabled - will be enabled on demand:%08X\n", 
              regBase->icSmbusIntrMask.value );
    
        if (dev->smbFeatures.arpEnb) regBase->icSmbusIntrMask.value |= SMBUS_ARP_INTR_MASK;
        LOGD("✓ Slave interrupt mask configured: 0x%08X (WR_REQ|RX_FULL|RD_REQ|TX_ABRT|RX_DONE|STOP_DET)\n",
             regBase->icIntrMask.value);

        /* 3. 最后：统一使能设备 */
        U32 icEnable = 0;
        icEnable |= SMBUS_IC_ENABLE_MASK;   /* Enable */
        icEnable |= SMBUS_IC_SAR_ENABLE_MASK;  /* SMBus Slave TX Enable */

        regBase->icEnable.value = icEnable;
        LOGD("SMBus: Slave probe completed, Device Enabled=0x%x\n", regBase->icEnable.value);

        /* 验证配置是否成功 */
        udelay(100);  /* 短暂延时确保硬件稳定 */

        LOGD("=== Slave Initialization Verification ===\n");
        LOGD("Final IC_CON    = 0x%08X\n", regBase->icCon.value);
        LOGD("Final IC_SAR    = 0x%02X\n", regBase->icSar.fields.icSar);
        LOGD("Final IC_ENABLE = 0x%08X\n", regBase->icEnable.value);
        LOGD("Final IC_INTR_MASK = 0x%08X\n", regBase->icIntrMask.value);
        LOGD("Final IC_INTR_SMBUS_MASK = 0x%08X\n", regBase->icSmbusIntrMask.value);
        LOGD("Final IC_TX_TL  = %d\n", regBase->icTxTl);
        LOGD("Final IC_RX_TL  = %d\n", regBase->icRxTl);
    } 
    LOGD("SMBus: Slave probe completed successfully\n");
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
 * @brief SMBus slave unprobe function for cleanup
 * @details Disables SMBus controller, clears configuration, and removes
 *          interrupt handlers for slave mode. This function performs
 *          cleanup operations when shutting down slave mode.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Compatible with I2C's i2c_dw_unprobe_slave interface
 * @note Disables controller and clears slave configuration
 * @note Removes interrupt handlers and frees slave buffers
 * @note Performs graceful shutdown of slave operations
 * @warning This function should only be called in Slave mode

 */
S32 smbusUnprobeSlave(SmbusDev_s *dev)
{
    SMBUS_CHECK_PARAM_RETURN(dev == NULL || dev->regBase == NULL, -EINVAL,
                            "SMBus: Invalid device structure");

    /* Disable controller */
    smbusDisable(dev);

    /* Clear slave configuration */
    dev->slaveCfg = 0;
    /* Note: Slave buffer deallocation is handled in smbusDeInit function */

    LOGD("SMBus: Slave unprobe completed successfully\n");
    return EXIT_SUCCESS;
}

/**
 * @brief SMBus Master/Slave mode switch core function
 * @details Implements the core LOGEc for switching between Master and Slave modes
 *          at the HAL layer. This function handles the hardware reconfiguration
 *          required to safely switch between operational modes.
 * @param[in] dev Pointer to SMBus device structure
 * @param[in] targetMode Target mode to switch to (SMBUS_MODE_MASTER or SMBUS_MODE_SLAVE)
 * @return EXIT_SUCCESS on successful mode switch, negative error code on failure:
 *         -EINVAL: Invalid parameters (NULL dev or invalid target mode)
 *         -EBUSY: Bus is busy and cannot be switched
 *         -EIO: Hardware configuration failed
 *         -ETIMEDOUT: Bus idle wait timeout
 *
 * @note Disables controller before reconfiguration
 * @note Waits for bus to be idle before mode switch
 * @note Configures controller for target mode (master or slave)
 * @note Re-enables controller after successful reconfiguration
 * @note Handles both master-to-slave and slave-to-master transitions
 * @warning This function should be called with proper device locking
 * @warning Controller is temporarily disabled during mode switch
 * @warning All ongoing transfers will be aborted during mode switch
 */
static S32 smbusModeSwitchCore(SmbusDev_s *dev, SmbusMode_e targetMode)
{
    S32 ret = EXIT_SUCCESS;
    U32 speed_val, ti2c_poll, enable_sts;
    U32 max_t_poll_count;
    U32 i;

    SMBUS_CHECK_PARAM(dev == NULL || dev->regBase == NULL, -EINVAL,
                    "SMBus: Invalid device structure for mode switch");

    SMBUS_CHECK_PARAM(targetMode != DW_SMBUS_MODE_MASTER && targetMode != DW_SMBUS_MODE_SLAVE,
                      -EINVAL, "SMBus: Invalid target mode %d for mode switch", targetMode);

    /* Check if already in target mode */
    if (dev->mode == targetMode) {
        LOGD("SMBus: Already in %s mode, no switch needed\n",
             targetMode == DW_SMBUS_MODE_MASTER ? "master" : "slave");
        ret = EXIT_SUCCESS;
        goto exit;
    }

    LOGD("SMBus: Switching from %s to %s mode\n",
         dev->mode == DW_SMBUS_MODE_MASTER ? "master" : "slave",
         targetMode == DW_SMBUS_MODE_MASTER ? "master" : "slave");

    /* Step 1: Wait for bus to be idle before disable */
    ret = smbusWaitBusNotBusy(dev);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: Bus busy during mode switch\n");
        ret = -EBUSY;
        goto exit;
    }

    /* Step 2: Read current speed configuration and calculate timing parameters */
    speed_val = dev->regBase->icCon.value & SMBUS_IC_CON_SPEED_MASK;

    /* Calculate delay parameters based on current speed (same as i2c-designware-master.c) */
    switch (speed_val) {
        case SMBUS_IC_CON_SPEED_STD:
            ti2c_poll = 100;  /* 100us delays for standard speed */
            max_t_poll_count = 10000;  /* 1s total timeout */
            break;
        case SMBUS_IC_CON_SPEED_FAST:
            ti2c_poll = 25;   /* 25us delays for fast speed */
            max_t_poll_count = 40000;  /* 1s total timeout */
            break;
        case SMBUS_IC_CON_SPEED_HIGH:
            ti2c_poll = 10;   /* 10us delays for high speed */
            max_t_poll_count = 100000; /* 1s total timeout */
            break;
        default:
            LOGE("SMBus: Invalid speed setting 0x%X for mode switch\n", speed_val);
            ret = -EINVAL;
            goto exit;
    }

    /* Step 3: Disable controller */
    smbusDisable(dev);

    /* Step 4: Wait for controller to be fully disabled with timing based on speed */
    for (i = 0; i < max_t_poll_count; i++) {
        enable_sts = dev->regBase->icEnableStatus;
        enable_sts = enable_sts & SMBUS_IC_ENABLE_STATUS_IC_EN;
        if (!enable_sts) {
            LOGD("SMBus: Controller disabled after %u attempts (delay=%u us)\n", i + 1, ti2c_poll);
            break;
        }
        /* Delay based on speed setting */
        udelay(ti2c_poll);
    }

    /* Step 5: Check if disable failed */
    if (i == max_t_poll_count) {
        LOGE("SMBus: Failed to disable controller within timeout\n");
        /* Restore enable state and return error */
        smbusEnable(dev);
        ret = -ETIMEDOUT;
        goto exit;
    }

    /* Step 6: Switch mode in software */
    dev->mode = targetMode;

    /* Step 7: Configure hardware for the target mode */
    if (targetMode == DW_SMBUS_MODE_MASTER) {
        /* Switch to master mode */
        smbusConfigureMaster(dev);

        /* Apply master configuration to hardware */
        dev->regBase->icCon.value = dev->masterCfg;
        LOGD("SMBus: Configured for master mode, cfg=0x%08X\n", dev->masterCfg);
            /* Step 9: Configure proper interrupt mask for the target mode */
        /* CRITICAL FIX: Ensure RX_FULL is always enabled for read operations */
        U32 masterMask = SMBUS_INTR_TX_ABRT |      /* TX abort */
                           SMBUS_INTR_STOP_DET |     /* Stop detection */
                           SMBUS_INTR_RX_FULL;       /* RX FIFO Full - CRITICAL FIX */

        dev->regBase->icIntrMask.value = masterMask;
        LOGD("SMBus: Set Master mode interrupt mask=0x%08X\n", dev->regBase->icIntrMask.value);
    } else {
        /* Switch to slave mode */
        smbusConfigureSlave(dev);

        /* Apply slave configuration to hardware */
        dev->regBase->icCon.value = dev->slaveCfg;

        /* Set slave address for slave mode */
        ret = smbusSetSlaveAddr(dev);
        if (ret != EXIT_SUCCESS) {
            LOGE("SMBus: Failed to set slave address during mode switch: %d\n", ret);
            goto exit;
        }

        LOGD("SMBus: Configured for slave mode, cfg=0x%08X, addr=0x%02X\n",
             dev->slaveCfg, dev->slaveAddr);
        /* Slave mode: Enable only interrupts needed for slave operations */
        dev->regBase->icIntrMask.value = SMBUS_SLAVE_INTR_MASK;

        LOGE("✓ Slave mode interrupt mask set: 0x%08X (expected: 0x2F4)\n", dev->regBase->icIntrMask.value);

        /* Verify the write was successful */
        U32 readBack = dev->regBase->icIntrMask.value;
        if (readBack != SMBUS_SLAVE_INTR_MASK) {
            LOGE("✗ Slave interrupt mask write failed! Expected: 0x%08X, Read: 0x%08X\n", SMBUS_SLAVE_INTR_MASK, readBack);
        }
    }
    /* Step 8: Re-enable controller */
    smbusEnable(dev);
    LOGE("SMBus: Successfully switched to %s mode\n",
         targetMode == DW_SMBUS_MODE_MASTER ? "master" : "slave");

exit:
    return ret;
}

/**
 * @brief SMBus receive length adjustment handler (for Block Read)
 * @details Called when first byte (length) is received in Block Read.
 *          Adjusts buffer length and re-enables TX_EMPTY to continue transfer.
 * @param[in] dev Pointer to SMBus device structure
 * @param[in] len Received length byte from slave
 * @return Adjusted total length (including PEC if enabled)
 *
 * @note Follows DW I2C Databook flow: receive len → adjust → re-enable TX_EMPTY
 * @note This is critical for Block Read protocol compliance
 */
static U8 smbusDwRecvLen(SmbusDev_s *dev, U8 len)
{
    SmbusMsg_s *msgs = dev->msgs;
    U32 flags = msgs[dev->msgReadIdx].flags;
    U32 intrMask;

    /*
     * Adjust the buffer length and mask the flag
     * after receiving the first byte.
     */
    len += (flags & SMBUS_M_PEC) ? 2 : 1;  ///< +1 for cmd, +1 for PEC if enabled 
    /* Adjust tx buffer len logic if your driver uses it to trigger reads 
     * This part depends heavily on how your TX_EMPTY logic uses masterTxBufLen 
     * Assuming logic: generate 'remaining' number of READ commands 
     */
    dev->masterTxBufLen = len - 1;

    msgs[dev->msgReadIdx].len = len;
    msgs[dev->msgReadIdx].flags &= ~SMBUS_M_RECV_LEN;
    /*
     * Received buffer length, re-enable TX_EMPTY interrupt
     * to resume the SMBus Block Read transaction.
     * This is the key mechanism from DW I2C Databook.
     */
    intrMask = dev->regBase->icIntrMask.value;
    intrMask |= SMBUS_INTR_TX_EMPTY;
    dev->regBase->icIntrMask.value = intrMask;
    
    LOGD("SMBus: Block Read len adjusted to %u, re-enabled TX_EMPTY\n", len);

    LOGD("RecvLen: LenByte=%d, PEC_Flag=%s, FinalTotalLen=%d\n", 
         len, (flags & SMBUS_M_PEC) ? "SET" : "CLEAR", len);
    return len;
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
    U8 *rxBufPtr = (U8 *)buf; ///< 转换入参指针类型方便操作

    if (pDrvData == NULL || buf == NULL) {
        LOGE("SMBus: Invalid parameters in read ISR\n");
        return;
    }

    dev = &pDrvData->pSmbusDev;
    regBase = dev->regBase;
    msgs = dev->msgs;

    if (regBase == NULL || msgs == NULL) {
        LOGE("SMBus: Invalid device or message in read ISR\n");
        return;
    }

    /* Iterate through all read messages */
    for (; dev->msgReadIdx < dev->msgsNum; dev->msgReadIdx++) {
        U32 tmp;
        U32 flags;
        /* 当前这条消息期望的总长度（可能在 RECV_LEN 处理中动态更新） */
        U32 msgTotalLen; 

        LOGD("SMBus: Processing msg[%d], flags=0x%X, status=0x%X\n",
             dev->msgReadIdx, msgs[dev->msgReadIdx].flags, dev->status);

        /* Skip write messages */
        if (!(msgs[dev->msgReadIdx].flags & SMBUS_M_RD)) {
            continue;
        }

        /* 获取当前 buffer 中已经积累的数据长度。
         * 如果是新消息，rxLength 应在处理完上一条后被清零，这里就是 0。
         * 如果是断点续传，这里就是上次中断留下的长度。
         */
        U32 currentRxOffset = pDrvData->rxLength;
        msgTotalLen = msgs[dev->msgReadIdx].len;

        /* Read actual FIFO level from hardware */
        rxValid = regBase->icRxflr;
        LOGD("SMBus: HW FIFO level=%u, accumulated=%u, expect=%u\n", 
             rxValid, currentRxOffset, msgTotalLen);

        /* Drain FIFO while data is available */
        while (rxValid > 0) {
            flags = msgs[dev->msgReadIdx].flags;

            /* 安全检查：防止 RX Buffer 溢出 */
            if (currentRxOffset >= maxLen) {
                LOGE("SMBus: RX buffer overflow! maxLen=%u\n", maxLen);
                dev->status |= SMBUS_ERR_RX_OVER;
                return;
            }

            /* Read data from hardware FIFO */
            tmp = regBase->icDataCmd.value;
            tmp &= 0xFF;  /* Extract data byte */
            LOGD("ISR Read: Byte[%u] = 0x%02X (MsgIdx=%d, Offset=%u, TotalExpected=%u)\n", 
                 pDrvData->rxLength, tmp, dev->msgReadIdx, currentRxOffset, msgTotalLen);
            /* Handle Block Read length byte (I2C_M_RECV_LEN) */
            if (flags & SMBUS_M_RECV_LEN) {
                /*
                 * Logic for Block Read: First byte received is the length.
                 * We need to update the expected message length.
                 */
                if (currentRxOffset == 0) {
                    if (!tmp || tmp > SMBUS_BLOCK_MAXLEN) {
                        tmp = 1;  /* Minimal valid length to complete transfer */
                    }

                    /* 注意：这里 smbusDwRecvLen 应该只更新硬件/msg状态，不应重置 rxLength */
                    msgTotalLen = smbusDwRecvLen(dev, tmp);
                
                    /* 更新消息结构体中的长度，确保后续判断完成条件正确 */
                    msgs[dev->msgReadIdx].len = msgTotalLen;
                    LOGD("ISR RecvLen: New MsgTotalLen = %u\n", msgTotalLen);
                    /* XferMsg 里停止操作。现在知道了总长度，
                     * 需要让 TX 逻辑(smbusDwXferMsg)醒过来，把剩余的 (Len-1) 个 CMD 发出去。
                     */
                    U32 mask = regBase->icIntrMask.value;
                    if (!(mask & SMBUS_INTR_TX_EMPTY)) {
                         regBase->icIntrMask.value = mask | SMBUS_INTR_TX_EMPTY;
                         LOGD("ISR: Re-enabling TX_EMPTY to queue remaining block bytes\n");
                    }
                }
            }

            /* Store received byte into the PASSED buffer at current offset */
            rxBufPtr[currentRxOffset++] = (U8)tmp;
            
            /* Update global tracking */
            pDrvData->rxLength = currentRxOffset;
            dev->rxOutstanding--; ///< 减少硬件待接收计数
            rxValid--;
            
            /* Check if this specific message is fully received */
            /* 注意：RECV_LEN 场景下 msgTotalLen 包含了长度字节本身 */
            if (currentRxOffset >= msgTotalLen) {
                break; ///< Message complete, exit FIFO loop to handle copy
            }
        }

        /* Check if current message is complete */
        if (pDrvData->rxLength >= msgTotalLen) {
            /* * Message complete!
             * Copy data from the Driver RX Buffer (入参 buf) to the User Message Buffer.
             */
            if (msgs[dev->msgReadIdx].buf != NULL) {
                memcpy(msgs[dev->msgReadIdx].buf, rxBufPtr, msgTotalLen);
            }
            
            /* Clear read-in-progress flag and reset Rx accumulator for next msg */
            dev->status &= ~SMBUS_STATUS_READ_IN_PROGRESS;
            pDrvData->rxLength = 0; // Reset for next message
            
            LOGD("SMBus: Message[%d] read complete, %u bytes copied\n", 
                 dev->msgReadIdx, msgTotalLen);
        } else {
            /* * Message NOT complete.
             * Set flag to indicate we are waiting for more data (next interrupt).
             * Data stays in rxBufPtr (pDrvData->rxBuffer), rxLength preserves offset.
             */
            dev->status |= SMBUS_STATUS_READ_IN_PROGRESS;
            LOGD("SMBus: Read incomplete, have %u/%u bytes\n", 
                 pDrvData->rxLength, msgTotalLen);
            return; // Exit ISR, wait for next RX_FULL
        }
    }
    /* 1. 只有当所有软件层面的消息都处理完时，才检查残留 */
    if (dev->msgReadIdx >= dev->msgsNum) {  
        /* 2. 再次检查硬件 FIFO 是否为空 */
        U32 residual = regBase->icRxflr;
        
        if (residual > 0) {
            LOGW("SMBus: Msgs done but FIFO has %u ghost bytes (outstanding=%d) - DRAINING!\n", 
                 residual, dev->rxOutstanding);        
            /* 3. 强制排空 FIFO */
            while (regBase->icRxflr > 0) {
                volatile U32 dummy = regBase->icDataCmd.value;
                (void)dummy;      
                /* 4. 同步修正 rxOutstanding 计数，确保上层 ISR 能正确判断结束条件 */
                if (dev->rxOutstanding > 0) {
                    dev->rxOutstanding--;
                }
            }
        }
    }
    LOGD("SMBus: All read messages processed\n");
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
    SmbusMsg_s *msgs;
    SmbusMsg_s *msg;
    U32 txLimit, rxLimit;
    U32 addr;
    U32 bufLen;
    U8 *buf;
    Bool needRestart = FALSE;
    U32 flr;
    U32 flags;
    U32 cmd;

    if (dev == NULL || dev->regBase == NULL || dev->msgs == NULL) {
        LOGE("SMBus: Invalid device or messages in xfer_msg\n");
        return;
    }

    regBase = dev->regBase;
    msgs = dev->msgs;

    /* Get target address from first message */
    addr = msgs[dev->msgWriteIdx].addr;

    /* Main message processing loop */
    for (; dev->msgWriteIdx < dev->msgsNum; dev->msgWriteIdx++) {
        msg = &msgs[dev->msgWriteIdx];
        flags = msg->flags;

        LOGD("SMBus: Processing msg[%d/%d], flags=0x%X, len=%d, status=0x%X\n",
             dev->msgWriteIdx, dev->msgsNum, flags, msg->len, dev->status);

        /* 
         * Verify target address hasn't changed within message array
         * All messages in a transfer must use same target address
         */
        if (msg->addr != addr) {
            LOGE("SMBus: Target address changed (0x%02X->0x%02X)\n", 
                 addr, msg->addr);
            dev->msgErr = -EINVAL;
            break;
        }

        /* Initialize buffer pointers for new message */
        if (!(dev->status & SMBUS_STATUS_WRITE_IN_PROGRESS)) {
            buf = msg->buf;
            bufLen = msg->len;

            /*
             * If both IC_EMPTYFIFO_HOLD_MASTER_EN and IC_RESTART_EN are set,
             * we must manually set RESTART bit between messages
             */
            if ((dev->masterCfg & SMBUS_IC_CON_RESTART_EN) && 
                (dev->msgWriteIdx > 0)) {
                needRestart = TRUE;
                LOGD("SMBus: RESTART required for msg[%d]\n", dev->msgWriteIdx);
            }
        } else {
            /* Continue with current buffer position */
            buf = dev->masterTxBuf;
            bufLen = dev->masterTxBufLen;
            if ((flags & SMBUS_M_RECV_LEN) && (msg->len < SMBUS_MAX_BLOCK_LEN)) {
                /* * 说明 ISR 已经收到了长度字节，并将 msg->len 修正为真实长度(例如 9)。
                 * 之前我们发了 1 个 CMD 用于读长度，所以剩余需要发送的 CMD 数量是 len - 1。
                 * 之前的 masterTxBufLen 保存的是 32 (33-1)，那是错误的，必须覆盖。
                 */
                if (msg->len >= 1) {
                    bufLen = msg->len - 1; 
                } else {
                    bufLen = 0;
                }
                LOGD("SMBus: Resuming RECV_LEN msg, adjusted remaining cmds to %d\n", bufLen);
            }
        }

        /*
         * CRITICAL: Read FIFO levels dynamically on each iteration
         * as recommended by DW_apb_i2c databook section 7.5
         */
        flr = regBase->icTxflr;
        txLimit = dev->txFifoDepth - flr;

        flr = regBase->icRxflr;
        rxLimit = dev->rxFifoDepth - flr;

        LOGD("SMBus: FIFO status - TX: %d/%d, RX: %d/%d, rxOut=%d\n",
             txLimit, dev->txFifoDepth, rxLimit, dev->rxFifoDepth, 
             dev->rxOutstanding);

        /* Process data while FIFO has space and data remains */
        while (bufLen > 0 && txLimit > 0 && rxLimit > 0) {
            cmd = 0;
            if (flags & SMBUS_M_RECV_LEN) {
                /* * 如果是 Block Read，且 msg->len 还是初始值(例如 33)，
                 * 说明我们还没有读到长度字节。
                 * 此时，严禁发送超过 1 个 Read Command。
                 */
                if ((msg->len > SMBUS_MAX_BLOCK_LEN) && (dev->rxOutstanding >= 1)) {
                    LOGD("SMBus: RECV_LEN phase 1 - waiting for length byte via ISR\n");
                    break; ///< 跳出 while 循环，停止填充 FIFO
                }
            }
            /*
             * Set STOP bit on last byte of last message
             * Exception: I2C_M_RECV_LEN requires dynamic length adjustment
             */
            if (((dev->msgWriteIdx == dev->msgsNum - 1) || 
                 (flags & SMBUS_M_STOP)) &&
                (bufLen == 1) && 
                !(flags & SMBUS_M_RECV_LEN)) {
                cmd |= SMBUS_IC_DATA_CMD_STOP;
                LOGD("SMBus: Setting STOP bit on final byte\n");
            }

            /* Set RESTART bit if needed between messages */
            if (needRestart) {
                cmd |= SMBUS_IC_DATA_CMD_RESTART;
                needRestart = FALSE;
                LOGD("SMBus: Setting RESTART bit\n");
            }

            /* Handle read vs write operations */
            if (flags & SMBUS_M_RD) {
                /*
                 * CRITICAL: Prevent RX FIFO overflow
                 * Check outstanding read commands against FIFO depth
                 */
                if (dev->rxOutstanding >= dev->rxFifoDepth) {
                    LOGD("SMBus: RX FIFO full, breaking (outstanding=%d)\n",
                         dev->rxOutstanding);
                    break;
                }

                /* Write read command to TX FIFO */
                cmd |= SMBUS_IC_DATA_CMD_READ_CMD;
                regBase->icDataCmd.value = cmd;
                
                rxLimit--;
                dev->rxOutstanding++;
                
                LOGD("SMBus: Queued READ cmd=0x%03X, rxOut now=%d\n", 
                    cmd, dev->rxOutstanding);
            } else {
                /* Write data byte to TX FIFO */
                cmd |= (*buf & 0xFF);
                regBase->icDataCmd.value = cmd;
                
                buf++;
                
                LOGD("SMBus: Queued WRITE cmd=0x%03X\n", cmd);
            }

            txLimit--;
            bufLen--;
        }

        /* Save current buffer state */
        if (buf != NULL && bufLen > 0 && bufLen <= 32) {
            /* Copy data to masterTxBuf array */
            U32 copyLen = (bufLen < sizeof(dev->masterTxBuf)) ? bufLen : sizeof(dev->masterTxBuf);
            memcpy(dev->masterTxBuf, buf, copyLen);
            dev->masterTxBufLen = bufLen;
            LOGD("SMBus: Saved %u bytes to masterTxBuf array\n", copyLen);
        } else {
            dev->masterTxBufLen = 0;
        }

        /*
        * Handle SMBus Block Read with dynamic length (first byte is count)
        * Disable TX_EMPTY interrupt while waiting for length byte
        * to avoid interrupt flood
        */
        if (bufLen > 0) {
            dev->status |= SMBUS_STATUS_WRITE_IN_PROGRESS;
            if (flags & SMBUS_M_RECV_LEN) {
                LOGD("SMBus: RECV_LEN pause, disabling TX_EMPTY until ISR\n");
            } else {
                LOGD("SMBus: %d bytes remaining in current msg\n", bufLen);
            }
            break;
        } else {
            dev->status &= ~SMBUS_STATUS_WRITE_IN_PROGRESS;
            if (flags & SMBUS_M_RECV_LEN) {
                LOGD("SMBus: Message[%d] queued completely (RECV_LEN)\n", dev->msgWriteIdx);
            } else {
                LOGD("SMBus: Message[%d] queued completely\n", dev->msgWriteIdx);
            }
        }
    }
    
    /*
     * If all messages are queued, disable TX_EMPTY interrupt
     * and enable RX_FULL for reading responses
     */
    if (dev->msgWriteIdx >= dev->msgsNum) {
        U32 timeout = 10000;
        while (timeout-- && regBase->icStatus.fields.tfe == 0) {
        udelay(1);
    }
    LOGD("SMBus: All queued, TFE=%d, timeout=%d\n", 
         regBase->icStatus.fields.tfe, timeout);
    }

    if (dev->msgErr) {
        LOGE("SMBus: Error detected (0x%X), disabling all interrupts\n",
             dev->msgErr);
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
S32 smbusI2cTransfer(SmbusDev_s *dev, SmbusMsg_s *msgs, S32 num)
{
    S32 ret = EXIT_SUCCESS;

    /* 1. Parameter Validation */
    if (dev == NULL || msgs == NULL || num <= 0) {
        LOGE("SMBus: Invalid parameters for I2C transfer\n");
        return -EINVAL;
    }

    /* ============================================================
     * BRANCH: SLAVE MODE HANDLING
     * 从机模式：仅仅是操作内存缓冲区 (Producer/Consumer 模型)
     * 不需要操作硬件寄存器发起传输，也不需要等待总线空闲
     * ============================================================*/
    if (dev->mode == SMBUS_MODE_SLAVE) {
        S32 i;
        LOGD("SMBus: Processing SLAVE transfer request (%d msgs)\n", num);

        for (i = 0; i < num; i++) {
            SmbusMsg_s *msg = &msgs[i];

            /* Parameter Check for Buffer */
            if (msg->len == 0 || msg->buf == NULL) {
                LOGE("SMBus: Slave msg[%d] invalid len/buf\n", i);
                return -EINVAL;
            }

            if (msg->flags & SMBUS_M_RD) {
                /* * [SLAVE READ] = 用户取走 ISR 接收到的数据
                 * 逻辑：Copy RX Buffer -> User Buffer -> Clear RX Buffer
                 */
                if (dev->slaveValidRxLen > 0) {
                    U32 copyLen = (msg->len < dev->slaveValidRxLen) ? msg->len : dev->slaveValidRxLen;
                    
                    if (dev->slaveRxBuf != NULL) {
                        memcpy(msg->buf, dev->slaveRxBuf, copyLen);
                        
                        /* 清除接收缓冲区状态 */
                        memset(dev->slaveRxBuf, 0, dev->slaveValidRxLen);
                        dev->slaveValidRxLen = 0;
                        /* 这里可以记录实际读取长度，但标准 I2C Msg 结构通常不回填 len */
                    }
                } else {
                    /* 无数据可用 */
                    /* 在 Slave Read 中，无数据通常不算错误，而是返回 0 长度 */
                    /* 为了让调用者知道没读到数据，可以将 buffer 清零 */
                    memset(msg->buf, 0, msg->len);
                }

            } else {
                /* * [SLAVE WRITE] = 用户准备数据给 ISR 发送
                 * 逻辑：User Buffer -> TX Buffer -> Set TX Valid Length
                 */
                if (dev->slaveTxBuf != NULL) {
                    /* 先标记数据无效，防止拷贝过程中被 ISR 误读 */
                    dev->slaveValidTxLen = 0;
                    
                    /* 限制拷贝长度不超过硬件/缓冲区最大限制 */
                    U32 writeLen = (msg->len > SMBUS_MAX_BLOCK_LEN) ? SMBUS_MAX_BLOCK_LEN : msg->len;
                    
                    memset(dev->slaveTxBuf, 0, SMBUS_MAX_BLOCK_LEN);
                    memcpy(dev->slaveTxBuf, msg->buf, writeLen);
                    
                    /* 提交数据：ISR 现在可以看到有效长度了 */
                    dev->slaveValidTxLen = writeLen;
                } else {
                    LOGE("SMBus: Slave TX buffer is NULL\n");
                    return -EINVAL;
                }
            }

        }
        
        /* Slave 模式下，直接返回处理的消息数，表示成功 */
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
    if (num == 1 && msgs[0].len == 0 && dev->smbFeatures.quickCmdEnb) {
        LOGD("SMBus: Quick Command detected during xfer init\n");

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
        .setSlaveAddr = smbusSetSlaveAddr,
        .enable = smbusEnable,
        .disable = smbusDisable,
        .modeSwitchCore = smbusModeSwitchCore,
        .i2cTransfer = smbusI2cTransfer,
        .smbusDwRead = smbusDwRead,
        .smbusDwXferMsg = smbusDwXferMsg,
        .configureSlave = smbusConfigureSlave,
    };
    return &halOps;
}
