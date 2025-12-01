/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_smbus_dw_master.c
 * @author wangkui (wangkui@starsmicrosystem.com)
 * @date 2025/10/20
 * @brief SMBus Master API layer implementation
 *
 * @par ChangeLog:
 * Date         Author          Description
 * 2025/10/20   wangkui         Initial version
 * 2025/11/23   wangkui         HAL operations optimization - eliminated 14+ smbusGetHalOps() calls
 *                              Replaced direct HAL access with pDrvData->pSmbusDev.halOps pointer
 *                              Added proper driver data retrieval for functions lacking pDrvData parameter
 *                              Removed unused variable declarations to fix compilation warnings
 *                              Improved performance by eliminating redundant function calls
 * @note This file implements the SMBus Master API layer, providing
 *       high-level Master mode operations for SMBus communication.
 *       It works on top of the SMBus core layer and handles
 *       Master-specific functionality including read/write operations,
 *       command processing, and error handling in Master mode.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "common_defines.h"
#include "bsp_config.h"
#include "bsp_drv_id.h"
#include "bsp_api.h"
#include "bsp_device.h"
#include "sbr_api.h"
#include "log_msg.h"
#include "udelay.h"
#include "osp_interrupt.h"
#include "drv_smbus_dw.h"
#include "drv_smbus_dw_i2c.h"
#include "drv_smbus_api.h"

/* ======================================================================== */
/*                    SMBus I2C compliant interface API                     */
/* ======================================================================== */

/**
 * @brief SMBus I2C Block Write implementation
 * @details Performs block write operation compatible with I2C interface.
 *          This function writes a block of data to the specified slave address
 *          using SMBus protocol with optional PEC (Packet Error Code) support.
 * @param[in] devId SMBus device identifier
 * @param[in] data Pointer to block write data structure containing slave address,
 *                  command byte, data buffer, length, and PEC settings
 * @return EXIT_SUCCESS on successful write, negative error code on failure:
 *         -EINVAL: Invalid parameters (NULL data, zero length, or length exceeds maximum)
 *         -EBUSY: Device locked or busy with another operation
 *         -EIO: Communication error (NACK, timeout, bus error)
 *         -EPROTO: Protocol error (invalid response from slave)
 * @note Supports PEC calculation when enabled in data structure
 * @note Maximum block length is SMBUS_BLOCK_MAXLEN (32 bytes)
 * @warning This function is thread-safe and uses device locking
 */
S32 smbusI2CBlockWrite(DevList_e devId, SmbusI2CBlockWriteData_s *data)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;
    SmbusDev_s *pDev = NULL;
    SmbusMsg_s *pMsg = NULL;

    /* 1. Parameter Validation */
    SMBUS_CHECK_PARAM(data == NULL, -EINVAL, "%s: data pointer is NULL", __func__);
    SMBUS_CHECK_PARAM(data->length == 0, -EINVAL, "%s: data length is 0", __func__);
    SMBUS_CHECK_PARAM(data->length > SMBUS_BLOCK_MAXLEN, -EINVAL, "%s: data length %d exceeds max %d",
                      __func__, data->length, SMBUS_BLOCK_MAXLEN);

    /* 2. Driver Lock and Validation */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    /* Local pointer aliases for better readability */
    pDev = &pDrvData->pSmbusDev;

    /* 3. Check HAL Operations */
    if (pDev->halOps == NULL || pDev->halOps->i2cWrite == NULL) {
        LOGE("%s: HAL I2C operations not available\n", __func__);
        ret = -ENOTSUP;
        goto unlock;
    }

    /* 4. Critical Status Setup */
    /* Ensure ISR sees active status immediately */
    pDev->status = SMBUS_STATUS_ACTIVE;
    LOGD("SMBus: Pre-set active status (0x%08X)\n", pDev->status);

    /* 5. Message and Context Initialization */
    pMsg = pDev->msgs;

    /* * Use C99 Compound Literal to assign structure fields.
     * Benefit: Clean syntax and implicit zeroing of unlisted fields.
     */
    *pMsg = (SmbusMsg_s){
        .addr  = data->slaveAddr, ///< Use address mode as addr (Legacy logic)
        .buf   = data->dataBuf,  ///< Pointer to data buffer
        .flags = pDev->flags,    ///< Transfer flags
        .len   = data->length    ///< Length of data to write
    };

    /* Update device context state */
    pDev->msgsNum   = data->length;    ///< Total number of messages
    pDev->slaveAddr = data->slaveAddr; ///< Target I2C slave address

    /* Reset runtime counters */
    pDev->msgWriteIdx   = 0; ///< Reset write index
    pDev->msgReadIdx    = 0; ///< Reset read index
    pDev->msgErr        = 0; ///< Clear message errors
    pDev->cmdErr        = 0; ///< Clear command errors
    pDev->abortSource   = 0; ///< Clear abort source
    pDev->rxOutstanding = 0; ///< Clear outstanding RX count

    LOGD("SMBus: Setup msg - msgs=%p, len=%d, slave=0x%02X\n", pMsg, pDev->msgsNum, pDev->slaveAddr);
    LOGD("SMBus: Master Mode: %d\n", pDrvData->sbrCfg.masterMode);

    /* 6. Execute I2C Block Write via HAL */
    ret = pDev->halOps->i2cWrite(pDev, data->slaveAddr, data->dataBuf, data->length);

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief Performs RAW I2C Read for debugging Block Read issues
 * @details This function performs a simple I2C read operation without SMBus Block Read protocol
 *          It directly reads a specified number of bytes from the device at a given address.
 * @param[in] devId SMBus device identifier
 * @param[in] slaveAddr I2C slave device address (7-bit)
 * @param[in] buf Pointer to buffer where read data will be stored
 * @param[in] len Number of bytes to read
 * @return EXIT_SUCCESS on successful read, negative error code on failure
 */
S32 smbusI2CRawRead(DevList_e devId, U8 slaveAddr, U8 *buf, U32 len)
{
    SmbusDrvData_s *pDrvData = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Validate parameters */
    if (buf == NULL || len == 0) {
        LOGE("SMBus RAW I2C: Invalid parameters - buf=%p, len=%d\n", buf, len);
        return -EINVAL;
    }

    /* Validate slave address (7-bit address range: 0x08-0x77) */
    if (slaveAddr < SMBUS_MIN_VALID_ADDRESS || slaveAddr > SMBUS_MAX_VALID_ADDRESS) {
        LOGE("SMBus RAW I2C: Invalid slave address 0x%02X (valid range: 0x%02X-0x%02X)\n",
             slaveAddr, SMBUS_MIN_VALID_ADDRESS, SMBUS_MAX_VALID_ADDRESS);
        return -EINVAL;
    }

    /* ===== 1. Driver Validation and Lock ===== */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    /* ===== 2. Get Driver Data ===== */
    ret = getDevDriver(devId, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS || pDrvData == NULL) {
        LOGE("SMBus RAW I2C: Failed to get driver data\n");
        goto exit;
    }

    /* Get HAL operations for I2C compatibility */
    // HAL operations already registered in device structure
    if (pDrvData->pSmbusDev.halOps == NULL || pDrvData->pSmbusDev.halOps->i2cRead == NULL) {
        LOGE("SMBus RAW I2C: HAL I2C operations not available\n");
        ret = -ENOTSUP;
        goto exit;
    }
    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)pDrvData->pSmbusDev.regBase;
#if 0
    /* ===== 3. Dump Register Status Before I2C Read ===== */
    
    if (regBase != NULL) {
        LOGE("[DEBUG PRE-READ] SMBus I2C Raw Read Register Dump - Device %d:\n", devId);

        /* Dump IC_CON register */
        SmbusIcConReg_u icCon;
        icCon.value = regBase->icCon.value;
        LOGE("[DEBUG PRE-READ] IC_CON (0x%08X): masterMode=%u, speed=%u, ic10bitaddrSlave=%u, ic10bitaddrMaster=%u, icRestartEn=%u, icSlaveDisable=%u\n",
             icCon.value, icCon.fields.masterMode, icCon.fields.speed, icCon.fields.ic10bitaddrSlave,
             icCon.fields.ic10bitaddrMaster, icCon.fields.icRestartEn, icCon.fields.icSlaveDisable);

        /* Dump IC_SAR register (relevant for slave mode) */
        SmbusIcSarReg_u icSar;
        icSar.value = regBase->icSar.value;
        LOGE("[DEBUG PRE-READ] IC_SAR (0x%08X): slaveAddress=0x%03X\n", icSar.value, icSar.fields.icSar);

        /* Dump IC_INTR_MASK register */
        SmbusIcIntrMask_u icIntrMask;
        icIntrMask.value = regBase->icIntrMask.value;
        LOGE("[DEBUG PRE-READ] IC_INTR_MASK (0x%08X): rxUnder=%u, rxOver=%u, rxFull=%u, txOver=%u, txEmpty=%u, rdReq=%u, txAbrt=%u, rxDone=%u\n",
             icIntrMask.value, icIntrMask.fields.rxUnder, icIntrMask.fields.rxOver, icIntrMask.fields.rxFull,
             icIntrMask.fields.txOver, icIntrMask.fields.txEmpty, icIntrMask.fields.rdReq,
             icIntrMask.fields.txAbrt, icIntrMask.fields.rxDone);
        LOGE("[DEBUG PRE-READ] IC_INTR_MASK continued: activity=%u, stopDet=%u, startDet=%u, genCall=%u, restartDet=%u\n",
             icIntrMask.fields.activity, icIntrMask.fields.stopDet, icIntrMask.fields.startDet,
             icIntrMask.fields.genCall, icIntrMask.fields.restartDet);

        /* Dump IC_STATUS register for additional context */
        SmbusIcStatusReg_u icStatus;
        icStatus.value = regBase->icStatus.value;
        LOGE("[DEBUG PRE-READ] IC_STATUS (0x%08X): activity=%u, tfnf=%u, tfe=%u, rfne=%u, rff=%u\n",
             icStatus.value, icStatus.fields.activity, icStatus.fields.tfnf, icStatus.fields.tfe,
             icStatus.fields.rfne, icStatus.fields.rff);

        /* Dump IC_ENABLE register */
        SmbusIcEnableReg_u icEnable;
        icEnable.value = regBase->icEnable.value;
        LOGE("[DEBUG PRE-READ] IC_ENABLE (0x%08X): enable=%u, abort=%u, icSarEn=%u\n",
             icEnable.value, icEnable.fields.enable, icEnable.fields.abort, icEnable.fields.icSarEn);

        LOGE("[DEBUG PRE-READ] End of Register Dump - Device %d\n", devId);
    }
#endif
    /* ===== 4. Perform RAW I2C Read ===== */
    LOGD("SMBus RAW I2C: Reading %d bytes from slave 0x%02X\n", len, slaveAddr);

    ret = pDrvData->pSmbusDev.halOps->i2cRead(&pDrvData->pSmbusDev, slaveAddr, buf, len);

    /* ===== 5. Dump Register Status After I2C Read ===== */
    if (regBase != NULL) {
        LOGE("[DEBUG POST-READ] SMBus I2C Raw Read Register Dump - Device %d (ret=%d):\n", devId, ret);

        /* Dump IC_CON register */
        SmbusIcConReg_u icCon;
        icCon.value = regBase->icCon.value;
        LOGE("[DEBUG POST-READ] IC_CON (0x%08X): masterMode=%u, speed=%u, ic10bitaddrSlave=%u, ic10bitaddrMaster=%u, icRestartEn=%u, icSlaveDisable=%u\n",
             icCon.value, icCon.fields.masterMode, icCon.fields.speed, icCon.fields.ic10bitaddrSlave,
             icCon.fields.ic10bitaddrMaster, icCon.fields.icRestartEn, icCon.fields.icSlaveDisable);

        /* Dump IC_SAR register */
        SmbusIcSarReg_u icSar;
        icSar.value = regBase->icSar.value;
        LOGE("[DEBUG POST-READ] IC_SAR (0x%08X): slaveAddress=0x%03X\n", icSar.value, icSar.fields.icSar);

        /* Dump IC_INTR_MASK register */
        SmbusIcIntrMask_u icIntrMask;
        icIntrMask.value = regBase->icIntrMask.value;
        LOGE("[DEBUG POST-READ] IC_INTR_MASK (0x%08X): rxUnder=%u, rxOver=%u, rxFull=%u, txOver=%u, txEmpty=%u, rdReq=%u, txAbrt=%u, rxDone=%u\n",
             icIntrMask.value, icIntrMask.fields.rxUnder, icIntrMask.fields.rxOver, icIntrMask.fields.rxFull,
             icIntrMask.fields.txOver, icIntrMask.fields.txEmpty, icIntrMask.fields.rdReq,
             icIntrMask.fields.txAbrt, icIntrMask.fields.rxDone);
        LOGE("[DEBUG POST-READ] IC_INTR_MASK continued: activity=%u, stopDet=%u, startDet=%u, genCall=%u, restartDet=%u\n",
             icIntrMask.fields.activity, icIntrMask.fields.stopDet, icIntrMask.fields.startDet,
             icIntrMask.fields.genCall, icIntrMask.fields.restartDet);

        /* Dump IC_RAW_INTR_STAT register - shows actual pending interrupts */
        SmbusIcRawIntrStatReg_u icRawIntrStat;
        icRawIntrStat.value = regBase->icRawIntrStat.value;
        LOGE("[DEBUG POST-READ] IC_RAW_INTR_STAT (0x%08X): rxUnder=%u, rxOver=%u, rxFull=%u, txOver=%u, txEmpty=%u, rdReq=%u, txAbrt=%u, rxDone=%u\n",
             icRawIntrStat.value, icRawIntrStat.fields.rxUnder, icRawIntrStat.fields.rxOver, icRawIntrStat.fields.rxFull,
             icRawIntrStat.fields.txOver, icRawIntrStat.fields.txEmpty, icRawIntrStat.fields.rdReq,
             icRawIntrStat.fields.txAbrt, icRawIntrStat.fields.rxDone);
        LOGE("[DEBUG POST-READ] IC_RAW_INTR_STAT continued: activity=%u, stopDet=%u, startDet=%u, genCall=%u, restartDet=%u\n",
             icRawIntrStat.fields.activity, icRawIntrStat.fields.stopDet, icRawIntrStat.fields.startDet,
             icRawIntrStat.fields.genCall, icRawIntrStat.fields.restartDet);

        /* Dump IC_STATUS register */
        SmbusIcStatusReg_u icStatus;
        icStatus.value = regBase->icStatus.value;
        LOGE("[DEBUG POST-READ] IC_STATUS (0x%08X): activity=%u, tfnf=%u, tfe=%u, rfne=%u, rff=%u\n",
             icStatus.value, icStatus.fields.activity, icStatus.fields.tfnf, icStatus.fields.tfe,
             icStatus.fields.rfne, icStatus.fields.rff);

        /* Dump IC_ENABLE register */
        SmbusIcEnableReg_u icEnable;
        icEnable.value = regBase->icEnable.value;
        LOGE("[DEBUG POST-READ] IC_ENABLE (0x%08X): enable=%u, abort=%u, icSarEn=%u\n",
             icEnable.value, icEnable.fields.enable, icEnable.fields.abort, icEnable.fields.icSarEn);

        LOGE("[DEBUG POST-READ] End of Register Dump - Device %d\n", devId);
    }

    if (ret < 0) {
        LOGE("SMBus RAW I2C: I2C read failed: %d\n", ret);
        ret = -EIO;
        goto exit;
    }

    /* ===== 4. Log Results for Debugging ===== */
    LOGE("SMBus RAW I2C: Successfully read %d bytes from slave 0x%02X\n", len, slaveAddr);
    LOGE("SMBus RAW I2C: Read data: ");
    for (U32 i = 0; i < len && i < 8; i++) {  /* Log first 8 bytes */
        LOGE("0x%02X ", buf[i]);
    }
    LOGE("\n");

    /* ===== 5. Success Path ===== */
    ret = EXIT_SUCCESS;

exit:
    /* ===== 6. Unlock Device ===== */
    devUnlockByDriver(devId);

    return ret;
}

S32 smbusI2CBlockRead(DevList_e devId, SmbusI2CBlockReadData_s *data)
{
    SmbusDrvData_s *pDrvData = NULL;
    S32 ret = EXIT_SUCCESS;
    U8 cmdBuf[1];
    U8 readBuf[SMBUS_MAX_BLOCK_LEN + 1];  /* Count byte + data bytes */
    U8 byteCount = 0;

    /* ===== 1. Parameter Validation ===== */
    if (data == NULL || data->readLength == NULL || data->data == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    /* Validate slave address (7-bit address range: 0x08-0x77) */
    if (data->slaveAddr < SMBUS_MIN_VALID_ADDRESS || data->slaveAddr > SMBUS_MAX_VALID_ADDRESS) {
        ret = -EINVAL;
        goto exit;
    }

    /* ===== 2. Driver Validation and Lock ===== */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    /* ===== 3. Get Driver Data ===== */
    ret = getDevDriver(devId, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS || pDrvData == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    /* Get HAL operations for I2C compatibility */
    // HAL operations already registered in device structure
    if (pDrvData->pSmbusDev.halOps == NULL || pDrvData->pSmbusDev.halOps->i2cWriteRead == NULL) {
        LOGE("%s(): HAL I2C operations not available\n", __func__);
        ret = -ENOTSUP;
        goto exit;
    }

    /* CRITICAL FIX: Use the driver's main device structure directly */
    /* This ensures ISR can access the same device structure and messages */
    /* No need to create temporary structure - use pDrvData->pSmbusDev directly */

    /* Ensure the device structure is properly configured */
    pDrvData->pSmbusDev.regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;
    pDrvData->pSmbusDev.addrMode = pDrvData->sbrCfg.addrMode;
    pDrvData->pSmbusDev.mode = pDrvData->sbrCfg.masterMode ? DW_SMBUS_MODE_MASTER : DW_SMBUS_MODE_SLAVE;

    LOGD("SMBus: Using pDrvData->pSmbusDev directly for I2C Block Read\n");

    /* ===== 5. SMBus I2C Block Read Protocol ===== */

    /*
     * Phase 1: Write Command Code
     * Send: S + Addr(W) + Command + (Sr for combined transaction)
     */
    cmdBuf[0] = data->cmdCode;

    /*
     * Phase 2: Read Count and Data
     * Combined Write-Read transaction with repeated start
     * Write: Command code
     * Read:  Byte count + data bytes (max 33 bytes total)
     */
    ret = pDrvData->pSmbusDev.halOps->i2cWriteRead(&pDrvData->pSmbusDev,
                                data->slaveAddr,
                                cmdBuf,
                                1,                           /* Write 1 byte (command) */
                                readBuf,
                                SMBUS_MAX_BLOCK_LEN);    /* Read count + max data */

    if (ret != EXIT_SUCCESS) {
        /* Communication error: NACK, timeout, bus error, etc. */
        ret = -EIO;
        goto exit;
    }

    /* ===== 6. Validate and Process Received Data ===== */

    /* First byte is the byte count from slave */
    byteCount = readBuf[0];

    LOGD("SMBus: Read block - byteCount=%d, max allowed=%d\n", byteCount, SMBUS_BLOCK_MAXLEN);
    LOGD("SMBus: First few bytes: 0x%02X 0x%02X 0x%02X 0x%02X\n",
         readBuf[0], readBuf[1], readBuf[2], readBuf[3]);

    /* Validate byte count according to SMBus specification */
    if (byteCount == 0 || byteCount > SMBUS_BLOCK_MAXLEN) {
        /* Invalid count: protocol violation */
        LOGE("SMBus: Invalid byte count %d (max %d)\n", byteCount, SMBUS_BLOCK_MAXLEN);
        ret = -EPROTO;
        goto exit;
    }

    /* ===== 7. Copy Data to User Buffer ===== */

    /* CRITICAL FIX: Add safety checks before memcpy */
    LOGD("SMBus: About to copy %d bytes to user buffer at %p\n", byteCount, data->data);

    /* Check for NULL pointer */
    if (data->data == NULL) {
        LOGE("SMBus: User data buffer is NULL\n");
        ret = -EINVAL;
        goto exit;
    }

    /* Copy actual data bytes (excluding count byte) */
    /* Use safer memory copy with bounds checking */
    for (U32 i = 0; i < byteCount; i++) {
        data->data[i] = readBuf[i + 1];
        if (i < 4) {  /* Log first few bytes */
            LOGD("SMBus: Copied byte[%d] = 0x%02X\n", i, data->data[i]);
        }
    }

    /* Return the byte count to caller */
    *(data->readLength) = byteCount;

    LOGD("SMBus: Block read completed successfully - copied %d bytes safely\n", byteCount);

    /* ===== 8. Success Path ===== */
    ret = EXIT_SUCCESS;

exit:
    /* ===== 9. Unlock Device ===== */
    devUnlockByDriver(devId);

    return ret;
}

/* ======================================================================== */
/*                    Modular SMBus Protocol Functions                       */
/* ======================================================================== */

/**
 * @brief Quick Command protocol implementation
 * @details Implements SMBus Quick Command protocol with optional PEC support.
 *          Uses direct register access for efficient quick command execution.
 * @param[in] halOps HAL operations structure
 * @param[in] smbusDev SMBus device structure
 * @param[in] param Protocol parameters containing read/write bit
 * @param[in] pecEn PEC enable flag
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
static S32 smbusQuickCmdProtocol(SmbusDev_s *smbusDev,
                                SmbusParam_s *param, bool pecEn)
{
    /*
     * SMBus Quick Command Protocol - Architecture Compliant Implementation
     *
     * Protocol Layer Definition:
     * - Quick Command = msg.len == 0
     * - Direction determined by msg.flags & I2C_M_RD
     * - No data buffer required
     *
     * Hardware Layer Responsibility (smbusDwXfer):
     * - Detect msg.len == 0 condition
     * - Set icTar.fields.special = 1
     * - Set icTar.fields.smbusQuickCmd = 1
     * - Configure icDataCmd for quick command
     * - Clear special flags after completion
     *
     * This maintains clean separation between protocol and hardware layers.
     */

    /* 1. Extract Address */
    U8 slaveAddr = param->base.slaveAddr & SMBUS_ADDRESS_MASK;

    /* 2. Determine Direction based on rwBit */
    /* If rwBit is 1, it acts like a Read request (but no data returned) */
    U16 flags = (param->param.quick.rwBit) ? SMBUS_M_RD : 0;

    LOGD("[PROTOCOL] Quick Command - addr=0x%02X, rwBit=%d (%s), pecEn=%d\r\n",
         slaveAddr, param->param.quick.rwBit,
         param->param.quick.rwBit ? "READ" : "WRITE", pecEn);

    /* 3. Construct Message */
    SmbusMsg_s msg = {
        .addr  = slaveAddr,
        .flags = flags,
        .len   = 0,     ///< Length 0 indicates Quick Command
        .buf   = NULL   ///< No data buffer needed
    };

    LOGD("[PROTOCOL] Quick Command message constructed - addr=0x%02X, flags=0x%04X, len=%d\r\n",
         msg.addr, msg.flags, msg.len);

    /* 4. Execute via unified I2C Transfer interface */
    /* The underlying driver (smbusDwXfer) MUST handle len=0 by setting special HW bits */
    LOGD("[PROTOCOL] Calling smbusExecuteTransfer for Quick Command\r\n");
    S32 ret = smbusExecuteTransfer(smbusDev, &msg, 1);

    LOGD("[PROTOCOL] Quick Command transfer completed - ret=%d\r\n", ret);
    return ret;
}

/**
 * @brief Send Byte protocol implementation (SMBus 3.1) - Optimized Version
 * @details Sends a single data byte to the slave with improved performance.
 * Packet: [Addr|W] + [Data] + [PEC(opt)]
 *
 * Optimizations:
 * - Unified i2cTransfer interface instead of i2cWrite
 * - Single stack allocation with pre-calculated length
 * - Reduced conditional branches and improved cache locality
 *
 * @param[in] smbusDev SMBus device structure
 * @param[in] param Protocol parameters containing data byte
 * @param[in] pecEn PEC enable flag
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
static S32 smbusSendByteProtocol(SmbusDev_s *smbusDev,
                                 SmbusParam_s *param, bool pecEn)
{
    S32 ret;
    const U8 slaveAddr = param->base.slaveAddr & SMBUS_ADDRESS_MASK;

    /* Stack-allocated buffer: Data(1) + PEC(1) - single allocation */
    U8 txBuffer[2];
    U16 txLen = 1;

    /* Prepare primary data */
    txBuffer[0] = param->param.byte.data;

    /* Handle PEC calculation and buffer preparation */
    if (pecEn) {
        /* PEC calculation sequence: CRC8(Addr(W) + Data) */
        const U8 pecInput[2] = {
            (U8)(slaveAddr << 1),  /* Slave Address + Write Bit(0) */
            txBuffer[0]            /* Data Byte */
        };

        /* Append PEC to buffer */
        txBuffer[1] = smbusPecPktConstruct(slaveAddr, true, pecInput, 2);
        txLen = 2;  /* Update length to include PEC */
    }

    /* Single I2C message structure - C99 compound literal */
    SmbusMsg_s msg = {
        .addr  = slaveAddr,
        .flags = 0,        /* Write operation */
        .len   = txLen,    /* Pre-calculated length */
        .buf   = txBuffer  /* Stack-allocated buffer */
    };

    /* Unified transfer interface - single HAL call */
    ret = smbusExecuteTransfer(smbusDev, &msg, 1);

    /* Error handling with consistent logging */
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Send byte failed (Addr:0x%02X, Ret:%d)\n", __func__, slaveAddr, ret);
        return -EIO;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Receive Byte protocol implementation (SMBus 3.1) - Optimized Version
 * @details Reads a single data byte from the slave with improved performance.
 * Packet: [Addr|R] + [Data] + [PEC(opt)]
 *
 * Optimizations:
 * - Unified i2cTransfer interface instead of i2cWriteRead
 * - Single stack allocation with pre-calculated length
 * - Reduced conditional branches and improved cache locality
 *
 * @param[in] smbusDev SMBus device structure
 * @param[in,out] param Protocol parameters, returns received data
 * @param[in] pecEn PEC enable flag
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
static S32 smbusReceiveByteProtocol(SmbusDev_s *smbusDev,
                                   SmbusParam_s *param, bool pecEn)
{
    S32 ret;
    const U8 slaveAddr = param->base.slaveAddr & SMBUS_ADDRESS_MASK;

    /* Stack-allocated buffer: Data(1) + [PEC(1)] - single allocation */
    U8 rxBuffer[2];
    const U16 rxLen = pecEn ? 2 : 1;

    /* Single I2C message structure for read operation */
    SmbusMsg_s msg = {
        .addr  = slaveAddr,
        .flags = SMBUS_M_RD,  /* Read operation */
        .len   = rxLen,       /* Pre-calculated length */
        .buf   = rxBuffer     /* Stack-allocated buffer */
    };

    /* Unified transfer interface - single HAL call */
    ret = smbusExecuteTransfer(smbusDev, &msg, 1);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Receive byte failed (Addr:0x%02X, Ret:%d)\n", __func__, slaveAddr, ret);
        return -EIO;
    }

    /* Handle PEC validation if enabled */
    if (pecEn) {
        /* PEC validation sequence: CRC8(Addr(R) + Data) */
        const U8 pecInput[2] = {
            (U8)((slaveAddr << 1) | SMBUS_I2C_READ_MODE),  /* Slave Address + Read Bit(1) */
            rxBuffer[0]                                     /* Received Data Byte */
        };

        /* Calculate and validate PEC */
        const U8 expectedPec = smbusPecPktConstruct(slaveAddr, false, pecInput, 2);
        if (expectedPec != rxBuffer[1]) {
            LOGE("%s: PEC mismatch (Addr:0x%02X, Calc:0x%02X, Recv:0x%02X)\n",
                 __func__, slaveAddr, expectedPec, rxBuffer[1]);
            return -EIO;
        }
    }

    /* Store received data in output parameter */
    param->param.byte.data = rxBuffer[0];

    return EXIT_SUCCESS;
}

/**
 * @brief Write Byte protocol implementation (SMBus 3.1) - Optimized Version
 * @details Writes command code followed by data byte to the slave with improved performance.
 * Packet: [Addr|W] + [Command] + [Data] + [PEC(opt)]
 *
 * Optimizations:
 * - Unified i2cTransfer interface instead of i2cWrite
 * - Single stack allocation with pre-calculated length
 * - Reduced conditional branches and improved cache locality
 *
 * @param[in] smbusDev SMBus device structure
 * @param[in] param Protocol parameters containing command and data
 * @param[in] pecEn PEC enable flag
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
static S32 smbusWriteByteProtocol(SmbusDev_s *smbusDev,
                                 SmbusParam_s *param, bool pecEn)
{
    S32 ret;
    const U8 slaveAddr = param->base.slaveAddr & SMBUS_ADDRESS_MASK;
    const U8 cmdCode = param->param.data.cmdCode;
    const U8 dataByte = param->param.data.buffer ? param->param.data.buffer[0] : param->param.byte.data;

    /* Stack-allocated buffer: Command(1) + Data(1) + [PEC(1)] - single allocation */
    U8 txBuffer[3];
    U16 txLen = 2;  /* Default: Command + Data */

    /* Prepare command and data */
    txBuffer[0] = cmdCode;
    txBuffer[1] = dataByte;

    /* Handle PEC calculation and buffer preparation */
    if (pecEn) {
        /* PEC calculation sequence: CRC8(Addr(W) + Command + Data) */
        const U8 pecInput[3] = {
            (U8)(slaveAddr << 1),  /* Slave Address + Write Bit(0) */
            cmdCode,               /* Command Code */
            dataByte               /* Data Byte */
        };

        /* Append PEC to buffer */
        txBuffer[2] = smbusPecPktConstruct(slaveAddr, true, pecInput, 3);
        txLen = 3;  /* Update length to include PEC */
    }

    /* Single I2C message structure - C99 compound literal */
    SmbusMsg_s msg = {
        .addr  = slaveAddr,
        .flags = 0,        /* Write operation */
        .len   = txLen,    /* Pre-calculated length */
        .buf   = txBuffer  /* Stack-allocated buffer */
    };

    /* Unified transfer interface - single HAL call */
    ret = smbusExecuteTransfer(smbusDev, &msg, 1);

    /* Error handling with consistent logging */
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Write byte failed (Addr:0x%02X, Cmd:0x%02X, Ret:%d)\n",
             __func__, slaveAddr, cmdCode, ret);
        return -EIO;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Read Byte protocol implementation (SMBus 3.1) - Optimized Version
 * @details Writes command code, then reads data byte from the slave with improved performance.
 * Packet: [Addr|W][Cmd] + [Addr|R][Data] + [PEC(opt)]
 *
 * @param[in] smbusDev SMBus device structure
 * @param[in,out] param Protocol parameters containing command, returns data
 * @param[in] pecEn PEC enable flag
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
static S32 smbusReadByteProtocol(SmbusDev_s *smbusDev,
                                SmbusParam_s *param, bool pecEn)
{
    S32 ret;
    const U8 slaveAddr = param->base.slaveAddr & SMBUS_ADDRESS_MASK;
    const U8 cmdCode = param->param.data.cmdCode;

    /* Validate buffer pointer */
    if (param->param.data.buffer == NULL) {
        return -EINVAL;
    }

    /* Stack-allocated buffers and message structures */
    U8 rxBuffer[2];  /* Data(1) + [PEC(1)] */
    const U16 rxLen = pecEn ? 2 : 1;

    /* Create message array for atomic write+read operation */
    SmbusMsg_s msgs[2] = {
        {
            .addr  = slaveAddr,
            .flags = 0,            /* Write operation */
            .len   = 1,            /* Command length */
            .buf   = (U8*)&cmdCode /* Command code */
        },
        {
            .addr  = slaveAddr,
            .flags = SMBUS_M_RD,   /* Read operation */
            .len   = rxLen,        /* Pre-calculated read length */
            .buf   = rxBuffer      /* Stack-allocated receive buffer */
        }
    };

    /* Unified transfer interface - atomic write+read operation */
    ret = smbusExecuteTransfer(smbusDev, msgs, 2);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Read byte failed (Addr:0x%02X, Cmd:0x%02X, Ret:%d)\n",
             __func__, slaveAddr, cmdCode, ret);
        return -EIO;
    }

    /* Handle PEC validation if enabled */
    if (pecEn) {
        /* Optimized PEC calculation: CRC8(Addr(W) + Cmd + Addr(R) + Data) */
        const U8 pecInput[4] = {
            (U8)(slaveAddr << 1),                      /* Address with Write bit */
            cmdCode,                                    /* Command code */
            (U8)((slaveAddr << 1) | SMBUS_I2C_READ_MODE), /* Address with Read bit */
            rxBuffer[0]                                 /* Received data */
        };

        const U8 expectedPec = smbusPecPktConstruct(slaveAddr, false, pecInput, 4);
        if (expectedPec != rxBuffer[1]) {
            LOGE("%s: PEC mismatch (Addr:0x%02X, Cmd:0x%02X, Calc:0x%02X, Recv:0x%02X)\n",
                 __func__, slaveAddr, cmdCode, expectedPec, rxBuffer[1]);
            return -EIO;
        }
    }

    /* Store received data and update read length */
    param->param.data.buffer[0] = rxBuffer[0];
    if (param->param.data.readLen) {
        *param->param.data.readLen = 1;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Write Word protocol implementation (SMBus 3.1) - Optimized Version
 * @details Writes command code followed by 2-byte data (little-endian) to the slave with improved performance.
 * Packet: [Addr|W] + [Command] + [Low Byte] + [High Byte] + [PEC(opt)]
 *
 * Optimizations:
 * - Unified i2cTransfer interface instead of i2cWrite
 * - Single stack allocation with pre-calculated length
 * - Reduced conditional branches and improved cache locality
 * - Enhanced error logging with detailed context
 *
 * @param[in] smbusDev SMBus device structure
 * @param[in] param Protocol parameters containing command and 2-byte data
 * @param[in] pecEn PEC enable flag
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
static S32 smbusWriteWordProtocol(SmbusDev_s *smbusDev,
                                 SmbusParam_s *param, bool pecEn)
{
    S32 ret;
    const U8 slaveAddr = param->base.slaveAddr & SMBUS_ADDRESS_MASK;

    /* Validate buffer pointer */
    if (param->param.data.buffer == NULL) {
        return -EINVAL;
    }

    const U8 cmdCode = param->param.data.cmdCode;
    const U8 lowByte = param->param.data.buffer[0];
    const U8 highByte = param->param.data.buffer[1];

    /* Stack-allocated buffer: Command(1) + Low(1) + High(1) + [PEC(1)] */
    U8 txBuffer[4];
    U16 txLen = 3;  /* Default: Command + Low + High */

    /* Prepare command and word data (Little-endian: Low byte first) */
    txBuffer[0] = cmdCode;
    txBuffer[1] = lowByte;
    txBuffer[2] = highByte;

    /* Handle PEC calculation and buffer preparation */
    if (pecEn) {
        /* PEC calculation sequence: CRC8(Addr(W) + Cmd + Low + High) */
        const U8 pecInput[4] = {
            (U8)(slaveAddr << 1),  /* Address + Write bit */
            cmdCode,               /* Command Code */
            lowByte,               /* Low Byte */
            highByte               /* High Byte */
        };

        /* Append PEC to buffer */
        txBuffer[3] = smbusPecPktConstruct(slaveAddr, true, pecInput, 4);
        txLen = 4;  /* Update length to include PEC */
    }

    /* Single I2C message structure - C99 compound literal */
    SmbusMsg_s msg = {
        .addr  = slaveAddr,
        .flags = 0,        /* Write operation */
        .len   = txLen,    /* Pre-calculated length */
        .buf   = txBuffer  /* Stack-allocated buffer */
    };

    /* Unified transfer interface - single HAL call */
    ret = smbusExecuteTransfer(smbusDev, &msg, 1);

    /* Error handling with enhanced logging */
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Write word failed (Addr:0x%02X, Cmd:0x%02X, Data:0x%04X, Ret:%d)\n",
             __func__, slaveAddr, cmdCode, (U16)((highByte << 8) | lowByte), ret);
        return -EIO;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Read Word protocol implementation (SMBus 3.1) - Optimized Version
 * @details Writes command code, then reads 2-byte data (little-endian) from the slave with improved performance.
 * Packet: [Addr|W][Cmd] + [Addr|R][Low Byte][High Byte] + [PEC(opt)]
 *
 * @param[in] smbusDev SMBus device structure
 * @param[in,out] param Protocol parameters containing command, returns 2-byte data
 * @param[in] pecEn PEC enable flag
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
static S32 smbusReadWordProtocol(SmbusDev_s *smbusDev,
                                SmbusParam_s *param, bool pecEn)
{
    S32 ret;
    const U8 slaveAddr = param->base.slaveAddr & SMBUS_ADDRESS_MASK;
    const U8 cmdCode = param->param.data.cmdCode;

    /* Validate buffer pointer */
    if (param->param.data.buffer == NULL) {
        return -EINVAL;
    }

    /* Stack-allocated buffers and message structures */
    U8 rxBuffer[3];  /* Low(1) + High(1) + [PEC(1)] */
    const U16 rxLen = pecEn ? 3 : 2;

    /* Create message array for atomic write+read operation */
    SmbusMsg_s msgs[2] = {
        {
            .addr  = slaveAddr,
            .flags = 0,            /* Write operation */
            .len   = 1,            /* Command length */
            .buf   = (U8*)&cmdCode /* Command code */
        },
        {
            .addr  = slaveAddr,
            .flags = SMBUS_M_RD,   /* Read operation */
            .len   = rxLen,        /* Pre-calculated read length */
            .buf   = rxBuffer      /* Stack-allocated receive buffer */
        }
    };

    /* Unified transfer interface - atomic write+read operation */
    ret = smbusExecuteTransfer(smbusDev, msgs, 2);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Read word failed (Addr:0x%02X, Cmd:0x%02X, Ret:%d)\n",
             __func__, slaveAddr, cmdCode, ret);
        return -EIO;
    }

    /* Handle PEC validation if enabled */
    if (pecEn) {
        /* Optimized PEC calculation: CRC8(Addr(W) + Cmd + Addr(R) + Low + High) */
        const U8 pecInput[5] = {
            (U8)(slaveAddr << 1),                      /* Address with Write bit */
            cmdCode,                                    /* Command code */
            (U8)((slaveAddr << 1) | SMBUS_I2C_READ_MODE), /* Address with Read bit */
            rxBuffer[0],                                /* Low byte */
            rxBuffer[1]                                 /* High byte */
        };

        const U8 expectedPec = smbusPecPktConstruct(slaveAddr, false, pecInput, 5);
        if (expectedPec != rxBuffer[2]) {
            LOGE("%s: PEC mismatch (Addr:0x%02X, Cmd:0x%02X, Calc:0x%02X, Recv:0x%02X)\n",
                 __func__, slaveAddr, cmdCode, expectedPec, rxBuffer[2]);
            return -EIO;
        }
    }

    /* Store received data (Little-endian: Low byte first) and update read length */
    param->param.data.buffer[0] = rxBuffer[0]; /* Low byte */
    param->param.data.buffer[1] = rxBuffer[1]; /* High byte */
    if (param->param.data.readLen) {
        *param->param.data.readLen = 2;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Block Write protocol implementation (SMBus 3.1) - Optimized Version
 * @details Writes command code, byte count, and data block to the slave with improved performance.
 * Sequence: [S] [Addr|W] [Cmd] [Count] [Data...] [PEC] [P]
 *
 * Optimizations:
 * - Stack allocation instead of dynamic malloc/free (no memory fragmentation)
 * - Unified i2cTransfer interface instead of i2cWrite
 * - Single memcpy operation with pre-calculated lengths
 * - Optimized PEC calculation covering complete transaction
 * - Enhanced error logging with detailed context information
 *
 * @param[in] smbusDev SMBus device structure
 * @param[in] param Protocol parameters containing command and block data
 * @param[in] pecEn PEC enable flag
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
static S32 smbusBlockWriteProtocol(SmbusDev_s *smbusDev,
                                  SmbusParam_s *param, bool pecEn)
{
    S32 ret;
    const U8 slaveAddr = param->base.slaveAddr & SMBUS_ADDRESS_MASK;

    /* Enhanced parameter validation */
    if (param->param.block.buffer == NULL || param->param.block.length == 0) {
        LOGE("%s: Invalid parameters for Block Write (buffer:%p, length:%d)\n",
             __func__, param->param.block.buffer, param->param.block.length);
        return -EINVAL;
    }

    if (param->param.block.length > SMBUS_BLOCK_MAXLEN) {
        LOGE("%s: Block length %d exceeds maximum %d\n",
             __func__, param->param.block.length, SMBUS_BLOCK_MAXLEN);
        return -EINVAL;
    }

    const U8 cmdCode = param->param.block.cmdCode;
    const U8 byteCount = (U8)param->param.block.length;

    /*
     * Stack-allocated Buffer Management (Optimized Performance)
     * Buffer: Cmd(1) + Count(1) + Data(32) + [PEC(1)] = 35 Max
     * Total stack usage: ~35 bytes (vs original malloc/free + memcpy overhead)
     */
    U8 txBuffer[SMBUS_BLOCK_MAXLEN + 3];  /* Maximum required space */
    U16 txLen = 2 + byteCount;  /* Default: Cmd + Count + Data */

    /* --- 1. Prepare Write Message --- */
    txBuffer[0] = cmdCode;  /* Command code */
    txBuffer[1] = byteCount;  /* Byte count */

    /* Single memcpy operation for data block */
    memcpy(&txBuffer[2], param->param.block.buffer, byteCount);

    /* --- 2. Handle PEC Calculation and Buffer Preparation --- */
    if (pecEn) {
        /* PEC calculation sequence: CRC8(Addr(W) + Cmd + Count + Data) */
        U8 pecInput[2 + SMBUS_BLOCK_MAXLEN] = {
            cmdCode,   /* Command code */
            byteCount  /* Byte count */
        };

        /* Copy data to PEC input buffer for continuous calculation */
        memcpy(&pecInput[2], param->param.block.buffer, byteCount);

        /* Append PEC to transmit buffer */
        txBuffer[txLen] = smbusPecPktConstruct(slaveAddr, true, pecInput, 2 + byteCount);
        txLen++;  /* Update length to include PEC */
    }

    /* --- 3. Single I2C Message Structure --- */
    SmbusMsg_s msg = {
        .addr  = slaveAddr,
        .flags = 0,        /* Write operation */
        .len   = txLen,    /* Pre-calculated length */
        .buf   = txBuffer  /* Stack-allocated buffer */
    };

    /* --- 4. Unified Transfer Interface (Single HAL Call) --- */
    ret = smbusExecuteTransfer(smbusDev, &msg, 1);

    /* --- 5. Enhanced Error Handling --- */
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Block write failed (Addr:0x%02X, Cmd:0x%02X, Count:%d, Ret:%d)\n",
             __func__, slaveAddr, cmdCode, byteCount, ret);
        return -EIO;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Block Read protocol implementation
 * @details Implements SMBus Block Read protocol with optional PEC support.
 *          Writes command code, then reads byte count and data block from the slave.
 * @param[in] halOps HAL operations structure
 * @param[in] smbusDev SMBus device structure
 * @param[in,out] param Protocol parameters containing command, returns block data
 * @param[in] pecEn PEC enable flag
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
/**
 * @brief Block Read protocol implementation (SMBus 3.1) - Optimized Version
 * @details Writes command code, then reads byte count and data block from the slave with improved performance.
 * Sequence: [S] [Addr|W] [Cmd] [Sr] [Addr|R] [Count] [Data...] [PEC] [P]
 *
 * @param[in] smbusDev SMBus device structure
 * @param[in,out] param Protocol parameters containing command, returns block data
 * @param[in] pecEn PEC enable flag
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
static S32 smbusBlockReadProtocol(SmbusDev_s *smbusDev,
                                 SmbusParam_s *param, bool pecEn)
{
    S32 ret;
    const U8 slaveAddr = param->base.slaveAddr & SMBUS_ADDRESS_MASK;

    /* Enhanced parameter validation */
    if (param->param.block.buffer == NULL) {
        LOGE("%s: Invalid buffer pointer for Block Read\n", __func__);
        return -EINVAL;
    }

    const U8 cmdCode = param->param.block.cmdCode;

    /*
     * Stack-allocated Buffer Management (Optimized Performance)
     * Strategy: Request maximum possible data length and parse dynamically
     * Rx Buffer: Count(1) + Data(32) + [PEC(1)] = 34 Max
     * Total stack usage: ~34 bytes (vs original malloc/free overhead)
     */
    U8 rxBuffer[SMBUS_BLOCK_MAXLEN + 2];  /* Maximum required space */

    /* --- 1. Prepare Atomic I2C Messages --- */
    /*
     * Message 0: Write Command (Single byte)
     * Message 1: Read Block (Count + Data + [PEC])
     * Strategy: Request maximum length since response size is unknown initially
     */
    const U16 maxRxLen = pecEn ? (SMBUS_BLOCK_MAXLEN + 2) : (SMBUS_BLOCK_MAXLEN + 1);

    SmbusMsg_s msgs[2] = {
        {
            .addr  = slaveAddr,
            .flags = 0,             /* Write operation */
            .len   = 1,             /* Command length */
            .buf   = (U8*)&cmdCode /* Command code */
        },
        {
            .addr  = slaveAddr,
            .flags = SMBUS_M_RD,    /* Read operation (corrected macro) */
            .len   = maxRxLen,      /* Request maximum possible length */
            .buf   = rxBuffer       /* Stack-allocated receive buffer */
        }
    };

    /* --- 2. Execute Atomic Transfer (Single HAL Call) --- */
    ret = smbusExecuteTransfer(smbusDev, msgs, 2);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Atomic Block Read failed (Addr:0x%02X, Cmd:0x%02X, Ret:%d)\n",
             __func__, slaveAddr, cmdCode, ret);
        return ret;
    }

    /* --- 3. Parse Response Byte Count --- */
    const U8 byteCount = rxBuffer[0];  /* First byte is the block length */

    /* Validate received byte count */
    if (byteCount == 0 || byteCount > SMBUS_BLOCK_MAXLEN) {
        LOGE("%s: Invalid response byte count %d (Addr:0x%02X, Cmd:0x%02X, Max:%d)\n",
             __func__, byteCount, slaveAddr, cmdCode, SMBUS_BLOCK_MAXLEN);
        return -EPROTO;
    }

    /* --- 4. Handle Buffer Size Limitations --- */
    /* Limit actual read count to user buffer capacity */
    const U8 actualReadCount = (byteCount < param->param.block.length) ?
                               byteCount : (U8)param->param.block.length;

    /* --- 5. Validate PEC (if enabled) --- */
    if (pecEn) {
        /*
         * PEC Calculation covers the ENTIRE transaction for SMBus 3.1 compliance:
         * Addr(W) + Cmd + Addr(R) + Count + Data...
         */

        /* Construct continuous data stream for PEC calculation */
        /* Max Size: Cmd(1) + Count(1) + Data(32) = 34 bytes */
        U8 pecInput[2 + SMBUS_BLOCK_MAXLEN];
        pecInput[0] = cmdCode;    /* Command code */
        pecInput[1] = byteCount;  /* Response count */

        /* Copy received data to PEC input buffer */
        memcpy(&pecInput[2], &rxBuffer[1], actualReadCount);

        /* Calculate PEC over complete transaction: Addr(W) + Cmd + Addr(R) + Count + Data */
        const U8 expectedPec = smbusPecPktConstruct(slaveAddr, false, pecInput, 2 + actualReadCount);

        /* PEC byte is located after the data block: rxBuffer[1 + actualReadCount] */
        const U8 receivedPec = rxBuffer[1 + actualReadCount];

        if (expectedPec != receivedPec) {
            LOGE("%s: PEC mismatch (Addr:0x%02X, Cmd:0x%02X, Count:%d, Calc:0x%02X, Recv:0x%02X)\n",
                 __func__, slaveAddr, cmdCode, byteCount, expectedPec, receivedPec);
            return -EIO;
        }
    }

    /* --- 6. Copy Received Data to User Buffer --- */
    memcpy(param->param.block.buffer, &rxBuffer[1], actualReadCount);

    /* Update actual read length indicator */
    if (param->param.block.readLen) {
        *param->param.block.readLen = actualReadCount;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Process Call protocol implementation
 * @details Implements SMBus Process Call protocol with optional PEC support.
 *          Writes command code and 2-byte data, then reads 2-byte response.
 * @param[in] halOps HAL operations structure
 * @param[in] smbusDev SMBus device structure
 * @param[in,out] param Protocol parameters containing command and data
 * @param[in] pecEn PEC enable flag
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
/**
 * @brief Process Call protocol implementation (SMBus 3.1) - Optimized Version
 * @details Writes command and 2-byte data, then reads 2-byte response with improved performance.
 * Sequence: [S] [Addr|W] [Cmd] [DataLow] [DataHigh] [Sr] [Addr|R] [RespLow] [RespHigh] [PEC] [P]
 *
 * @param[in]     smbusDev SMBus device handle
 * @param[in,out] param    Protocol parameters with 2-byte write/read buffers
 * @param[in]     pecEn    PEC enable flag
 * @return EXIT_SUCCESS or negative error code
 */
static S32 smbusProcessCallProtocol(SmbusDev_s *smbusDev, SmbusParam_s *param, bool pecEn)
{
    S32 ret;
    const U8 slaveAddr = param->base.slaveAddr & SMBUS_ADDRESS_MASK;

    /* Enhanced Parameter Validation - Process Call requires exactly 2 bytes */
    if (param->param.processCall.wbuffer == NULL ||
        param->param.processCall.rbuffer == NULL ||
        param->param.processCall.wlen < 2) {
        LOGE("%s: Invalid parameters for Process Call (wlen:%d, required:2)\n",
             __func__, param->param.processCall.wlen);
        return -EINVAL;
    }

    const U8 cmdCode = param->param.processCall.cmdCode;
    const U8 lowByte = param->param.processCall.wbuffer[0];   /* Little-endian: low byte first */
    const U8 highByte = param->param.processCall.wbuffer[1];  /* High byte second */

    /*
     * Stack-allocated Buffer Management (Optimized Performance)
     * Tx: Cmd(1) + DataLow(1) + DataHigh(1) + [PEC(1)] = 4 Max
     * Rx: DataLow(1) + DataHigh(1) + [PEC(1)] = 3 Max
     * Total stack usage: ~7 bytes (vs original malloc/free + memcpy overhead)
     */
    U8 txBuffer[4];  /* Command + 2-byte data + optional PEC */
    U8 rxBuffer[3];  /* 2-byte response + optional PEC */

    /* --- 1. Prepare Write Message (Little-endian format) --- */
    txBuffer[0] = cmdCode;      /* Command code */
    txBuffer[1] = lowByte;     /* Data low byte */
    txBuffer[2] = highByte;    /* Data high byte */
    U16 writeLen = 3;           /* Default: Cmd + Low + High */

    /* --- 2. Handle PEC for Write Phase --- */
    if (pecEn) {
        /* PEC calculation: CRC8(Addr(W) + Cmd + Low + High) */
        const U8 pecInput[3] = {
            cmdCode,   /* Command code */
            lowByte,   /* Data low byte */
            highByte   /* Data high byte */
        };

        txBuffer[3] = smbusPecPktConstruct(slaveAddr, true, pecInput, 3);
        writeLen = 4;  /* Include PEC in write length */
    }

    /* --- 3. Prepare Atomic I2C Messages --- */
    const U16 rxLen = pecEn ? 3 : 2;  /* Response data + optional PEC */

    SmbusMsg_s msgs[2] = {
        {
            .addr  = slaveAddr,
            .flags = 0,              /* Write operation */
            .len   = writeLen,       /* Pre-calculated write length */
            .buf   = txBuffer
        },
        {
            .addr  = slaveAddr,
            .flags = SMBUS_M_RD,     /* Read operation (corrected macro) */
            .len   = rxLen,          /* Pre-calculated read length */
            .buf   = rxBuffer
        }
    };

    /* --- 4. Execute Atomic Transfer (Single HAL Call) --- */
    ret = smbusExecuteTransfer(smbusDev, msgs, 2);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Atomic Process Call failed (Addr:0x%02X, Cmd:0x%02X, Data:0x%04X, Ret:%d)\n",
             __func__, slaveAddr, cmdCode, (U16)((highByte << 8) | lowByte), ret);
        return ret;
    }

    /* --- 5. Validate PEC (if enabled) --- */
    if (pecEn) {
        /*
         * PEC Calculation covers the ENTIRE transaction for SMBus 3.1 compliance:
         * Addr(W) + Cmd + DataLow + DataHigh + Addr(R) + RespLow + RespHigh
         */

        /* Construct continuous data stream for PEC calculation */
        const U8 pecInput[5] = {
            cmdCode,        /* Command code */
            lowByte,        /* Transmit low byte */
            highByte,       /* Transmit high byte */
            rxBuffer[0],    /* Response low byte */
            rxBuffer[1]     /* Response high byte */
        };

        /* Calculate PEC over complete transaction: Addr(W) + Data + Addr(R) + Response */
        const U8 expectedPec = smbusPecPktConstruct(slaveAddr, false, pecInput, 5);

        /* PEC byte is located after the response data: rxBuffer[2] */
        const U8 receivedPec = rxBuffer[2];

        if (expectedPec != receivedPec) {
            LOGE("%s: PEC mismatch (Addr:0x%02X, Cmd:0x%02X, Tx:0x%04X, Rx:0x%04X, Calc:0x%02X, Recv:0x%02X)\n",
                 __func__, slaveAddr, cmdCode, (U16)((highByte << 8) | lowByte),
                 (U16)((rxBuffer[1] << 8) | rxBuffer[0]), expectedPec, receivedPec);
            return -EIO;
        }
    }

    /* --- 6. Copy Response Data to User Buffer (Little-endian) --- */
    param->param.processCall.rbuffer[0] = rxBuffer[0];  /* Response low byte */
    param->param.processCall.rbuffer[1] = rxBuffer[1];  /* Response high byte */

    /* Update read length indicator */
    if (param->param.processCall.readLen) {
        *param->param.processCall.readLen = 2;  /* Process Call always returns 2 bytes */
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Block Process Call protocol implementation (SMBus 3.1) - Optimized Version
 * @details Writes a command and data block, then reads a data block response with improved performance.
 * Sequence: [S] [Addr|W] [Cmd] [CntW] [DataW...] [Sr] [Addr|R] [CntR] [DataR...] [PEC] [P]
 * @param[in]     smbusDev SMBus device handle
 * @param[in,out] param    Protocol parameters with write/read buffers
 * @param[in]     pecEn    PEC enable flag
 * @return EXIT_SUCCESS or negative error code
 */
static S32 smbusBlockProcessCallProtocol(SmbusDev_s *smbusDev, SmbusParam_s *param, bool pecEn)
{
    S32 ret;
    const U8 slaveAddr = param->base.slaveAddr & SMBUS_ADDRESS_MASK;

    /* Enhanced Parameter Validation */
    if (param->param.processCall.wbuffer == NULL ||
        param->param.processCall.rbuffer == NULL ||
        param->param.processCall.wlen == 0 ||
        param->param.processCall.wlen > SMBUS_BLOCK_MAXLEN) {
        LOGE("%s: Invalid parameters for Block Process Call (wlen:%d, max:%d)\n",
             __func__, param->param.processCall.wlen, SMBUS_BLOCK_MAXLEN);
        return -EINVAL;
    }

    const U8 cmdCode = param->param.processCall.cmdCode;
    const U8 wLen = (U8)param->param.processCall.wlen;

    /*
     * Stack-allocated Buffer Management (Optimized Performance)
     * Tx: Cmd(1) + Count(1) + Data(32) = 34 Max
     * Rx: Count(1) + Data(32) + PEC(1) = 34 Max
     * Total stack usage: ~68 bytes (vs original malloc/free overhead)
     */
    U8 txBuffer[SMBUS_BLOCK_MAXLEN + 2];
    U8 rxBuffer[SMBUS_BLOCK_MAXLEN + 2];

    /* --- 1. Prepare Write Message --- */
    txBuffer[0] = cmdCode;                              /* Command code */
    txBuffer[1] = wLen;                                 /* Write byte count */
    memcpy(&txBuffer[2], param->param.processCall.wbuffer, wLen);  /* Write data */

    /* --- 2. Prepare Atomic I2C Messages --- */
    /*
     * Message 0: Write Block (Cmd + Count + Data)
     * Message 1: Read Block  (Count + Data + PEC)
     * Note: Request maximum length since response size is unknown at this point
     */
    const U16 rxReqLen = pecEn ? (SMBUS_BLOCK_MAXLEN + 2) : (SMBUS_BLOCK_MAXLEN + 1);

    SmbusMsg_s msgs[2] = {
        {
            .addr  = slaveAddr,
            .flags = 0,                         /* Write operation */
            .len   = (U16)(2 + wLen),           /* Cmd + Count + Data length */
            .buf   = txBuffer
        },
        {
            .addr  = slaveAddr,
            .flags = SMBUS_M_RD,                /* Read operation (corrected macro) */
            .len   = rxReqLen,                  /* Request max length for safety */
            .buf   = rxBuffer
        }
    };

    /* --- 3. Execute Atomic Transfer (Single HAL Call) --- */
    ret = smbusExecuteTransfer(smbusDev, msgs, 2);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Atomic Block Process Call failed (Addr:0x%02X, Cmd:0x%02X, Wlen:%d, Ret:%d)\n",
             __func__, slaveAddr, cmdCode, wLen, ret);
        return ret;
    }

    /* --- 4. Parse Response --- */
    /* First byte of RX buffer is the response byte count */
    const U8 rLen = rxBuffer[0];

    if (rLen == 0 || rLen > SMBUS_BLOCK_MAXLEN) {
        LOGE("%s: Invalid response block length %d (Addr:0x%02X, Cmd:0x%02X, Max:%d)\n",
             __func__, rLen, slaveAddr, cmdCode, SMBUS_BLOCK_MAXLEN);
        return -EPROTO;
    }

    /* --- 5. Validate PEC (if enabled) --- */
    if (pecEn) {
        /*
         * PEC Calculation covers the ENTIRE transaction for SMBus 3.1 compliance:
         * Addr(W) + Cmd + CntW + DataW... + Addr(R) + CntR + DataR...
         */

        /* Construct temporary buffer for continuous CRC calculation */
        /* Max Size: AddrW(1)+Cmd(1)+CntW(1)+DataW(32) + AddrR(1)+CntR(1)+DataR(32) = 70 bytes */
        U8 pecCalcBuf[72];  /* Slightly larger for safety */
        U32 pIdx = 0;

        /* Write Phase Data Stream */
        pecCalcBuf[pIdx++] = (U8)(slaveAddr << 1);        /* Addr + Write bit */
        pecCalcBuf[pIdx++] = cmdCode;                     /* Command code */
        pecCalcBuf[pIdx++] = wLen;                        /* Write count */
        memcpy(&pecCalcBuf[pIdx], param->param.processCall.wbuffer, wLen);
        pIdx += wLen;

        /* Read Phase Data Stream */
        pecCalcBuf[pIdx++] = (U8)((slaveAddr << 1) | SMBUS_I2C_READ_MODE); /* Addr + Read bit */
        pecCalcBuf[pIdx++] = rLen;                        /* Read count */
        memcpy(&pecCalcBuf[pIdx], &rxBuffer[1], rLen);    /* Read data */
        pIdx += rLen;

        /* Calculate Expected PEC over complete transaction */
        /* Pass 'false' to treat buffer as raw data stream (no address prefix) */
        const U8 expectedPec = smbusPecPktConstruct(slaveAddr, false, pecCalcBuf, pIdx);

        /* PEC byte is located after the data block: rxBuffer[1 + rLen] */
        const U8 receivedPec = rxBuffer[1 + rLen];

        if (expectedPec != receivedPec) {
            LOGE("%s: PEC mismatch (Addr:0x%02X, Cmd:0x%02X, Wlen:%d, Rlen:%d, Calc:0x%02X, Recv:0x%02X)\n",
                 __func__, slaveAddr, cmdCode, wLen, rLen, expectedPec, receivedPec);
            return -EIO;
        }
    }

    /* --- 6. Safe Data Copy to User Buffer --- */
    /* Handle user buffer size limitations safely */
    const U8 userBufLen = (U8)param->param.processCall.rlen;
    const U8 copyLen = (rLen < userBufLen) ? rLen : userBufLen;

    memcpy(param->param.processCall.rbuffer, &rxBuffer[1], copyLen);

    /* Always return actual received length for user information */
    if (param->param.processCall.readLen) {
        *param->param.processCall.readLen = rLen;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Find ARP device by UDID
 * @details Searches the ARP master's device list for a device with the specified
 *          UDID. This function performs a linear search through the device list.
 * @param[in] master Pointer to the ARP master structure
 * @param[in] udid Pointer to the UDID structure to search for
 * @return Pointer to device node if found, NULL otherwise
 *
 * @note Uses smbusUdidCompare for UDID comparison
 * @note Performs linear search through device list
 * @warning This function is not thread-safe
 * @warning Caller must ensure pointers are valid
 *
 */
static SmbusArpDeviceNode_s* ArpDevFind(SmbusArpMaster_s *master, const SmbusUdid_s *udid)
{
    SmbusArpDeviceNode_s *current = NULL;

    if (master == NULL || udid == NULL) {
        return NULL;
    }

    current = master->deviceList;
    while (current != NULL) {
        if (smbusUdidCompare(&current->udid, udid)) {
            return current;
        }
        current = current->next;
    }

    return NULL;
}

/* ======================================================================== */
/*                    Master Protocol Operation Functions                   */
/* ======================================================================== */

/**
 * [MASTER_API] Master API layer - high-level Master mode operations
 */
S32 smbusMasterOperation(DevList_e devId, SmbusParam_s *param)
{
    if (param == NULL) {
        LOGE("%s(): param == NULL\n", __func__);
        return -EINVAL;
    }

    /* Check driver lock and validate */
    SmbusDrvData_s *pDrvData = NULL;
    S32 ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void **)&pDrvData) != EXIT_SUCCESS) {
        LOGE("%s(): get driver data failed\n", __func__);
        devUnlockByDriver(devId);
        return -EINVAL;
    }

    if (pDrvData == NULL) {
        LOGE("%s(): invalid driver data\n", __func__);
        devUnlockByDriver(devId);
        return -EINVAL;
    }

    bool pecEn = (param->base.enablePec != 0);

    /* Get HAL operations */
    // HAL operations already registered in device structure
    if (pDrvData->pSmbusDev.halOps == NULL) {
        LOGE("%s(): HAL operations not available\n", __func__);
        ret = -ENOTSUP;
        goto unlock;
    }

    /* CRITICAL FIX: Use the driver's main device structure directly */
    /* This ensures ISR can access the same device structure - consistent with Block Read fix */
    pDrvData->pSmbusDev.regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;
    pDrvData->pSmbusDev.addrMode = pDrvData->sbrCfg.addrMode;
    pDrvData->pSmbusDev.mode = pDrvData->sbrCfg.masterMode ? DW_SMBUS_MODE_MASTER : DW_SMBUS_MODE_SLAVE;

    /* Set transfer timeout */
    U32 timeout = param->base.timeout;
    if (timeout == 0) timeout = SMBUS_DEFAULT_TIMEOUT_MS;

    /* Dispatch to appropriate protocol handler */
    switch (param->protocol) {
    case SMBUS_PROTOCOL_QUICK_CMD:
        ret = smbusQuickCmdProtocol(&pDrvData->pSmbusDev, param, pecEn);
        break;

    case SMBUS_PROTOCOL_SEND_BYTE:
        ret = smbusSendByteProtocol(&pDrvData->pSmbusDev, param, pecEn);
        break;

    case SMBUS_PROTOCOL_RECEIVE_BYTE:
        ret = smbusReceiveByteProtocol(&pDrvData->pSmbusDev, param, pecEn);
        break;

    case SMBUS_PROTOCOL_WRITE_BYTE:
        ret = smbusWriteByteProtocol(&pDrvData->pSmbusDev, param, pecEn);
        break;

    case SMBUS_PROTOCOL_READ_BYTE:
        ret = smbusReadByteProtocol(&pDrvData->pSmbusDev, param, pecEn);
        break;

    case SMBUS_PROTOCOL_WRITE_WORD:
        ret = smbusWriteWordProtocol(&pDrvData->pSmbusDev, param, pecEn);
        break;

    case SMBUS_PROTOCOL_READ_WORD:
        ret = smbusReadWordProtocol(&pDrvData->pSmbusDev, param, pecEn);
        break;

    case SMBUS_PROTOCOL_BLOCK_WRITE:
        ret = smbusBlockWriteProtocol(&pDrvData->pSmbusDev, param, pecEn);
        break;

    case SMBUS_PROTOCOL_BLOCK_READ:
        ret = smbusBlockReadProtocol(&pDrvData->pSmbusDev, param, pecEn);
        break;

    case SMBUS_PROTOCOL_PROCESS_CALL:
        ret = smbusProcessCallProtocol(&pDrvData->pSmbusDev, param, pecEn);
        break;

    case SMBUS_PROTOCOL_BLOCK_PROCESS_CALL:
        ret = smbusBlockProcessCallProtocol(&pDrvData->pSmbusDev, param, pecEn);
        break;

    default:
        LOGE("%s(): unsupported protocol %d\n", __func__, param->protocol);
        ret = -ENOTSUP;
        break;
    }

unlock:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * [MASTER_API] Master API layer - high-level Master mode operations
 */
S32 smbusHostNotifyCmd(DevList_e devId, SmbusHostNotifyData_s *data)
{
    S32 ret = 0;
    volatile SmbusRegMap_s *regBase = NULL;

    /* Step 0: Parameter validation */
    if (data == NULL) {
        ret = -EINVAL;
        return ret;
    }

    /* Check driver lock and validate */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ///< Get register base
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        ret = -EINVAL;
        devUnlockByDriver(devId);
        goto exit;
    }

    /* Step 4: Get driver data and check HAL operations */
    SmbusDrvData_s *pDrvData = NULL;
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS || pDrvData == NULL) {
        LOGE("%s(): Failed to get driver data\n", __func__);
        ret = -EINVAL;
        goto unlock;
    }

    if (pDrvData->pSmbusDev.halOps == NULL || pDrvData->pSmbusDev.halOps->hostNotifyCore == NULL) {
        LOGE("%s(): HAL operations not available\n", __func__);
        ret = -ENOTSUP;
        goto unlock;
    }

    /* Step 5: Call HAL layer Host Notify core function */
    ret = pDrvData->pSmbusDev.halOps->hostNotifyCore(regBase, data);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): HAL Host Notify failed, ret=%d\n", __func__, ret);
        goto exit;
    }

    LOGD("%s(): Host Notify command executed successfully\n", __func__);
unlock:
    /* Step 6: Unlock device */
    devUnlockByDriver(devId);

exit:
    return ret;
}

/* ======================================================================== */
/*                    Master ARP Status Functions                              */
/* ======================================================================== */

/**
 * @brief Get SAR address resolved status
 * @details Retrieves the address resolved status for the specified SAR
 *          (Secondary Address Register) in the SMBus controller.
 *          This function checks whether the SAR address has been resolved
 *          through the ARP process and is ready for use.
 * @param[in] devId SMBus device identifier
 * @param[in] sarNum SAR register number (1 or 2)
 * @return 1 if address is resolved, 0 if not resolved, negative error code on failure
 *
 * @note Reads IC_STATUS register to get address resolved status
 * @note sarAddrResolved is located in IC_STATUS[18] for SAR1 and IC_STATUS[22] for SAR2
 * @note Performs device locking for thread safety
 * @warning Only valid in Slave mode with ARP enabled
 * @warning Caller must ensure valid sarNum value (1 or 2)
 *
 * [MASTER_API] Master API layer - ARP status management
 */
S32 smbusArpAddrResolvedGetStatus(DevList_e devId, U32 sarNum)
{
    volatile SmbusRegMap_s *regBase = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Check driver lock and validate */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ///< Get register base
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        devUnlockByDriver(devId);
        return -EINVAL;
    }

    /* Read address resolved status from IC_STATUS register */
    if (sarNum == 1) {
        ret = regBase->icStatus.fields.smbusSlaveAddrResolved;
    } else if (sarNum == 2) {
        /* Note: SAR2 address resolved status would be in a different bit position
         * This would need to be confirmed from the hardware documentation
         * For now, return not implemented for SAR2
         */
        ret = -ENOTSUP;  /* Not implemented for SAR2 */
    } else {
        ret = -EINVAL;
        goto unlock;
    }

unlock:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief Device address assignment wrapper function
 * @details Wrapper function that performs device address assignment using
 *          the HAL layer core function. This function handles driver validation,
 *          device locking, and error handling around the core address assignment LOGEc.
 * @param[in] devId SMBus device identifier
 * @param[in] assignAddr Address to assign to the device
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Calls HAL layer smbusDevAddrAssignCore function
 * @note Performs complete device validation and locking
 * @note Suitable for Master mode operations
 * @warning This function performs blocking operations
 * @warning Address assignment may take significant time to complete
 *
 * [MASTER_API] Master API layer - device management
 */
S32 smbusDevAddrAssign(DevList_e devId, U8 assignAddr)
{
    volatile SmbusRegMap_s *regBase = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Check driver lock and validate */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ///< Get register base
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        devUnlockByDriver(devId);
        return -EINVAL;
    }

    /* Get driver data and HAL operations */
    SmbusDrvData_s *pDrvData = NULL;
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS || pDrvData == NULL) {
        LOGE("%s(): Failed to get driver data\n", __func__);
        ret = -EINVAL;
        goto unlock;
    }

    if (pDrvData->pSmbusDev.halOps == NULL || pDrvData->pSmbusDev.halOps->devAddrAssignCore == NULL) {
        ret = -ENOTSUP;
        goto unlock;
    }

    /* Call core layer function */
    ret = pDrvData->pSmbusDev.halOps->devAddrAssignCore(regBase, assignAddr);

unlock:
    devUnlockByDriver(devId);
    return ret;
}

/* ======================================================================== */
/*                    Master ARP Protocol Functions                          */
/* ======================================================================== */

/**
 * @brief Send general reset command to all devices on bus
 * @details Sends a general reset command to the ARP default address (0x61) to reset
 *          all ARP-capable devices on the SMBus according to SMBus 3.1 specification.
 *          This command causes all devices to return to their default address (0x61)
 *          and clear any assigned addresses. This is typically used to initialize
 *          the ARP process or recover from error conditions.
 * @param[in] devId SMBus device identifier for the master controller
 * @return EXIT_SUCCESS on successful command transmission, negative error code on failure:
 *         -EINVAL: Invalid parameters or register base
 *         -ENODEV: Device not found or not initialized
 *         -EBUSY: Device locked by another operation
 *         -ENOTSUP: HAL operations not available
 *         -ETIMEDOUT: TX ready timeout occurred
 *         -EIO: Command transmission failed or abort detected
 *
 * @note Sends command to ARP default address (0x61)
 * @note Command format: [ARP_ADDR][GENERAL_RESET_CMD][PEC]
 * @note Affects all ARP-capable devices on the bus
 * @note Devices will return to default address (0x61) after reset
 * @note Uses PEC (Packet Error Code) for data integrity verification
 * @note Performs TX abort checking and error recovery
 * @note Only valid in Master mode
 * @warning This will reset ALL ARP-capable devices on the bus
 * @warning All assigned addresses will be lost and must be re-assigned
 * @warning Failed transmission may leave bus in undefined state
 * @warning Should be used with caution as it affects all devices
 *
 */
S32 smbusArpResetDeviceGeneral(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    volatile SmbusRegMap_s *regBase = NULL;
    U32 defaultAddr = SMBUS_ARP_ADDR;
    U8 resetCmd = SMBUS_CMD_GENERAL_RESET_DEVICE;
    U8 pecData = 0;
    SmbusIcTxAbrtSourceReg_u abrtSource;

    /* Check driver lock and validate */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    ///< Get register base
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        devUnlockByDriver(devId);
        ret = -ENODEV;
        goto exit;
    }

    LOGD("%s: Sending general reset command\n", __func__);

    ///< Get driver data and HAL operations for TX checks
    SmbusDrvData_s *pDrvData = NULL;
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS || pDrvData == NULL) {
        LOGE("%s(): Failed to get driver data\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (pDrvData->pSmbusDev.halOps == NULL || pDrvData->pSmbusDev.halOps->checkTxReady == NULL) {
        LOGE("%s: HAL operations not available\n", __func__);
        ret = -ENOTSUP;
        goto unlock;
    }

    ///< Step 1: Set target address to ARP default address (Write operation)
    regBase->icTar.fields.icTar = defaultAddr;

    ///< Step 2: Send Reset Device command byte (without STOP)
    ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: TX ready timeout for reset command\n", __func__);
        ret = -ETIMEDOUT;
        goto unlock;
    }
    regBase->icDataCmd.fields.dat = resetCmd;  ///<<不设置STOP位

    ///< Step 3: Calculate PEC
    pecData = smbusPecPktConstruct(defaultAddr, true, &resetCmd, 1);  ///<<false表示Write

    ///< Step 4: Send PEC byte with STOP bit
    ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: TX ready timeout for PEC\n", __func__);
        ret = -ETIMEDOUT;
        goto unlock;
    }
///<<发送PEC，并设置STOP位
    regBase->icDataCmd.fields.dat = pecData | (1 << 9);  ///<<Set STOP bit

    ///< Step 5: Wait for transmission to complete
    if (pDrvData->pSmbusDev.halOps->waitTransmitComplete == NULL) {
        LOGE("%s: HAL waitTransmitComplete not available\n", __func__);
        ret = -ENOTSUP;
        goto unlock;
    }
    ret = pDrvData->pSmbusDev.halOps->waitTransmitComplete(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: General reset transmission failed\n", __func__);
        goto unlock;
    }

    ///< Step 6: Check for abort conditions
    abrtSource.value = regBase->icTxAbrtSource.value;
    if ((abrtSource.value & SMBUS_ARP_ABORT_MASK) != 0) {
        LOGE("%s: General reset failed, abort source=0x%08X\n", __func__, abrtSource.value);
///<清除abort状态
        regBase->icClrTxAbrt = 1;
        ret = -EIO;
        goto unlock;
    }

    ///< Step 7: Allow devices time to process reset command
///<<根据规范，设备需要时间来处理reset命令
    udelay(10000); ///< 10ms delay for devices to complete reset

    LOGD("%s: General reset command sent successfully\n", __func__);

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

/**
 * @brief Get UDID from any device on bus using general discovery
 * @details Sends a general Get UDID command to the ARP default address (0x61) to discover
 *          any device on the SMBus according to SMBus 3.1 specification. This command
 *          allows discovery of devices that are listening at the default ARP address.
 *          The first device to respond will have its UDID read and returned.
 *          This is typically used during initial device discovery phase.
 * @param[in] devId SMBus device identifier for the master controller
 * @param[out] udid Pointer to UDID structure to store the discovered device information
 * @return EXIT_SUCCESS on successful UDID retrieval, negative error code on failure:
 *         -EINVAL: Invalid parameters (NULL udid or invalid register base)
 *         -ENODEV: Device not found or not initialized
 *         -EBUSY: Device locked by another operation
 *         -ENOTSUP: HAL operations not available
 *         -ETIMEDOUT: TX ready or RX ready timeout occurred
 *         -EIO: Command transmission failed, abort detected, or PEC verification failed
 *         -EPROTO: Protocol error (invalid byte count or response format)
 *
 * @note Sends command to ARP default address (0x61)
 * @note Command format: [ARP_ADDR][GET_UDID_CMD][BYTE_COUNT=0][PEC]
 * @note Response format: [BYTE_COUNT][UDID(16 bytes)][PEC]
 * @note Byte count in response should be 16 for valid UDID
 * @note Uses PEC (Packet Error Code) for data integrity verification
 * @note Returns UDID of first device that responds
 * @note Only valid in Master mode
 * @warning udid parameter cannot be NULL
 * @warning May fail if no devices are listening at ARP address
 * @warning Multiple devices responding may cause bus contention
 * @warning Only discovers one device at a time - use directed commands for specific devices
 */
S32 smbusArpGetUdidGeneral(DevList_e devId, SmbusUdid_s *udid)
{
    S32 ret = EXIT_SUCCESS;
    volatile SmbusRegMap_s *regBase = NULL;
    U32 defaultAddr = SMBUS_ARP_ADDR;
    U8 getUdidCmd = SMBUS_CMD_GENERAL_GET_UDID;  ///<<0x03
    U8 byteCount = 0;
    U8 receivedPec = 0;
    U8 calculatedPec = 0;
    U8 udidData[17];  ///<<16 bytes UDID + 1 byte Device Target Address
    U8 pecBuffer[20]; ///<<For PEC calculation
    U8 i;
    U32 pecIndex = 0;
    SmbusIcTxAbrtSourceReg_u abrtSource;

    ///< Parameter validation
    if (udid == NULL) {
        return -EINVAL;
    }

    /* Check driver lock and validate */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        devUnlockByDriver(devId);
        ret = -ENODEV;
        goto exit;
    }

    LOGD("%s: Getting UDID from general address\n", __func__);

    ///< Get driver data and HAL operations for TX/RX checks
    SmbusDrvData_s *pDrvData = NULL;
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS || pDrvData == NULL) {
        LOGE("%s(): Failed to get driver data\n", __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (pDrvData->pSmbusDev.halOps == NULL || pDrvData->pSmbusDev.halOps->checkTxReady == NULL ||
        pDrvData->pSmbusDev.halOps->checkRxReady == NULL) {
        LOGE("%s: HAL operations not available\n", __func__);
        ret = -ENOTSUP;
        goto unlock;
    }

    ///< Step 1: Set target address to ARP default address
    regBase->icTar.fields.icTar = defaultAddr;

    ///< Step 2: Send Get UDID command (Write phase)
    ///< S + Address(Write) + Command
    ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: TX ready timeout for Get UDID command\n", __func__);
        ret = -ETIMEDOUT;
        goto unlock;
    }
    ///< 发送命令，不带STOP位（因为后面要Repeated Start）
    regBase->icDataCmd.fields.dat = getUdidCmd;

    ///< Step 3: Issue Repeated Start and start reading
    ///< Sr + Address(Read) + Byte Count
    
    ///< 首先发送Read命令以触发Repeated Start + 读取Byte Count
    ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: TX ready timeout for read command\n", __func__);
        ret = -ETIMEDOUT;
        goto unlock;
    }
    ///< 设置READ bit，不设置STOP（后续还有数据）
    regBase->icDataCmd.value = (1 << 8);  ///<<CMD_READ bit

    ///< Step 4: Read Byte Count
    ret = pDrvData->pSmbusDev.halOps->checkRxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: RX ready timeout for byte count\n", __func__);
        ret = -ETIMEDOUT;
        goto unlock;
    }
    byteCount = (U8)(regBase->icDataCmd.value & 0xFF);

    ///< Verify byte count should be 17 (0x11)
    ///< 16 bytes UDID + 1 byte Device Target Address
    if (byteCount != 0x11) {
        LOGE("%s: Invalid UDID byte count: 0x%02X (expected 0x11)\n", 
             __func__, byteCount);
        ret = -EPROTO;
        goto unlock;
    }

    ///< Step 5: Issue read commands for 17 data bytes
    for (i = 0; i < 17; i++) {
        ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: TX ready timeout for read command %d\n", __func__, i);
            ret = -ETIMEDOUT;
            goto unlock;
        }
///<<发送READ命令，不设置STOP
        regBase->icDataCmd.value = (1 << 8);  ///<<CMD_READ bit
    }

    ///< Step 6: Issue read command for PEC byte (with STOP)
    ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: TX ready timeout for PEC read command\n", __func__);
        ret = -ETIMEDOUT;
        goto unlock;
    }
///<<发送READ命令，设置STOP位
    regBase->icDataCmd.value = (1 << 8) | (1 << 9);  ///<<CMD_READ + STOP

    ///< Step 7: Read 17 data bytes
    for (i = 0; i < 17; i++) {
        ret = pDrvData->pSmbusDev.halOps->checkRxReady(regBase);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: RX ready timeout for UDID data byte %d\n", __func__, i);
            ret = -ETIMEDOUT;
            goto unlock;
        }
        udidData[i] = (U8)(regBase->icDataCmd.value & 0xFF);
    }

    ///< Step 8: Read PEC byte
    ret = pDrvData->pSmbusDev.halOps->checkRxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: RX ready timeout for PEC\n", __func__);
        ret = -ETIMEDOUT;
        goto unlock;
    }
    receivedPec = (U8)(regBase->icDataCmd.value & 0xFF);

    ///< Step 9: Calculate and verify PEC
    ///<<///<< PEC calculation includes:
    ///< Write Address + Command + Read Address + Byte Count + Data[0..16]
    pecIndex = 0;
    pecBuffer[pecIndex++] = (defaultAddr << 1) | 0x00;  ///< Write address
    pecBuffer[pecIndex++] = getUdidCmd;                 ///<Command
    pecBuffer[pecIndex++] = (defaultAddr << 1) | 0x01;  ///< Read address
    pecBuffer[pecIndex++] = byteCount;                  ///< Byte count
    
    ///<Add all 17 data bytes
    memcpy(pecBuffer, udidData, sizeof(U8) * 17);
    calculatedPec = smbusCalcPEC(calculatedPec, pecBuffer, pecIndex);
    
    if (calculatedPec != receivedPec) {
        LOGE("%s: PEC verification failed: calculated=0x%02X, received=0x%02X\n",
             __func__, calculatedPec, receivedPec);
        ret = -EBADMSG;
        goto unlock;
    }

    ///< Step 10: Check for abort conditions
    abrtSource.value = regBase->icTxAbrtSource.value;
    if ((abrtSource.value & SMBUS_ARP_ABORT_MASK) != 0) {
        LOGE("%s: Get UDID failed, abort source=0x%08X\n", 
             __func__, abrtSource.value);
        regBase->icClrTxAbrt = 1; ///< Clear abort status
        ret = -EIO;
        goto unlock;
    }

    ///< Step 11: Parse and populate UDID structure
    ///< According to protocol notes:
    ///<- Bit 0 (LSB) in Data17 must be 1b
    ///<- If AV flag clear, remaining bits in Data17 should be 0x7F
    
    memset(udid, 0, sizeof(SmbusUdid_s));
    
    ///< Data 1-16: UDID Bytes 15-0 (reverse order)
   ///< Data 17: Device Target Address
    
    ///< Check Data17 LSB (must be 1)
    if ((udidData[16] & 0x01) != 0x01) {
        LOGW("%s: Data17 LSB is not 1b (value=0x%02X)\n", 
             __func__, udidData[16]);
    }
    
    udid->deviceAddr = udidData[16];  ///<<Device Target Address (Data 17)
    
    ///< UDID structure (16 bytes total, Data 1-16 are UDID Bytes 15-0)
    ///< Assuming standard UDID format (may need adjustment based on your definition)
    for (i = 0; i < 16; i++) {
        ///< Store in correct order (Data 1 = Byte 15, Data 16 = Byte 0)
        udid->bytes[15 - i] = udidData[i];
    }

    LOGD("%s: UDID read successfully: addr=0x%02X\n",
         __func__, udid->deviceAddr);
    
    ///< Log UDID for debugging
    LOGD("%s: UDID bytes: ", __func__);
    for (i = 0; i < 16; i++) {
        LOGD("%02X ", udid->bytes[i]);
    }
    LOGD("\n");

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

/**
 * @brief Enumerate all ARP-capable devices on SMBus bus
 * @details Performs complete device discovery and address assignment according to
 *          SMBus 3.1 ARP protocol. This function implements the device enumeration
 *          flow that discovers all devices listening at the ARP default address (0x61),
 *          reads their UDID information, assigns unique addresses, and maintains
 *          a device list for subsequent operations.
 * @param[in] devId SMBus device identifier for the master controller
 * @return Number of devices successfully discovered and configured on success,
 *         negative error code on failure:
 *         -EINVAL: Invalid parameters or device not initialized
 *         -ENODEV: Device not found or not initialized
 *         -EBUSY: Device locked by another operation
 *         -ENOTSUP: HAL operations not available
 *         -EIO: Critical command transmission failed
 *         -ENOMEM: Memory allocation failed for device list
 *
 * @note Implements complete SMBus 3.1 ARP enumeration sequence
 * @note Resets all devices to default address before enumeration
 * @note Assigns addresses from available address pool (0x0C-0x7E)
 * @note Maintains device list with UDID and assigned address mapping
 * @note Uses PEC (Packet Error Code) for all command verification
 * @note Only valid in Master mode
 * @warning This function affects ALL ARP-capable devices on the bus
 * @warning All existing assigned addresses will be reset and reassigned
 * @warning Enumeration may take significant time depending on device count
 * @warning Failed enumeration may leave bus in undefined state
 *
 * [MASTER_API] Master API layer - ARP protocol operations
 */
S32 smbusArpEnumDev(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;
    S32 deviceCount = 0;
    S32 maxRetries = 3;
    S32 retryCount = 0;
    U8 nextAvailableAddr = SMBUS_MIN_DYNAMIC_ADDRESS;
    bool enumerationComplete = false;

    /* Check driver lock and validate */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        devUnlockByDriver(devId);
        return -ENODEV;
    }

    if (pDrvData == NULL) {
        devUnlockByDriver(devId);
        return -EINVAL;
    }

    LOGE("%s: Starting ARP device enumeration\n", __func__);

    ///< Step 1: Clear existing device list and reset ARP master state
    smbusClearDevs(&pDrvData->arpMaster);
    pDrvData->arpMaster.nextAddress = SMBUS_MIN_DYNAMIC_ADDRESS;

    ///< Step 2: Send General Reset Device command
    LOGD("%s: Step 1 - Sending General Reset Device command\n", __func__);
    ret = smbusArpResetDeviceGeneral(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: General Reset Device command failed, ret=%d\n", __func__, ret);
        goto unlock;
    }

    ///< Allow devices time to process reset command
    udelay(50000); ///< 50ms delay for reset processing

    ///< Step 3: Send Prepare to ARP command
    LOGD("%s: Step 2 - Sending Prepare to ARP command\n", __func__);
    ret = smbusArpPrepareToArp(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Prepare to ARP command failed, ret=%d\n", __func__, ret);
        goto unlock;
    }

    ///< Allow devices time to prepare for ARP
    udelay(10000); ///< 10ms delay for ARP preparation

    ///< Step 4: Device discovery loop
    LOGD("%s: Step 3 - Starting device discovery loop\n", __func__);

    while (!enumerationComplete && deviceCount < SMBUS_MAX_DEVICE_NUM) {
        SmbusUdid_s deviceUdid;
        bool deviceFound = false;

        ///< Attempt to discover device with retry mechanism
        for (retryCount = 0; retryCount < maxRetries && !deviceFound; retryCount++) {
            ///< Clear UDID structure
            memset(&deviceUdid, 0, sizeof(SmbusUdid_s));

            ///< Step 4a: Send Get UDID (General) command
            ret = smbusArpGetUdidGeneral(devId, &deviceUdid);

            if (ret == EXIT_SUCCESS) {
                ///< Device responded successfully
                deviceFound = true;
                LOGD("%s: Device discovered (attempt %d)\n", __func__, retryCount + 1);
                break;
            } else if (ret == -EIO || ret == -ETIMEDOUT) {
                ///< No device responded (NACK/Timeout)
                if (retryCount == 0) {
                    LOGD("%s: No more devices responding, enumeration complete\n", __func__);
                }
                deviceFound = false;
                break;
            } else {
                ///< Other error, retry
                LOGW("%s: Get UDID failed (attempt %d), ret=%d\n",
                     __func__, retryCount + 1, ret);
                udelay(5000); ///< 5ms delay between retries
            }
        }

        if (!deviceFound) {
            ///< No more devices on the bus
            enumerationComplete = true;
            continue;
        }

        ///< Step 4b: Validate device UDID and check if already discovered
        if (smbusUdidCompare(&deviceUdid, &deviceUdid) == false) {
            LOGW("%s: Invalid UDID received, skipping device\n", __func__);
            continue;
        }

        ///< Check if device already exists in our list
        if (ArpDevFind(&pDrvData->arpMaster, &deviceUdid) != NULL) {
            LOGD("%s: Device already discovered, skipping\n", __func__);
            continue;
        }

        ///< Step 4c: Assign address to discovered device
        U8 assignedAddr = nextAvailableAddr;

        ///< Find next available address in pool
        while (smbusArpIsAddrUsed(&pDrvData->arpMaster, assignedAddr) &&
               assignedAddr <= SMBUS_MAX_VALID_ADDRESS) {
            assignedAddr++;
        }

        if (assignedAddr > SMBUS_MAX_VALID_ADDRESS) {
            LOGE("%s: No more addresses available in pool\n", __func__);
            break;
        }

        ///< Set the assigned address in UDID structure
        deviceUdid.deviceAddr = assignedAddr;

        LOGD("%s: Assigning address 0x%02X to discovered device\n",
             __func__, assignedAddr);

        ///< Step 4d: Send Assign Address command
        ret = smbusArpAssignAddress(devId, &deviceUdid);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: Assign Address command failed for addr 0x%02X, ret=%d\n",
                 __func__, assignedAddr, ret);
            continue;
        }

        ///< Allow device time to process address assignment
        udelay(10000); ///< 10ms delay for address assignment

        ///< Step 4e: Add device to ARP master device list
        ret = ArpDevInstall(&pDrvData->arpMaster, &deviceUdid, assignedAddr);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s: Failed to install device in list, ret=%d\n", __func__, ret);
            continue;
        }

        ///< Step 4f: Update enumeration state
        deviceCount++;
        nextAvailableAddr = assignedAddr + 1;

        LOGE("%s: Device %d configured successfully - addr=0x%02X, vendor=0x%04X\n",
             __func__, deviceCount, assignedAddr, deviceUdid.vendorId);

        ///< Step 4g: Small delay between device discoveries
        udelay(5000); ///< 5ms delay between devices
    }

    ///< Step 5: Verification phase - try to discover any missed devices
    if (deviceCount > 0) {
        LOGD("%s: Step 4 - Verification phase\n", __func__);

        SmbusUdid_s verifyUdid;
        ret = smbusArpGetUdidGeneral(devId, &verifyUdid);
        if (ret == EXIT_SUCCESS) {
            LOGW("%s: Additional device discovered during verification\n", __func__);
            ///< Note: In production, you might want to handle this case more robustly
        }
    }

    ///< Step 6: Final enumeration summary
    LOGE("%s: ARP enumeration completed - %d devices discovered\n",
         __func__, deviceCount);

    ///< Update driver statistics
    pDrvData->arpMaster.deviceCount = deviceCount;

    if (deviceCount == 0) {
        LOGW("%s: No ARP-capable devices found on bus\n", __func__);
        ret = 0; ///< Return 0 devices found, not an error
    } else {
        ret = deviceCount; ///< Return number of devices discovered
    }

unlock:
    devUnlockByDriver(devId);

    LOGE("%s: ARP enumeration finished with result: %d\n", __func__, ret);
    return ret;
}

/**
 * @brief Send directed reset command to specified UDID device
 * @details Sends a directed reset command to the device with specified UDID
 *          according to SMBus 3.1 ARP protocol. Directed reset command contains
 *          complete UDID information, only matching device will respond.
 *          This is a Master operation that targets another device on the bus.
 * @param[in] devId SMBus device ID
 * @param[in] udid Target device UDID
 * @return 0 on success, negative error code on failure
 *
 * @note Uses SMBus ARP protocol to send directed reset command
 * @note Command format: [ARP_ADDR][CMD][UDID(16 bytes)][PEC]
 * @note Performs complete UDID matching and command transmission flow
 * @note Automatically handles retry and error recovery
 * @warning udid parameter cannot be NULL
 * @warning Failed transmission will automatically clean interrupt status
 *
 * [MASTER_API] Master API layer - ARP protocol operations
 */
S32 smbusArpResetDeviceDirected(DevList_e devId, const SmbusUdid_s *udid)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;
    volatile SmbusRegMap_s *regBase = NULL;

    ///< Parameter validation
    if (udid == NULL) {
        LOGE("%s(): udid is NULL\n", __func__);
        return -EINVAL;
    }

    /* Check driver lock and validate */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        devUnlockByDriver(devId);
        ret = -EINVAL;
        goto unlock;
    }

    ///< Get HAL operations
    // HAL operations already registered in device structure
    if (pDrvData->pSmbusDev.halOps == NULL || pDrvData->pSmbusDev.halOps->i2cWrite == NULL) {
        LOGE("%s(): HAL operations not available\n", __func__);
        ret = -ENOTSUP;
        goto unlock;
    }

    ///< Prepare directed reset command data
    ///< Format: [CMD][UDID(16 bytes)] = 17 bytes
    U8 resetCmd[17];
    resetCmd[0] = SMBUS_CMD_DIRECTED_RESET;  ///< Directed reset command code
    memcpy(&resetCmd[1], udid, 16);           ///< 16-byte UDID

    /* CRITICAL FIX: Use the driver's main device structure directly */
    /* This ensures ISR can access the same device structure - consistent with Block Read fix */
    pDrvData->pSmbusDev.regBase = regBase;
    pDrvData->pSmbusDev.addrMode = pDrvData->sbrCfg.addrMode;
    pDrvData->pSmbusDev.mode = DW_SMBUS_MODE_MASTER;  ///< Reset command sent in Master mode

    LOGD("%s(): Using pDrvData->pSmbusDev directly for directed reset\n", __func__);

    ///< Send directed reset command to ARP address (0x61)
    ret = pDrvData->pSmbusDev.halOps->i2cWrite(&pDrvData->pSmbusDev, SMBUS_ARP_ADDR, resetCmd, 17);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): Directed reset command failed, ret=%d\n", __func__, ret);
        ret = -EIO;
        goto unlock;
    }

    ///< Wait for command completion
    udelay(1000);  ///< 1ms delay for command processing

    ///< Clean possible interrupt status
    regBase->icClrIntr = 0xFFFFFFFF;

    LOGD("%s(): Directed reset command sent successfully\n", __func__);

unlock:
    devUnlockByDriver(devId);
    return ret;
}

/* ======================================================================== */
/*                    Master Configuration Functions                         */
/* ======================================================================== */

/**
 * @brief Configure SMBus controller in master mode
 * @details Configures the DesignWare SMBus controller for master operation.
 *          This function disables the controller, sets the appropriate
 *          configuration bits for master mode, and configures timing
 *          and FIFO thresholds.
 * @param[in] dev Pointer to the SMBus device structure
 * @return void
 *
 * @note Disables controller before reconfiguration
 * @note Sets fast mode (2) as default speed
 * @note Enables RESTART conditions for combined transactions
 * @note ARP is disabled by default in master mode
 * @note Configures TX/RX FIFO thresholds for interrupt generation
 * @warning The function does not re-enable the controller after configuration
 * [MASTER_API] Master API layer - master configuration
 */
__attribute((unused)) static void smbusConfigureMaster(SmbusDev_s *dev)
{
    SmbusIcConReg_u con;

    if (dev == NULL || dev->regBase == NULL) {
        return;
    }

    /* Check HAL operations */
    if (dev->halOps == NULL || dev->halOps->disable == NULL) {
        return;
    }

    /* Disable before configuration */
    dev->halOps->disable(dev);

    /* Read current configuration */
    con.value = dev->regBase->icCon.value;

    /* Configure as master */
    con.fields.masterMode = 1;
    con.fields.icSlaveDisable = 1;
    con.fields.icRestartEn = 1;
    con.fields.speed = 2;  /* Fast mode by default */

    /* Set address mode */
    if (dev->addrMode == 1) {
        con.fields.ic10bitaddrMaster = 1;
    } else {
        con.fields.ic10bitaddrMaster = 0;
    }

    /* Enable SMBus features */
    con.fields.smbusArpEn = 0;  /* ARP disabled by default in master mode */

    /* Write configuration */
    dev->regBase->icCon.value = con.value;

    /* Set TX/RX FIFO thresholds */
    dev->regBase->icTxTl = 0;  /* TX threshold: generate interrupt when TX FIFO is empty */
    dev->regBase->icRxTl = 0;  /* RX threshold: generate interrupt when RX FIFO has 1+ bytes */
}

/**
 * @brief Prepare devices for ARP (Address Resolution Protocol)
 * @details Sends Prepare to ARP command to all SMBus devices to prepare them
 *          for address assignment operations. This is the first step in SMBus ARP.
 * @param[in] devId SMBus device identifier from DevList_e enumeration
 * @return EXIT_SUCCESS on success, negative error code on failure:
 *         - -ENODEV: Invalid device identifier
 *         - -EINVAL: Invalid register base
 *         - -ETIMEDOUT: TX ready timeout
 *         - -EIO: Transmission failed or abort condition detected
 *         - -ENOTSUP: HAL operations not available
 */
S32 smbusArpPrepareToArp(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    volatile SmbusRegMap_s *regBase;
    U32 defaultAddr = SMBUS_ARP_ADDR;
    U8 prepareData = SMBUS_CMD_PREPARE_FOR_ARP;
    U8 pecData = 0;
    SmbusIcTxAbrtSourceReg_u abrtSource;

    ///< Get register base
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        LOGE("%s(): getSmbusReg failed\n", __func__);
        return -ENODEV;
    }

    LOGD("%s(): Sending Prepare to ARP command\n", __func__);

    ///< Get driver data and HAL operations for TX checks
    SmbusDrvData_s *pDrvData = NULL;
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS || pDrvData == NULL) {
        LOGE("%s(): Failed to get driver data\n", __func__);
        return -EINVAL;
    }

    if (pDrvData->pSmbusDev.halOps == NULL || pDrvData->pSmbusDev.halOps->checkTxReady == NULL) {
        LOGE("%s(): HAL operations not available\n", __func__);
        return -ENOTSUP;
    }

    ///< Step 1: Set target address to ARP default address
    regBase->icTar.fields.icTar = defaultAddr;

    ///< Step 2: Send Prepare to ARP command
    ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): TX ready timeout for prepare command\n", __func__);
        return -ETIMEDOUT;
    }
    regBase->icDataCmd.fields.dat = prepareData;

    ///< Step 3: Calculate and send PEC
    pecData = smbusPecPktConstruct(defaultAddr, true, &prepareData, 1);
    pecData = (pecData & 0xff) | (1 << 9); ///< Set STOP bit

    ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): TX ready timeout for PEC\n", __func__);
        return -ETIMEDOUT;
    }
    regBase->icDataCmd.fields.dat = pecData;

    ///< Step 4: Wait for transmission to complete
    ret = pDrvData->pSmbusDev.halOps->waitTransmitComplete(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): Prepare to ARP transmission failed\n", __func__);
        return ret;
    }

    ///< Step 5: Check for abort conditions
    abrtSource.value = regBase->icTxAbrtSource.value;
    if ((abrtSource.value & SMBUS_ARP_ABORT_MASK) != 0) {
        LOGE("%s(): Prepare to ARP failed, abort source=0x%08X\n", __func__, abrtSource.value);
        return -EIO;
    }

    LOGD("%s(): Prepare to ARP command sent successfully\n", __func__);
    return EXIT_SUCCESS;
}

/**
 * @brief Assign address to a specific SMBus device
 * @details Sends Assign Address command to a specific device using its UDID.
 *          This command assigns a new address to a device that has previously
 *          been prepared for ARP.
 * @param[in] devId SMBus device identifier from DevList_e enumeration
 * @param[in] udid Pointer to device UDID structure containing device identification
 * @return EXIT_SUCCESS on success, negative error code on failure:
 *         - -ENODEV: Invalid device identifier
 *         - -EINVAL: Invalid parameter or register base
 *         - -ETIMEDOUT: TX ready timeout
 *         - -EIO: Transmission failed or abort condition detected
 *         - -ENOTSUP: HAL operations not available
 */
S32 smbusArpAssignAddress(DevList_e devId, const SmbusUdid_s *udid)
{
    S32 ret = EXIT_SUCCESS;
    volatile SmbusRegMap_s *regBase;
    U32 defaultAddr = SMBUS_ARP_ADDR;
    U8 assignAddrCmd = SMBUS_CMD_GENERAL_ASSIGN_ADDR;
    U8 udidData[16];
    U8 pec = 0;
    U8 i;
    SmbusIcTxAbrtSourceReg_u abrtSource;

    if (udid == NULL) {
        return -EINVAL;
    }

    ///< Get register base
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        LOGE("%s(): getSmbusReg failed\n", __func__);
        return -ENODEV;
    }

    LOGD("%s(): Assigning address to device\n", __func__);

    ///< Get driver data and HAL operations for TX checks
    SmbusDrvData_s *pDrvData = NULL;
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS || pDrvData == NULL) {
        LOGE("%s(): Failed to get driver data\n", __func__);
        return -EINVAL;
    }

    if (pDrvData->pSmbusDev.halOps == NULL || pDrvData->pSmbusDev.halOps->checkTxReady == NULL) {
        LOGE("%s(): HAL operations not available\n", __func__);
        return -ENOTSUP;
    }

    ///< Step 1: Convert UDID structure to byte array
    udidData[0] = udid->nextAvailAddr;
    udidData[1] = udid->version;
    udidData[2] = (U8)(udid->vendorId & 0xFF);
    udidData[3] = (U8)((udid->vendorId >> 8) & 0xFF);
    udidData[4] = (U8)(udid->deviceId & 0xFF);
    udidData[5] = (U8)((udid->deviceId >> 8) & 0xFF);
    udidData[6] = (U8)(udid->interface & 0xFF);
    udidData[7] = (U8)((udid->interface >> 8) & 0xFF);
    udidData[8] = (U8)(udid->subsystemVendorId & 0xFF);
    udidData[9] = (U8)((udid->subsystemVendorId >> 8) & 0xFF);
    for (i = 0; i < 6; i++) {
        udidData[10 + i] = udid->bytes[i];
    }

    ///< Step 2: Set target address to ARP default address
    regBase->icTar.fields.icTar = defaultAddr;

    ///< Step 3: Send Assign Address command
    ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): TX ready timeout for assign command\n", __func__);
        return -ETIMEDOUT;
    }
    regBase->icDataCmd.fields.dat = assignAddrCmd;

    ///< Step 4: Send Byte Count = 17 (16 bytes UDID + 1 byte address)
    ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): TX ready timeout for byte count\n", __func__);
        return -ETIMEDOUT;
    }
    regBase->icDataCmd.fields.dat = 17;

    ///< Step 5: Send 16-byte UDID data
    for (i = 0; i < 16; i++) {
        ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
        if (ret != EXIT_SUCCESS) {
            LOGE("%s(): TX ready timeout for UDID data byte %d\n", __func__, i);
            return -ETIMEDOUT;
        }
        regBase->icDataCmd.fields.dat = udidData[i];
    }

    ///< Step 6: Send Assigned Address
    ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): TX ready timeout for assigned address\n", __func__);
        return -ETIMEDOUT;
    }
    regBase->icDataCmd.fields.dat = udid->deviceAddr;

    ///< Step 7: Calculate and send PEC
    U8 pec_buffer[19];
    pec_buffer[0] = SMBUS_CMD_GENERAL_ASSIGN_ADDR;  ///< Command
    pec_buffer[1] = 17;  ///< Byte count
    for (i = 0; i < 16; i++) {
        pec_buffer[i + 2] = udidData[i];
    }
    pec_buffer[18] = udid->deviceAddr;  ///< Assigned address

    pec = smbusPecPktConstruct(defaultAddr, true, pec_buffer, 19);
    pec = (pec & 0xff) | (1 << 9); ///< Set STOP bit

    ret = pDrvData->pSmbusDev.halOps->checkTxReady(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): TX ready timeout for PEC\n", __func__);
        return -ETIMEDOUT;
    }
    regBase->icDataCmd.fields.dat = pec;

    ///< Step 8: Wait for transmission to complete
    if (pDrvData->pSmbusDev.halOps->waitTransmitComplete == NULL) {
        LOGE("%s(): HAL waitTransmitComplete not available\n", __func__);
        return -ENOTSUP;
    }
    ret = pDrvData->pSmbusDev.halOps->waitTransmitComplete(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("%s(): Assign address transmission failed\n", __func__);
        return ret;
    }

    ///< Step 9: Check for abort conditions
    abrtSource.value = regBase->icTxAbrtSource.value;
    if ((abrtSource.value & SMBUS_ARP_ABORT_MASK) != 0) {
        LOGE("%s(): Assign address failed, abort source=0x%08X\n", __func__, abrtSource.value);
        return -EIO;
    }

    LOGD("%s(): Address 0x%02X assigned successfully\n", __func__, udid->deviceAddr);
    return EXIT_SUCCESS;
}

/**
 * @brief SMBus I2C reset function for hardware recovery
 * @details This function provides API-level access to the SMBus I2C reset functionality.
 *          It performs driver validation, locking, and calls the HAL reset implementation.
 * @param[in] devId SMBus device identifier
 * @return EXIT_SUCCESS on successful reset, negative error code on failure:
 *         -EINVAL: Invalid parameters or device not properly initialized
 *         -ENOTSUP: HAL reset operations not available
 *         -EIO: Hardware reset or reinitialization failed
 *         -EBUSY: Device locking failed
 *
 * @note This function is thread-safe and uses device locking
 * @note Resets both Master and Slave mode configurations
 * @note Performs complete hardware recovery sequence
 * @note Reinitializes controller in current mode after reset
 * @warning This function performs hardware reset - ensure bus isolation if needed
 * @warning Controller will be temporarily disabled during reset sequence
 * [MASTER_API] Master API layer - high-level I2C compatibility reset operations
 */
S32 smbusReset(DevList_e devId)
{
    SmbusDrvData_s *pDrvData = NULL;
    S32 ret = EXIT_SUCCESS;

    LOGD("SMBus I2C Reset: Starting API reset for device %d\n", devId);

    /* ===== 1. Driver Validation and Lock ===== */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus I2C Reset: Driver validation failed: %d\n", ret);
        return ret;
    }

    /* ===== 2. Get Driver Data ===== */
    ret = getDevDriver(devId, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS || pDrvData == NULL) {
        LOGE("SMBus I2C Reset: Failed to get driver data\n");
        ret = -EINVAL;
        goto exit;
    }

       /* Verify controller status */
    volatile U32 sar2 = pDrvData->pSmbusDev.regBase->icSar2.value;
    LOGD("SMBus I2C Reset: Controller status after reset: 0x%08X\n", sar2);
    /* ===== 3. Check HAL Operations ===== */
    if (pDrvData->pSmbusDev.halOps == NULL) {
        LOGE("SMBus I2C Reset: HAL operations not available\n");
        ret = -ENOTSUP;
        goto exit;
    }

    /* ===== 4. Log Device State Before Reset ===== */
    LOGD("SMBus I2C Reset: Current state - mode=%s, enabled=%d, busId=%u\n",
         (pDrvData->pSmbusDev.mode == DW_SMBUS_MODE_MASTER) ? "MASTER" : "SLAVE",
         pDrvData->pSmbusDev.restartEnb, pDrvData->pSmbusDev.busId);

    /* ===== 5. Perform HAL Reset Operation ===== */
    if (pDrvData->pSmbusDev.halOps->i2cReset != NULL) {
        LOGD("SMBus I2C Reset: Using I2C reset operation (SMBus reset not available)\n");
        ret = pDrvData->pSmbusDev.halOps->i2cReset(&pDrvData->pSmbusDev, devId);
    } else {
        LOGE("SMBus I2C Reset: Both SMBus and I2C reset operations not available\n");
        ret = -ENOTSUP;
        goto exit;
    }

    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus I2C Reset: HAL reset operation failed: %d\n", ret);
        ret = -EIO;
        goto exit;
    }

    /* ===== 6. Verify Reset Success ===== */
    /* Wait a moment for hardware to settle */
    udelay(2000);  /* 2ms delay */

    /* Check if device is still accessible */
    if (pDrvData->pSmbusDev.regBase == NULL) {
        LOGE("SMBus I2C Reset: Device register base lost after reset\n");
        ret = -EIO;
        goto exit;
    }

    /* Verify controller status */
    volatile U32 status = pDrvData->pSmbusDev.regBase->icStatus.value;
    LOGD("SMBus I2C Reset: Controller status after reset: 0x%08X\n", status);

    /* ===== 7. Success Path ===== */
    LOGE("SMBus I2C Reset: API reset completed successfully for device %d\n", devId);
    LOGE("SMBus I2C Reset: Device restored to %s mode\n",
         (pDrvData->pSmbusDev.mode == DW_SMBUS_MODE_MASTER) ? "MASTER" : "SLAVE");

    /* ===== 8. Verify and Restore Device State ===== */
    /* Ensure device semaphores and internal state are properly initialized */
    if (pDrvData->pSmbusDev.semaphoreId == 0) {
        LOGD("SMBus I2C Reset: Device semaphoreId is 0, attempting to restore device state\n");

        /* CRITICAL FIX: Recreate semaphore using the static inline function */
        S32 semRet = smbusCreateSemaphore(&pDrvData->pSmbusDev, "SMBus I2C Reset");
        if (semRet != 0) {
            ret = semRet;
            goto exit;
        }

        /* Re-initialize device state if needed */
        if (pDrvData->pSmbusDev.mode == DW_SMBUS_MODE_MASTER) {
            /* Re-configure Master mode settings */
            pDrvData->pSmbusDev.transferTimeout = SMBUS_DEFAULT_TIMEOUT_MS;
            pDrvData->pSmbusDev.status = 0;  /* Clear any pending status */
            pDrvData->pSmbusDev.rxOutstanding = 0;
            pDrvData->pSmbusDev.msgWriteIdx = 0;
            pDrvData->pSmbusDev.msgReadIdx = 0;
            pDrvData->pSmbusDev.msgsNum = 0;
            LOGD("SMBus I2C Reset: Restored Master mode timeout and state settings\n");
        } else if (pDrvData->pSmbusDev.mode == DW_SMBUS_MODE_SLAVE) {
            /* Re-configure Slave mode settings */
            pDrvData->pSmbusDev.transferTimeout = SMBUS_DEFAULT_TIMEOUT_MS;
            pDrvData->pSmbusDev.status = 0;  /* Clear any pending status */
            pDrvData->pSmbusDev.slaveValidRxLen = 0;
            LOGD("SMBus I2C Reset: Restored Slave mode timeout and state settings\n");
        }

        /* Verify basic device functionality */
        if (pDrvData->pSmbusDev.regBase != NULL) {
            volatile U32 enabled = pDrvData->pSmbusDev.regBase->icEnable.value;
            if (enabled == 0) {
                LOGD("SMBus I2C Reset: Re-enabling controller after reset\n");
                pDrvData->pSmbusDev.regBase->icEnable.value = 1;
                /* Small delay for controller to stabilize */
                udelay(1000);  /* 1ms delay */
            }
        }
    }

    /* Log final device state for debugging */
    LOGD("SMBus I2C Reset: Final state - mode=%s, reenabled=%d, semaphoreId=%u\n",
         (pDrvData->pSmbusDev.mode == DW_SMBUS_MODE_MASTER) ? "MASTER" : "SLAVE",
         pDrvData->pSmbusDev.restartEnb, pDrvData->pSmbusDev.semaphoreId);

    ret = EXIT_SUCCESS;

exit:
    /* ===== 9. Unlock Device ===== */
    devUnlockByDriver(devId);

    LOGD("SMBus I2C Reset: API reset completed with result: %d\n", ret);
    return ret;
}