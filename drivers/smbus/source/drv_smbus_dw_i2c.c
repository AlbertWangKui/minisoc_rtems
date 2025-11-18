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
 *
 * @note This file implements the I2C Hardware register Layer, providing
 *       direct register access functions and low-level hardware operations.
 *       These functions operate directly on the DesignWare SMBus controller
 *       registers and provide the foundation for higher-level protocols.
 */

#include <rtems.h>
#include <rtems/counter.h>
#include <rtems/bspIo.h>
#include "log_msg.h"
#include "udelay.h"
#include "osp_interrupt.h"
#include "osp_barrier.h"
#include "drv_smbus_dw_i2c.h"
#include "drv_smbus_dw.h"

#define SMBUS_CMD_GENERAL_GET_UDID    (0x03)
#define SMBUS_CMD_GENERAL_ASSIGN_ADDR (0x04)
/* ======================================================================== */
/*                    Function Forward Declarations                          */
/* ======================================================================== */

static S32 smbusDwXferPoll(SmbusDev_s *dev, SmbusMsg_t *msgs, U32 num);
static S32 smbusWaitBusNotBusy(SmbusDev_s *dev);
static bool smbusIsControllerActive(SmbusDev_s *dev);
static S32 smbusHandleTxAbort(SmbusDev_s *dev);

/* ======================================================================== */
/*                    HAL Layer - Register Access Functions                  */
/* ======================================================================== */

/**
 * @brief Check if TX FIFO is ready (not full)
 * [HAL] Hardware abstraction layer - direct register access
 */
static S32 smbusCheckTxready(volatile SmbusRegMap_s *regBase)
{
    SmbusIcStatusReg_u status;
    U32 timeout = SMBUS_TX_READY_TIMEOUT_US;
    while (timeout--) {
        status.value = regBase->icStatus.value;
        if (status.fields.tfnf) {  /* TX FIFO not full */
            return 0;
        }
        udelay(1);
    }
    return -ETIMEDOUT;
}

/**
 * @brief Check if RX FIFO has data
 * [HAL] Hardware abstraction layer - direct register access
 */
static S32 smbusCheckRxready(volatile SmbusRegMap_s *regBase)
{
    SmbusIcStatusReg_u status;
    U32 timeout = SMBUS_RX_READY_TIMEOUT_US;
    while (timeout--) {
        status.value = regBase->icStatus.value;
        if (status.fields.rfne) {  /* RX FIFO not empty */
            return 0;
        }
        udelay(1);
    }
    return -ETIMEDOUT;
}

/**
 * @brief Wait for the current SMBus transaction to complete on the bus.
 *        This function should be called after all data has been written to the TX FIFO
 *        or after all expected data has been read from the RX FIFO.
 * @param pDrvData Pointer to the driver's private data structure.
 * @return 0 on success, -ETIMEDOUT on failure.
 * [HAL] Hardware abstraction layer - direct register access
 */
static S32 waitTransmitComplete(volatile SmbusRegMap_s *regBase)
{
    SmbusIcStatusReg_u status;
    U32 timeout = SMBUS_TRANSACTION_TIMEOUT_US; // Timeout may need to be longer for full transaction

    while (timeout--) {
        status.value = regBase->icStatus.value;
        // We wait for the 'activity' bit to be cleared, indicating the
        // master is no longer active on the bus (i.e., STOP condition has been sent).
        if (!status.fields.activity) {
            return 0; // Transaction completed successfully
        }
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(10));
    }
    return -ETIMEDOUT; // Transaction timeout
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
 * [HAL] Hardware abstraction layer - direct register access
 */
static S32 smbusSetSlaveAddr(SmbusDev_s *dev)
{
    SmbusIcSarReg_u sar;
    SmbusIcEnableReg_u enable;

    if (dev == NULL || dev->regBase == NULL) {
        return -EINVAL;
    }

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

    return 0;
}

/* ======================================================================== */
/*                    HAL Layer - Controller Management                        */
/* ======================================================================== */

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
 * [HAL] Hardware abstraction layer - direct register access
 */
static void smbusDisable(SmbusDev_s *dev)
{
    U32 timeout = 100;
    SmbusIcEnableReg_u enable;
    SmbusIcStatusReg_u status;

    if (dev == NULL || dev->regBase == NULL) {
        return;
    }

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
 * [HAL] Hardware abstraction layer - direct register access
 */
static void smbusEnable(SmbusDev_s *dev)
{
    SmbusIcEnableReg_u enable;

    if (dev == NULL || dev->regBase == NULL) {
        return;
    }

    enable.value = dev->regBase->icEnable.value;
    enable.fields.enable = 1;
    dev->regBase->icEnable.value = enable.value;
    dev->status |= SMBUS_STATUS_ACTIVE;  /* Set STATUS_ACTIVE */
}

/* ======================================================================== */
/*                    HAL Layer - ARP Protocol Functions                        */
/* ======================================================================== */

/**
 * @brief SMBus ARP Reset Device command implementation
 * @details Implements the SMBus ARP Reset Device command according to the
 *          SMBus specification. This function sends the reset command to
 *          the target device at the ARP default address and includes PEC.
 * @param[in] regBase Pointer to the SMBus controller register base
 * @return EXIT_SUCCESS on success, negative error code on failure:
 *         -ETIMEDOUT: TX FIFO timeout
 *         -ENOBUFS: PEC transmission timeout
 *         -ECONNRESET: NACK detected or transmission abort
 *
 * @note Sends reset command to SMBUS_ARP_ADDR (0x61)
 * @note Includes PEC calculation and transmission
 * @note Waits for slave acknowledgment with 1ms timeout
 * @warning This function should only be called in Master mode
 * [HAL] Hardware abstraction layer - ARP protocol operations
 */
static U32 smbusArpResetDevice(volatile SmbusRegMap_s *regBase)
{
    U32 ret = EXIT_SUCCESS;
    U8 data_buf[1];
    U8 defaultAddr = SMBUS_ARP_ADDR;
    U32 arpResetData = 0;
    U32 pecData = 0;
    SmbusIcTxAbrtSourceReg_u abrtSource;

    /* Step 1: Send reset device command */
    arpResetData = ((SMBUS_CMD_GENERAL_RESET_DEVICE) & 0xff) | (0 << 8);
    if (smbusCheckTxready(regBase) != 0) {
        ret = -ETIMEDOUT;
        goto exit;
    }
    regBase->icDataCmd.value = arpResetData;

    /* Step 2: Fill PEC */
    data_buf[0] = SMBUS_CMD_GENERAL_RESET_DEV;
    pecData = smbusPecPktConstruct(defaultAddr, true, data_buf, 1);
    pecData = (pecData & 0xff) | (0 << 8) | (1 << 9);
    if (smbusCheckTxready(regBase) != 0) {
        ret = ENOBUFS;
        goto exit;
    }
    regBase->icDataCmd.value = pecData;

    /* Wait for Slave ACK or timeout */
    udelay(1000);
    abrtSource.value = regBase->icTxAbrtSource.value;
    if ((abrtSource.value & SMBUS_ARP_ABORT_MASK) != 0) {
        /* NACK detected */
        ret = -ECONNRESET;
    }

exit:
    return ret;
}

/**
 * @brief SMBus ARP Address Assign command implementation
 * @details Implements the SMBus ARP Address Assign command according to the
 *          SMBus specification. This function assigns a new address to a device
 *          by sending the UDID and the assigned address to the target device.
 * @param[in] regBase Pointer to the SMBus controller register base
 * @param[in,out] udid_word0 Pointer to UDID data (32-bit word array)
 * @param[in] assign_addr Address to assign to the device
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Sends Assign Address command to SMBUS_ARP_ADDR (0x61)
 * @note Transmits 17-byte UDID data plus assigned address
 * @note Includes PEC calculation and transmission
 * @warning This function should only be called in Master mode
 * [HAL] Hardware abstraction layer - ARP protocol operations
 */
static U32 smbusArpAddressAssign(volatile SmbusRegMap_s *regBase, U32 *udid_word0, U8 assign_addr)
{
    U32 ret = EXIT_SUCCESS;
    U32 i;
    U8 *buffer = (U8 *)udid_word0;
    U8 data_buf[19];
    U8 pec = 0;
    U8 default_addr = SMBUS_ARP_ADDR;
    U32 assignAddrCmd = 0;
    U8 index = 0;

    /* Step 1: Send Assign Address command */
    assignAddrCmd = ((SMBUS_CMD_GENERAL_ASSIGN_ADDR) & 0xff) | (0 << 8);
    if (smbusCheckTxready(regBase) != 0) {
        ret = -ETIMEDOUT;
        goto exit;
    }
    regBase->icDataCmd.value = assignAddrCmd;
    data_buf[index] = (SMBUS_CMD_GENERAL_ASSIGN_ADDR);
    index++;

    /* Step 2: Write Byte Count = 17 */
    assignAddrCmd = ((17) & 0xff) | (0 << 8);
    if (smbusCheckTxready(regBase) != 0) {
        ret = -EISCONN;
        goto exit;
    }
    regBase->icDataCmd.value = assignAddrCmd;
    data_buf[index] = 17;
    index++;

    /* Step 3: Write UDID data (16 bytes) */
    for (i = 0; i < 16; i++) {
        assignAddrCmd = (buffer[i] & 0xff) | (0 << 8);
        if (smbusCheckTxready(regBase) != 0) {
            ret = -EIO;
            goto exit;
        }
        regBase->icDataCmd.value = assignAddrCmd;
        data_buf[index] = buffer[i];
        index++;
    }

    /* Step 4: Write Assigned Address */
    assignAddrCmd = (assign_addr & 0xff) | (0 << 8);
    if (smbusCheckTxready(regBase) != 0) {
        ret = -EIO;
        goto exit;
    }
    regBase->icDataCmd.value = assignAddrCmd;
    data_buf[index] = assign_addr;
    index++;

    /* Step 5: Calculate and send PEC */
    pec = smbusPecPktConstruct(default_addr, true, data_buf, 18);
    pec = (pec & 0xff) | (0 << 8) | (1 << 9);
    if (smbusCheckTxready(regBase) != 0) {
        ret = -ENOBUFS;
        goto exit;
    }
    regBase->icDataCmd.value = pec;

exit:
    return ret;
}

/**
 * @brief SMBus ARP Get UDID command implementation
 * @details Implements the SMBus ARP Get UDID command according to the
 *          SMBus specification. This function retrieves the UDID from
 *          the target device at the ARP default address.
 * @param[in] regBase Pointer to the SMBus controller register base
 * @param[in,out] udid_word0 Pointer to store UDID data (32-bit word array)
 * @param[in] timeout Timeout for the operation in microseconds
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Sends Get UDID command to SMBUS_ARP_ADDR (0x61)
 * @note Reads 16-byte UDID data plus PEC
 * @note Includes PEC verification
 * @warning This function should only be called in Master mode
 * [HAL] Hardware abstraction layer - ARP protocol operations
 */
static U32 smbusUdidGet(volatile SmbusRegMap_s *regBase, U32 *udid_word0, U32 timeout)
{
    U32 ret = EXIT_SUCCESS;
    U32 i;
    U8 buffer[19];
    U8 data_buf[20];
    U8 pec = 0;
    U32 data_index = 0;
    U8 default_addr = SMBUS_ARP_ADDR;
    U32 udidGetCmd = ((SMBUS_CMD_GENERAL_GET_UDID) & 0xff) | (0 << 8);
    U32 data;

    /* Initialize buffers */
    memset(buffer, 0, sizeof(buffer));
    memset(data_buf, 0, sizeof(data_buf));

    /* Step 1: Send Get UDID command */
    if (smbusCheckTxready(regBase) != 0) {
        ret = -EPROTO;
        goto exit;
    }
    regBase->icDataCmd.value = udidGetCmd;

    /* Step 2: Send Byte Count = 0 */
    udidGetCmd = (0 & 0xff) | (1 << 8) | (1 << 10); /* Read + STOP */
    if (smbusCheckTxready(regBase) != 0) {
        ret = -EPROTO;
        goto exit;
    }
    regBase->icDataCmd.value = udidGetCmd;

    /* Step 3: Read UDID data (16 bytes) + Byte Count (1 byte) + PEC (1 byte) = 18 bytes total */
    data_index = 0;

    /* Read Byte Count first */
    if (smbusCheckRxready(regBase) != 0) {
        ret = -ETIMEDOUT;
        goto exit;
    }
    data = regBase->icDataCmd.value;
    buffer[data_index++] = (U8)(data & 0xFF);

    /* Verify byte count should be 16 */
    if (buffer[0] != 16) {
        LOGW("SMBus: Unexpected UDID byte count: %d (expected 16)\n", buffer[0]);
    }

    /* Read 16-byte UDID data */
    for (i = 0; i < 16; i++) {
        if (smbusCheckRxready(regBase) != 0) {
            ret = -ETIMEDOUT;
            goto exit;
        }
        data = regBase->icDataCmd.value;
        buffer[data_index++] = (U8)(data & 0xFF);
        data_buf[data_index] = (U8)(data & 0xFF);
    }

    /* Read PEC byte */
    if (smbusCheckRxready(regBase) != 0) {
        ret = -ETIMEDOUT;
        goto exit;
    }
    data = regBase->icDataCmd.value;
    pec = (U8)(data & 0xFF);
    buffer[data_index++] = pec;
    data_buf[data_index] = pec;

    /* Verify PEC */
    data_buf[0] = SMBUS_CMD_GENERAL_GET_UDID;  /* Command */
    data_buf[1] = 0;  /* Byte count = 0 for Get UDID */
    U8 calculated_pec = smbusPecPktConstruct(default_addr, false, data_buf, 2 + 16);

    if (calculated_pec != pec) {
        LOGE("SMBus: UDID PEC verification failed: calculated=0x%02X, received=0x%02X\n",
             calculated_pec, pec);
        ret = -EIO;
        goto exit;
    }

    /* Copy UDID data to output buffer (32-bit word array) */
    if (udid_word0 != NULL) {
        U8 *udid_bytes = (U8 *)udid_word0;
        for (i = 0; i < 16; i++) {
            udid_bytes[i] = buffer[i + 1];  /* Skip byte count, copy UDID data */
        }

        LOGD("SMBus: UDID read successfully: %02X %02X %02X %02X %02X %02X %02X %02X\n",
             udid_bytes[0], udid_bytes[1], udid_bytes[2], udid_bytes[3],
             udid_bytes[4], udid_bytes[5], udid_bytes[6], udid_bytes[7]);
        LOGD("SMBus: UDID continued: %02X %02X %02X %02X %02X %02X %02X %02X\n",
             udid_bytes[8], udid_bytes[9], udid_bytes[10], udid_bytes[11],
             udid_bytes[12], udid_bytes[13], udid_bytes[14], udid_bytes[15]);
    }

exit:
    return ret;
}

/**
 * @brief Core SMBus device address assignment logic
 * @details Implements the core logic for assigning an address to an SMBus
 *          device using ARP protocol. This function configures the controller
 *          and performs the address assignment sequence.
 * @param[in] regBase Pointer to the SMBus controller register base
 * @param[in] assign_addr Address to assign to the device
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Configures controller for 7-bit addressing
 * @note Sets target address to SMBUS_ARP_ADDR
 * @note Calls ARP address assignment protocol
 * @warning This function should only be called in Master mode
 * [HAL] Hardware abstraction layer - ARP protocol operations
 */
static U32 smbusDevAddrAssignCore(volatile SmbusRegMap_s *regBase, U8 assign_addr)
{
    U32 ret = EXIT_SUCCESS;
    U32 default_addr = SMBUS_ARP_ADDR;
    U32 udid_word[4] = {0};
    U32 timeout;
    SmbusIcConReg_u ic_con;

    /* Step 1: Set IC_TAR as ARP default address */
    /* Disable controller before configuration */
    SmbusIcEnableReg_u enable;
    enable.value = regBase->icEnable.value;
    enable.fields.enable = 0;
    regBase->icEnable.value = enable.value;

    /* Configure for 7-bit addressing */
    ic_con.value = regBase->icCon.value;
    ic_con.fields.ic10bitaddrMaster = 0;
    regBase->icCon.value = ic_con.value;

    /* Set TAR to ARP default address */
    default_addr = default_addr & (0x7F);
    regBase->icTar.fields.icTar = default_addr;

    /* Re-enable controller for address assignment */
    enable.fields.enable = 1;
    regBase->icEnable.value = enable.value;

    /* Wait for controller to be ready */
    timeout = SMBUS_TX_READY_TIMEOUT_US;
    while (timeout--) {
        SmbusIcStatusReg_u status;
        status.value = regBase->icStatus.value;
        if (!status.fields.activity) {
            break;
        }
        udelay(1);
    }

    if (timeout == 0) {
        LOGE("SMBus: Controller not ready for address assignment\n");
        ret = -ETIMEDOUT;
        goto exit;
    }

    /* Step 2: Send ARP Reset Device command first */
    ret = smbusArpResetDevice(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: ARP reset device failed, ret=%d\n", ret);
        goto exit;
    }

    /* Wait for reset to complete */
    udelay(1000);

    /* Step 3: Get UDID from target device */
    ret = smbusUdidGet(regBase, udid_word, SMBUS_TRANSACTION_TIMEOUT_US);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: Get UDID failed, ret=%d\n", ret);
        goto exit;
    }

    /* Step 4: Assign new address to device */
    ret = smbusArpAddressAssign(regBase, udid_word, assign_addr);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: ARP address assign failed, ret=%d\n", ret);
        goto exit;
    }

    /* Wait for address assignment to complete */
    udelay(2000);

    LOGI("SMBus: Address 0x%02x assigned successfully\n", assign_addr);

exit:
    /* Disable controller before exit */
    enable.fields.enable = 0;
    regBase->icEnable.value = enable.value;

    return ret;
}

/* ======================================================================== */
/*                    HAL Layer - I2C Compatible Transfer Functions               */
/* ======================================================================== */

/**
 * @brief Initialize SMBus transfer based on I2C xfer_init pattern
 * @details This function initializes the SMBus controller for transfer by
 *          configuring addressing mode, target address, and controller settings.
 *          It follows the same pattern as i2c_dw_xfer_init but uses SMBus
 *          register access and configuration.
 * @param[in] dev Pointer to SMBus device structure
 * @param[in] msgs Array of SMBus messages
 * @param[in] msgIdx Current message index
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Disables controller before reconfiguration
 * @note Supports both 7-bit and 10-bit addressing modes
 * @note Configures controller for master mode operation
 * [HAL] Hardware abstraction layer - Transfer initialization
 */
static S32 smbusDwXferInit(SmbusDev_s *dev, SmbusMsg_t msgs[], U32 msgIdx)
{
    SmbusIcConReg_u icCon;
    SmbusIcTarReg_u icTar;
    SmbusIcEnableReg_u enable;
    U32 dummy;

    if (dev == NULL || dev->regBase == NULL || msgs == NULL) {
        LOGE("SMBus: Invalid parameters for xfer init\n");
        return -EINVAL;
    }
    
    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)dev->regBase;
    /* Disable the adapter */
    enable.value = regBase->icEnable.value;
    enable.fields.enable = 0;
    regBase->icEnable.value = enable.value;

    /* Configure control register */
    icCon.value = regBase->icCon.value;

    /* Set master mode */
    icCon.fields.masterMode = 1;

    /* Configure addressing mode based on device address mode */
    if (dev->addrMode == 1) {
        icCon.fields.ic10bitaddrMaster = 1;
        LOGD("SMBus: 10-bit addressing mode enabled in control register\n");
    } else {
        icCon.fields.ic10bitaddrMaster = 0;
        LOGD("SMBus: 7-bit addressing mode in control register\n");
    }

    /* CRITICAL FIX: Enable Restart for I2C Block Read protocol */
    /* Block Read requires Restart condition for combined write-read transaction */
    icCon.fields.icRestartEn = 1;
    LOGD("SMBus: Restart enabled for Block Read protocol\n");

    ///< Don't override speed setting - preserve existing configuration
    ///< icCon.fields.speed = 1; /* Removed - keep existing speed */

    ///< Don't force slave disable - preserve existing configuration
    ///< icCon.fields.icSlaveDisable = 1; /* Removed - keep existing setting */

    dev->regBase->icCon.value = icCon.value;

    /* Set target address - support both 7-bit and 10-bit addressing */
    icTar.value = 0;

    if (dev->addrMode == 1) {
        /* 10-bit addressing mode */
        icTar.fields.icTar = msgs[msgIdx].addr & 0x3FF; /* 10-bit address */
        icTar.fields.ic10bitaddrMaster = 1;
        LOGD("SMBus: 10-bit addressing enabled, addr=0x%03X\n", msgs[msgIdx].addr);
    } else {
        /* 7-bit addressing mode */
        icTar.fields.icTar = msgs[msgIdx].addr & 0x7F; /* 7-bit address */
        icTar.fields.ic10bitaddrMaster = 0;
        LOGD("SMBus: 7-bit addressing, addr=0x%02X\n", msgs[msgIdx].addr);
    }

    regBase->icTar.value = icTar.value;

    /* Disable interrupts during initialization (similar to I2C) */
    regBase->icIntrMask = 0;

    /* Enable the adapter */
    enable.fields.enable = 1;
    regBase->icEnable.value = enable.value;

    /* Dummy read to avoid register getting stuck */
    dummy = regBase->icEnableStatus;

    /* Clear any pending interrupts */
    dummy = regBase->icClrIntr;
    dummy = dummy;

    /* Enable master interrupts based on work mode - CRITICAL FIX FOR POLLING */
    if (dev->workMode == 1) {
        /* Polling mode - DISABLE all interrupts to let CPU handle data polling */
        regBase->icIntrMask = 0;
        LOGD("SMBus: Polling mode - interrupts disabled for CPU polling\n");
    } else {
        /* Async interrupt mode - enable interrupts */
        U32 interruptMask = SMBUS_IC_INTR_RX_FULL_MASK |      ///< RX FIFO Full
                           SMBUS_IC_INTR_TX_ABRT_MASK |      ///< TX Abort
                           SMBUS_IC_INTR_STOP_DET_MASK |     ///< Stop Detection
                           SMBUS_IC_INTR_TX_EMPTY_MASK |     ///< TX FIFO Empty
                           SMBUS_IC_INTR_ACTIVITY_MASK;      ///< ACTIVE status

        regBase->icIntrMask = interruptMask;
        LOGD("SMBus: Async mode - interrupts enabled (0x%08X)\n", interruptMask);
    }

    /* Log current interrupt mask status */
    U32 currentMask = regBase->icIntrMask;
    LOGD("SMBus: Transfer initialized, addr=0x%02X, mode=%u, intr_mask=0x%08X\n",
         msgs[msgIdx].addr, dev->addrMode, currentMask);

    /* Debug: Check interrupt status after initialization */
    U32 enableStatus;
    enableStatus = regBase->icEnableStatus;
    LOGD("SMBus: Controller enable status=0x%08X, enabled=%d\n",
         enableStatus, 1);

    SmbusIcRawIntrStatReg_u rawIntr = {0};
    rawIntr.value = regBase->icRawIntrStat.value;
    LOGD("SMBus: Raw interrupt status=0x%08X\n", rawIntr.value);

    SmbusIcIntrStatReg_u maskedIntr = {0};
    maskedIntr.value = regBase->icIntrStat.value;
    LOGD("SMBus: Masked interrupt status=0x%08X\n", maskedIntr.value);

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
 * [HAL] Hardware abstraction layer - Error checking
 */
static S32 smbusDwCheckErrors(SmbusDev_s *dev)
{
    SmbusIcTxAbrtSourceReg_u abrtSource;
    volatile SmbusRegMap_s *regBase;

    if (dev == NULL || dev->regBase == NULL) {
        return -EINVAL;
    }

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
 * [HAL] Hardware abstraction layer - Stop detection
 */
static S32 smbusDwCheckStopBit(SmbusDev_s *dev)
{
    U32 timeout = SMBUS_TRANSACTION_TIMEOUT_US;

    if (dev == NULL || dev->regBase == NULL) {
        return -EINVAL;
    }

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

    if (dev == NULL || dev->regBase == NULL || mg == NULL || num == 0) {
        LOGE("SMBus: Invalid parameters for dw xfer\n");
        return (U32)(-EINVAL);
    }

    SmbusMsg_t *msgs = (SmbusMsg_t *)mg;

    ///< Polling mode (original implementation)
    if (dev->workMode == 1) {
        ret = smbusDwXferPoll(dev, msgs, num);
        return (U32)ret;
    }

    ///< Async interrupt mode - similar to I2C implementation
    while(RTEMS_SUCCESSFUL == rtems_semaphore_obtain(dev->semaphoreId, RTEMS_NO_WAIT, 0)) {
        ///< Clear any pending semaphore
        ;
    }

    LOGD("SMBus: BEFORE Waiting for async transfer completion, timeout=%dms\n", dev->transferTimeout);
    ///< Initialize async transfer state
    /* CRITICAL FIX: Copy ALL messages, not just one */
    memcpy(&dev->msgs, msgs, sizeof(SmbusMsg_t) * num);
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
        goto done;
    }

    ///< Start the async transfer
    ret = smbusDwXferInit(dev, msgs, 0);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: Transfer initialization failed: %d\n", ret);
        goto done;
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

        ret = -EIO;
        goto done;
    }

    LOGD("SMBus: Transfer completed - semaphore released by ISR\n");

    ///< Check if controller is still active (similar to I2C issue)
    if (smbusIsControllerActive(dev)) {
        LOGE("SMBus: Controller still active after transfer completion\n");
    }

    ///< Disable controller to prevent spurious interrupts
    SmbusIcEnableReg_u enable;

    enable.value = dev->regBase->icEnable.value;
    enable.fields.enable = 0;
    dev->regBase->icEnable.value = enable.value;

    ///< Handle transfer completion
    if (dev->msgErr) {
        ret = dev->msgErr;
        LOGE("SMBus: Transfer error: %d\n", ret);
        goto done;
    }

    ///< Check for command errors
    if (dev->cmdErr) {
        ret = smbusHandleTxAbort(dev);
        goto done;
    }

    ///< Check for status errors
    if (dev->status & ~SMBUS_STATUS_MASK) {
        LOGE("SMBus: Transfer terminated early - interrupt latency too high?\n");
        ret = -EFAULT;
        goto done;
    }

    ///< Success - return number of messages transferred
    ret = num;
    LOGD("SMBus: Async dw xfer completed successfully\n");

done:
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
 * [HAL] Hardware abstraction layer - polling transfer
 */
static S32 smbusDwXferPoll(SmbusDev_s *dev, SmbusMsg_t *msgs, U32 num)
{
    S32 status = 0;
    U32 msgWrtIdx, msgItrLmt, bufLen;
    U8 *buf;
    U32 val;
    U32 checkErrRetry = 0;

    if (dev == NULL || dev->regBase == NULL || msgs == NULL || num == 0) {
        LOGE("SMBus: Invalid parameters for poll xfer\n");
        return -EINVAL;
    }

    LOGT("SMBus: Starting poll dw xfer, %u messages\n", num);

    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)dev->regBase;

    /* Store message information */
    memcpy(&dev->msgs, msgs, sizeof(SmbusMsg_t));
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
                    } else {
                        LOGE("SMBus: Error detected in read command: %d\n", status);
                        return status;
                    }
                    checkErrRetry++;
                    udelay(10); /* Small delay between retries */
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
                    } else {
                        LOGE("SMBus: Error detected after read: %d\n", status);
                        return status;
                    }
                    checkErrRetry++;
                    udelay(10); /* Small delay between retries */
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
                    udelay(10); /* Small delay between retries */
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

/**
 * @brief Wait for SMBus bus to become not busy
 * @details Checks if the SMBus controller is idle and ready for new transfer.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note This function polls the IC_STATUS register
 * [HAL] Hardware abstraction layer - bus status check
 */
static S32 smbusWaitBusNotBusy(SmbusDev_s *dev)
{
    U32 timeout = 1000;  ///< 1 second timeout
    U32 retries = 0;
    U32 status;

    if (dev == NULL || dev->regBase == NULL) {
        return -EINVAL;
    }

    volatile SmbusRegMap_s *regBase = dev->regBase;

    while (retries < timeout) {
        status = regBase->icStatus.value;
        if ((status & 0x01) == 0) {  ///< Check ACTIVITY bit
            return EXIT_SUCCESS;
        }
        retries++;
        udelay(1000);  ///< 1ms delay
    }

    LOGE("SMBus: Bus busy timeout\n");
    return -ETIMEDOUT;
}

/**
 * @brief Check if SMBus controller is still active
 * @details Checks if the SMBus controller is still active after transfer.
 * @param[in] dev Pointer to SMBus device structure
 * @return true if controller is active, false otherwise
 *
 * @note This function checks the IC_STATUS register
 * [HAL] Hardware abstraction layer - controller status check
 */
static bool smbusIsControllerActive(SmbusDev_s *dev)
{
    if (dev == NULL || dev->regBase == NULL) {
        return false;
    }

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
 * [HAL] Hardware abstraction layer - abort handling
 */
static S32 smbusHandleTxAbort(SmbusDev_s *dev)
{
    if (dev == NULL || dev->regBase == NULL) {
        return -EINVAL;
    }

    volatile SmbusRegMap_s *regBase = dev->regBase;
    U32 abortSource = regBase->icTxAbrtSource.value;

    LOGE("SMBus: TX abort detected, source=0x%08X\n", abortSource);

    ///< Clear abort status
    (void)regBase->icClrTxAbrt;

    if (abortSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_MASK) {
        return -EIO;  ///< Master disabled error
    } else if (abortSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_MASK) {
        return -ENXIO;  ///< No acknowledge error
    } else if (abortSource & SMBUS_IC_TX_ABRT_SOURCE_ABRT_ARB_LOST_MASK) {
        return -EAGAIN;  ///< Arbitration lost error
    }

    return -EIO;  ///< Generic I/O error
}

/**
 * @brief Read and clear interrupt bits (modular function)
 * @details Reads interrupt status register and clears the interrupts
 * @param[in] regBase Register base address
 * @return Raw interrupt status value
 *
 * @note This function combines read and clear operations
 * [CORE] Core protocol layer - interrupt management
 */
U32 smbusReadClearIntrBits(volatile SmbusRegMap_s *regBase)
{
    U32 stat = 0;

    // 读取中断状态
    stat = regBase->icIntrStat.value;

    // 如果使用的是老式调用（兼容性处理），清除所有中断
    LOGD("%s: Legacy call - clearing ALL interrupts: 0x%08X\n", __func__, stat);

    /* Read/clear interrupt status registers */
    (void)regBase->icClrIntr;
    (void)regBase->icClrRxDone;
    (void)regBase->icClrRxUnder;
    (void)regBase->icClrRxOver;
    (void)regBase->icClrTxAbrt;
    (void)regBase->icClrRdReq;
    (void)regBase->icClrTxOver;
    (void)regBase->icClrRestartDet;
    (void)regBase->icClrActivity;
    (void)regBase->icClrStopDet;
    (void)regBase->icClrStartDet;
    (void)regBase->icClrGenCall;

    return stat;
}

/**
 * @brief Read and clear specific interrupts based on mask
 * @details Selectively clears specific interrupts based on mask parameter
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] mask Bitmask of interrupts to clear
 * @return Current interrupt status
 *
 * @note This function follows your recommended logic for selective interrupt clearing
 */
U32 smbusReadClearIntrBitsMasked(SmbusDrvData_s *pDrvData, U32 mask)
{
    volatile SmbusRegMap_s *regBase = pDrvData->sbrCfg.regAddr;
    U32 stat = 0;

    // 如果需要清除所有中断
    if (mask == SMBUS_IC_INTR_ALL) {
        stat = regBase->icClrIntr;  // 读取这个寄存器会清除所有中断
        LOGD("%s: Cleared ALL interrupts: 0x%08X\n", __func__, stat);
        return stat;
    }

    // 读取当前中断状态
    stat = regBase->icIntrStat.value;

    // 根据 mask 清除特定中断
    if (mask & SMBUS_IC_INTR_RX_UNDER_MASK) {
        (void)regBase->icClrRxUnder;
    }
    if (mask & SMBUS_IC_INTR_RX_OVER_MASK) {
        (void)regBase->icClrRxOver;
    }
    if (mask & SMBUS_IC_INTR_RD_REQ_MASK) {
        /* 关键：检查这个寄存器是否存在 */
        (void)regBase->icClrRdReq;  // 如果这里 crash，说明寄存器定义错误
        LOGD("%s: Cleared RD_REQ interrupt\n", __func__);
    }
    if (mask & SMBUS_IC_INTR_TX_ABRT_MASK) {
        (void)regBase->icClrTxAbrt;
    }
    if (mask & SMBUS_IC_INTR_RX_DONE_MASK) {
        (void)regBase->icClrRxDone;
    }
    if (mask & SMBUS_IC_INTR_ACTIVITY_MASK) {
        (void)regBase->icClrActivity;
    }
    if (mask & SMBUS_IC_INTR_STOP_DET_MASK) {
        (void)regBase->icClrStopDet;
    }
    if (mask & SMBUS_IC_INTR_START_DET_MASK) {
        (void)regBase->icClrStartDet;
    }
    if (mask & SMBUS_IC_INTR_GEN_CALL_MASK) {
        (void)regBase->icClrGenCall;
    }
    if (mask & SMBUS_IC_INTR_RESTART_DET_MASK) {
        (void)regBase->icClrRestartDet;
    }

    LOGD("%s: Cleared specific interrupts (mask=0x%08X, stat=0x%08X)\n",
         __func__, mask, stat);

    return regBase->icIntrStat.value;
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

    /* CRITICAL FIX: Add comprehensive safety checks to prevent core dump */
    if (pDrvData == NULL || regBase == NULL) {
        LOGE("SMBus: NULL pointer in smbusMasterReadData\n");
        return;
    }

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
    SmbusMsg_t *msg;
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
        U32 currentMask = regBase->icIntrMask;
        currentMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
        regBase->icIntrMask = currentMask;
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
            U32 currentMask = regBase->icIntrMask;
            currentMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
            regBase->icIntrMask = currentMask;
        }
    } else {
        /* Read operation - not implemented in this context */
        LOGE("SMBus: Read operation not supported in legacy TX handler\n");
        pDrvData->txComplete = 1;
    }
}

/**
 * @brief Trigger slave event callback
 * @details Triggers appropriate SMBus callback for slave events
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] eventType Event type to trigger
 * @param[in] data Event data pointer
 * @param[in] len Data length
 * @return void
 *
 * [CORE] Core protocol layer - slave event handling
 */
void smbusTriggerSlaveEvent(SmbusDrvData_s *pDrvData, U32 eventType, void *data, U32 len)
{
    DevList_e devId = pDrvData->devId;
    SmbusEventData_u eventData;

    /* Prepare event data based on event type */
    switch (eventType) {
        case SMBUS_EVENT_SLAVE_READ_REQ:
            eventData.slaveReadReq.data = (U8*)data;
            eventData.slaveReadReq.len = (U32*)&len;
            break;

        case SMBUS_EVENT_SLAVE_WRITE_REQ:
            eventData.slaveWriteReq.data = (U8*)data;
            eventData.slaveWriteReq.len = len;
            break;

        case SMBUS_EVENT_SLAVE_DONE:
            eventData.slaveStop.val = (U8*)data;
            break;

        default:
            LOGW("SMBus: Unknown slave event type %d\n", eventType);
            return;
    }

    /* Direct callback execution */
    if (pDrvData->callback.cb) {
        LOGD("SMBus: Direct callback for event %d\n", eventType);
        pDrvData->callback.cb(devId, eventType, &eventData);
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
 * [CORE] Core protocol layer - master SMBus interrupts
 */
void smbusHandleMasterSpecificInterrupts(SmbusDrvData_s *pDrvData,
                                         volatile SmbusRegMap_s *regBase,
                                         U32 smbusIntrStat)
{
    DevList_e devId = pDrvData->devId;
    SmbusEventData_u eventData;

    /* Handle SMBus timeout interrupts */
    if (smbusIntrStat & (SMBUS_MST_CLK_EXT_TIMEOUT_BIT | SMBUS_MST_CLK_LOW_TIMEOUT_BIT)) {
        SmbusTimeoutType_e timeoutType = (smbusIntrStat & SMBUS_MST_CLK_EXT_TIMEOUT_BIT) ?
                                       SMBUS_TIMEOUT_TYPE_CLOCK_EXTEND : SMBUS_TIMEOUT_TYPE_CLOCK_LOW;

        /* Handle timeout recovery */
        smbusHandleMasterTimeoutRecovery(pDrvData, timeoutType);

        /* Trigger timeout event callback */
        U32 event = (timeoutType == SMBUS_TIMEOUT_TYPE_CLOCK_EXTEND) ?
                    SMBUS_EVENT_MST_CLK_EXT_TIMEOUT : SMBUS_EVENT_MST_CLK_LOW_TIMEOUT;

        if (pDrvData->callback.cb) {
            pDrvData->callback.cb(devId, event, &eventData);
        }

        LOGW("SMBus Master timeout: type=%d\n", timeoutType);
    }

    /* Clear SMBus interrupts */
    regBase->icClrSmbusIntr = smbusIntrStat;
}

/**
 * @brief Handle slave specific SMBus interrupts
 * @details Processes SMBus-specific interrupts in slave mode
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @param[in] smbusIntrStat SMBus interrupt status
 * @return void
 *
 * [CORE] Core protocol layer - slave SMBus interrupts
 */
void smbusHandleSlaveSpecificInterrupts(SmbusDrvData_s *pDrvData,
                                        volatile SmbusRegMap_s *regBase,
                                        U32 smbusIntrStat)
{
    DevList_e devId = pDrvData->devId;
    SmbusEventData_u eventData;

    /* Handle ARP-related interrupts */
    if (smbusIntrStat & SMBUS_ARP_INTR_BIT) {
        /* Handle ARP assign address completion */
        if (smbusIntrStat & SMBUS_ASSIGN_ADDR_INTR_BIT) {
            smbusSlvArpAssignAddrFinishHandle(pDrvData);
            LOGD("SMBus Slave: ARP assign address finished\n");
        }

        /* Handle other ARP events */
        if (smbusIntrStat & SMBUS_GET_UDID_INTR_BIT) {
            LOGD("SMBus Slave: ARP get UDID interrupt\n");
        }
    }

    /* Handle SMBus timeout interrupts */
    if (smbusIntrStat & (SMBUS_SLV_CLK_EXT_TIMEOUT_BIT | SMBUS_SLV_CLK_LOW_TIMEOUT_BIT)) {
        SmbusTimeoutType_e timeoutType = (smbusIntrStat & SMBUS_SLV_CLK_EXT_TIMEOUT_BIT) ?
                                       SMBUS_TIMEOUT_TYPE_CLOCK_EXTEND : SMBUS_TIMEOUT_TYPE_CLOCK_LOW;

        /* Handle timeout recovery */
        smbusHandleSlaveTimeoutRecovery(pDrvData, timeoutType);

        /* Trigger timeout event callback */
        U32 event = (timeoutType == SMBUS_TIMEOUT_TYPE_CLOCK_EXTEND) ?
                    SMBUS_EVENT_SLV_CLK_EXT_TIMEOUT : SMBUS_EVENT_SLV_CLK_LOW_TIMEOUT;

        if (pDrvData->callback.cb) {
            pDrvData->callback.cb(devId, event, &eventData);
        }
        LOGW("SMBus Slave timeout: type=%d\n", timeoutType);
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
 * [HAL] Hardware abstraction layer - hardware configuration
 */
S32 smbusCalcFifoSize(SmbusDev_s *dev)
{
    U32 param, tx_fifo_depth, rx_fifo_depth;

    if (dev == NULL || dev->regBase == NULL) {
        LOGE("SMBus: Invalid device structure\n");
        return -EINVAL;
    }

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

#if 0
/**
 * @brief Calculate SMBus master timing parameters
 * @details Calculates SCL high/low count and SDA hold time for master mode
 *          based on the configured clock speed. Ported from I2C timing
 *          calculation to ensure proper SMBus timing compliance.
 * @param[in] dev Pointer to SMBus device structure
 * @return void
 *
 * @note Uses hardware-specific timing constants for different speed modes
 * @note Configures SCL counts for standard, fast, fast-plus, and high-speed modes
 * @note Initializes SDA hold time to default value
 * @note Compatible with I2C timing calculation methodology
 * [HAL] Hardware abstraction layer - timing calculation
 */
static void smbusCalcTimingsMaster(SmbusDev_s *dev)
{
    if (dev == NULL) {
        return;
    }

    /* Initialize all timing counters to zero */
    dev->ssHcnt = 0;
    dev->ssLcnt = 0;
    dev->fsHcnt = 0;
    dev->fsLcnt = 0;
    dev->hsHcnt = 0;
    dev->hsLcnt = 0;
    dev->fpHcnt = 0;
    dev->fpLcnt = 0;

    /* Calculate timing parameters based on clock rate */
    switch (dev->clkRate) {
        case SMBUS_MAX_STANDARD_MODE_FREQ:
            /* Standard mode (100 kHz) */
            dev->ssHcnt = SMBUS_HW_CLK_H100;
            dev->ssLcnt = SMBUS_HW_CLK_L100;
            break;

        case SMBUS_MAX_FAST_MODE_FREQ:
            /* Fast mode (400 kHz) */
            dev->fsHcnt = SMBUS_HW_CLK_H400;
            dev->fsLcnt = SMBUS_HW_CLK_L400;
            break;

        case SMBUS_MAX_FAST_MODE_PLUS_FREQ:
            /* Fast mode plus (1 MHz) */
            dev->fpHcnt = SMBUS_HW_CLK_H1000;
            dev->fpLcnt = SMBUS_HW_CLK_L1000;
            /* Also configure fast mode as fallback */
            dev->fsHcnt = SMBUS_HW_CLK_H400;
            dev->fsLcnt = SMBUS_HW_CLK_L400;
            break;

        case SMBUS_MAX_HIGH_SPEED_MODE_FREQ:
            /* High speed mode (3.4 MHz) */
            dev->hsHcnt = SMBUS_HW_CLK_H3400;
            dev->hsLcnt = SMBUS_HW_CLK_L3400;
            /* Also configure fast mode for compatibility */
            dev->fsHcnt = SMBUS_HW_CLK_H400;
            dev->fsLcnt = SMBUS_HW_CLK_L400;
            break;

        default:
            /* Unknown speed, use fast mode as default */
            dev->fsHcnt = SMBUS_HW_CLK_H400;
            dev->fsLcnt = SMBUS_HW_CLK_L400;
            LOGW("SMBus: Unknown clock rate %u Hz, using fast mode timing\n", dev->clkRate);
            break;
    }

    /* Initialize SDA hold time to default value (same as I2C) */
    dev->sdaHoldTime = 0;  /* Use hardware default */

    /* Initialize spike suppression lengths to defaults */
    dev->fsSpklen = 0;  /* Use hardware default */
    dev->hsSpklen = 0;  /* Use hardware default */

    LOGD("SMBus: Timing calculated - SS:%u/%u, FS:%u/%u, FP:%u/%u, HS:%u/%u\n",
         dev->ssHcnt, dev->ssLcnt, dev->fsHcnt, dev->fsLcnt,
         dev->fpHcnt, dev->fpLcnt, dev->hsHcnt, dev->hsLcnt);
}
#endif

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
 * [HAL] Hardware abstraction layer - timing calculation
 */
static void smbusCalcTimingsMaster(SmbusDev_s *dev)
{
    U32 ic_clk;  /* Internal peripheral clock frequency (12.5MHz) */
    
    if (dev == NULL) {
        return;
    }
    
    /* Initialize all timing counters to zero */
    dev->ssHcnt = 0;
    dev->ssLcnt = 0;
    dev->fsHcnt = 0;
    dev->fsLcnt = 0;
    dev->hsHcnt = 0;
    dev->hsLcnt = 0;
    dev->fpHcnt = 0;
    dev->fpLcnt = 0;
    
    /* Calculate timing parameters based on clock rate */
    switch (dev->clkRate) {
        case SMBUS_MAX_STANDARD_MODE_FREQ:
            /* Standard mode (100 kHz) */
            dev->ssHcnt = SMBUS_HW_CLK_H100;
            dev->ssLcnt = SMBUS_HW_CLK_L100;
            break;
            
        case SMBUS_MAX_FAST_MODE_FREQ:
            /* Fast mode (400 kHz) */
            dev->fsHcnt = SMBUS_HW_CLK_H400;
            dev->fsLcnt = SMBUS_HW_CLK_L400;
            break;
            
        case SMBUS_MAX_FAST_MODE_PLUS_FREQ:
            /* Fast mode plus (1 MHz) */
            dev->fpHcnt = SMBUS_HW_CLK_H1000;
            dev->fpLcnt = SMBUS_HW_CLK_L1000;
            /* Also configure fast mode as fallback */
            dev->fsHcnt = SMBUS_HW_CLK_H400;
            dev->fsLcnt = SMBUS_HW_CLK_L400;
            break;
            
        case SMBUS_MAX_HIGH_SPEED_MODE_FREQ:
            /* High speed mode (3.4 MHz) */
            dev->hsHcnt = SMBUS_HW_CLK_H3400;
            dev->hsLcnt = SMBUS_HW_CLK_L3400;
            /* Also configure fast mode for compatibility */
            dev->fsHcnt = SMBUS_HW_CLK_H400;
            dev->fsLcnt = SMBUS_HW_CLK_L400;
            break;
            
        default:
            /* Unknown speed, use fast mode as default */
            dev->fsHcnt = SMBUS_HW_CLK_H400;
            dev->fsLcnt = SMBUS_HW_CLK_L400;
            LOGW("SMBus: Unknown clock rate %u Hz, using fast mode timing\n", dev->clkRate);
            break;
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
    #define SMBUS_IC_CLK_FREQ (12500000)
    ic_clk = SMBUS_IC_CLK_FREQ;  /* 12.5MHz = 12500000 Hz */
    
    if (ic_clk > 0 && dev->clkRate > 0) {
        /* 
         * Formula: sdaHoldTime = (0.15 × ic_clk) / (2 × speed)
         * Use fixed-point: (15 × ic_clk) / (200 × speed) to avoid float
         * Add rounding: + (100 × speed) before division
         */
        dev->sdaHoldTime = (15 * ic_clk + 100 * dev->clkRate) / (200 * dev->clkRate);
        
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
    
    /* Initialize spike suppression lengths to defaults */
    dev->fsSpklen = 0;  /* Use hardware default */
    dev->hsSpklen = 0;  /* Use hardware default */
    
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
 * [HAL] Hardware abstraction layer - master configuration
 */
static void smbusConfigureMaster(SmbusDev_s *dev)
{
    U32 masterCfg = 0;

    if (dev == NULL || dev->regBase == NULL) {
        return;
    }

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
 * [HAL] Hardware abstraction layer - slave configuration
 */
static void smbusConfigureSlave(SmbusDev_s *dev)
{
    U32 slaveCfg = 0;

    if (dev == NULL || dev->regBase == NULL) {
        return;
    }

    /* Configure slave mode - compatible with smbusProbeSlave settings */
    slaveCfg &= ~(1U << SMBUS_IC_CON_MASTER_MODE_EN_BIT);  /* Master mode disable */
    slaveCfg &= ~(1U << SMBUS_IC_CON_SLAVE_DISABLE_BIT);   /* Slave enable */
    slaveCfg |= (1U << SMBUS_IC_CON_RESTART_EN_BIT);      /* Restart enable for combined transactions */
    slaveCfg |= SMBUS_SPEED_FAST_CFG;                     /* Fast mode by default */

    /* Set address mode */
    if (dev->addrMode == 1) {
        slaveCfg |= (1U << SMBUS_IC_CON_10BIT_SLAVE_ADDR_BIT);  /* 10-bit slave addressing */
    }

    /* Configure SMBus features for slave mode */
    slaveCfg |= (1U << SMBUS_IC_CON_RX_FIFO_HOLD_BIT);         /* RX FIFO full hold control */
    slaveCfg |= (1U << SMBUS_IC_CON_STOP_DET_IFADDRESSED_BIT); /* STOP detection when addressed */
    slaveCfg |= (1U << SMBUS_IC_CON_ARP_ENABLE_BIT);           /* ARP enabled in slave mode */
    slaveCfg |= (1U << SMBUS_IC_CON_QUICK_CMD_BIT);            /* SMBus slave quick command enable */

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
 * [HAL] Hardware abstraction layer - master initialization
 */
S32 smbusProbeMaster(SmbusDev_s *dev)
{
    S32 ret;
    U32 ic_con;

    if (dev == NULL || dev->regBase == NULL) {
        LOGE("SMBus: Invalid device structure\n");
        return -EINVAL;
    }
    
    /* Step 1: Configure master mode parameters (like i2c_dw_configure_master) */
    smbusConfigureMaster(dev);

    /* Step 2: Calculate FIFO size (like i2c_dw_calc_fifo_size) */
    ret = smbusCalcFifoSize(dev);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: FIFO size calculation failed: %d\n", ret);
        return ret;
    }
    
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
    ic_con = dev->regBase->icCon.value;
    ic_con &= ~(1U << 14);  /* ARP disabled by default in master mode */
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
    regBase->icIntrMask = 0;

    /* Step 8: Enable controller */
    SmbusIcEnableReg_u enable;
    enable.value = 0;
    enable.fields.enable = 1;
    regBase->icEnable.value = enable.value;

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
 * [HAL] Hardware abstraction layer - slave initialization
 */
S32 smbusProbeSlave(SmbusDev_s *dev)
{
    S32 ret;
    U32 ic_con;

    if (dev == NULL || dev->regBase == NULL) {
        LOGE("SMBus: Invalid device structure\n");
        return -EINVAL;
    }

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

        /* 禁用设备 */
        regBase->icEnable.value = 0;
        LOGD("Device disabled (IC_ENABLE = 0x%08X)\n", regBase->icEnable.value);

        /* 配置 IC_CON */
        U32 icCon = 0;
        icCon |= (0 << 0);  // MASTER_MODE = 0 (Slave mode)
        icCon |= (2 << 1);  // SPEED = 2 (Fast mode 400kHz)
        icCon |= (0 << 6);  // IC_SLAVE_DISABLE = 0 (Slave enabled)

        /* Set address mode */
        if (dev->addrMode == 1) {
            icCon |= (1U << 3);  /* 10-bit slave addressing */
        }

        /* Configure SMBus features for slave mode */
        icCon |= (1U << 5);   // Restart enable for combined transactions
        icCon |= (1U << 14);  // ARP enabled in slave mode
        icCon |= (1U << 15);  // SMBus slave quick command enable

        regBase->icCon.value = icCon;
        LOGD("IC_CON configured = 0x%08X\n", regBase->icCon.value);

        /* 设置从机地址 */
        regBase->icSar.fields.icSar = 0x21;  /* 直接设置为0x21 */
        LOGD("IC_SAR set to 0x%02X\n", regBase->icSar.fields.icSar);

        /* Set TX/RX FIFO thresholds for interrupt generation */
        regBase->icTxTl = 0;  /* TX threshold: generate interrupt when TX FIFO is empty */
        regBase->icRxTl = 0;  /* RX threshold: generate interrupt when RX FIFO has 1+ bytes */
        LOGD("FIFO thresholds set: TX_TL=%d, RX_TL=%d\n", regBase->icTxTl, regBase->icRxTl);

        /* 关键：使能设备时包含 SMBus 扩展位 */
        U32 icEnable = 0;
        icEnable |= (1 << 0);   // ENABLE
        icEnable |= (1 << 19);  // 保留你原来的 bit[19]（可能是 SMBus Slave TX Enable）

        regBase->icEnable.value = icEnable;
        LOGI("Slave IC_ENABLE = 0x%08X\n", regBase->icEnable.value);

        /* 配置中断掩码 - 包含 TX_EMPTY 处理时序问题 */
        U32 slaveMask = 0;
        slaveMask |= (1 << 2);  // RX_FULL   - 0x004
        slaveMask |= (1 << 4);  // TX_EMPTY  - 0x010 (启用处理RD_REQ时序问题)
        slaveMask |= (1 << 5);  // RD_REQ    - 0x020
        slaveMask |= (1 << 6);  // TX_ABRT   - 0x040
        slaveMask |= (1 << 7);  // RX_DONE   - 0x080
        slaveMask |= (1 << 9);  // STOP_DET  - 0x200

        /* 计算结果：0x004 | 0x010 | 0x020 | 0x040 | 0x080 | 0x200 = 0x2F4 */
        regBase->icIntrMask = slaveMask;
        LOGI("✓ Slave interrupt mask configured with TX_EMPTY: 0x%08X (expected: 0x2F4)\n",
             regBase->icIntrMask);

        /* 验证配置是否成功 */
        udelay(100);  /* 短暂延时确保硬件稳定 */

        LOGD("=== Slave Initialization Verification ===\n");
        LOGD("Final IC_CON    = 0x%08X\n", regBase->icCon.value);
        LOGD("Final IC_SAR    = 0x%02X\n", regBase->icSar.fields.icSar);
        LOGD("Final IC_ENABLE = 0x%08X\n", regBase->icEnable.value);
        LOGD("Final IC_INTR_MASK = 0x%08X\n", regBase->icIntrMask);
        LOGD("Final IC_TX_TL  = %d\n", regBase->icTxTl);
        LOGD("Final IC_RX_TL  = %d\n", regBase->icRxTl);

        /* 验证写入是否成功 */
        U32 readBack = regBase->icIntrMask;
        if (readBack != slaveMask) {
            LOGE("✗ Interrupt mask write failed! Read back: 0x%08X\n", readBack);
        } else {
            LOGI("✓ All slave configurations verified successfully\n");
        }
    } else {
        /* Fallback to original logic for non-slave modes */
        ic_con = regBase->icCon.value;
        ic_con &= ~(1U << 0);  /* Master mode disable */
        ic_con &= ~(1U << 6);  /* Slave enable */
        ic_con |= (1U << 5);   /* Restart enable for combined transactions */
        ic_con |= (2U << 1);   /* Fast mode by default */

        /* Set address mode */
        if (dev->addrMode == 1) {
            ic_con |= (1U << 3);  /* 10-bit slave addressing */
        }

        /* Configure SMBus features for slave mode */
        ic_con |= (1U << 14);  /* ARP enabled in slave mode */
        ic_con |= (1U << 15);  /* SMBus slave quick command enable */

        regBase->icCon.value = ic_con;

        /* Set slave address using original method */
        ret = smbusSetSlaveAddr(dev);
        if (ret != EXIT_SUCCESS) {
            LOGE("SMBus: Slave address configuration failed: %d\n", ret);
            return ret;
        }

        /* Set TX/RX FIFO thresholds for interrupt generation */
        regBase->icTxTl = 0;  /* TX threshold: generate interrupt when TX FIFO is empty */
        regBase->icRxTl = 0;  /* RX threshold: generate interrupt when RX FIFO has 1+ bytes */
    }

    /* For non-slave modes or fallback, configure remaining settings */
    if (dev->mode != SMBUS_MODE_SLAVE) {
        /* Slave mode interrupt configuration - configure correct mask */
        /* 关键：必须在这里设置正确的中断掩码，包含TX_EMPTY */
        U32 slaveMask = 0;

        slaveMask |= (1 << 2);  // RX_FULL   - 0x004
        slaveMask |= (1 << 4);  // TX_EMPTY  - 0x010 (启用处理RD_REQ时序问题)
        slaveMask |= (1 << 5);  // RD_REQ    - 0x020
        slaveMask |= (1 << 6);  // TX_ABRT   - 0x040
        slaveMask |= (1 << 7);  // RX_DONE   - 0x080
        slaveMask |= (1 << 9);  // STOP_DET  - 0x200

        /* 计算结果：0x004 | 0x010 | 0x020 | 0x040 | 0x080 | 0x200 = 0x2F4 */

        /* 写入寄存器 */
        regBase->icIntrMask = slaveMask;

        LOGI("Slave interrupt mask configured: 0x%08X (expected: 0x2F4)\n",
             regBase->icIntrMask);

        /* 验证写入是否成功 */
        U32 readBack = regBase->icIntrMask;
        if (readBack != slaveMask) {
            LOGE("Interrupt mask write failed! Read back: 0x%08X\n", readBack);
        }

        /* Enable controller using original method */
        smbusEnable(dev);
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
 * [HAL] Hardware abstraction layer - master cleanup
 */
S32 smbusUnprobeMaster(SmbusDev_s *dev)
{
    if (dev == NULL || dev->regBase == NULL) {
        LOGE("SMBus: Invalid device structure\n");
        return -EINVAL;
    }

    /* Disable controller */
    smbusDisable(dev);

    /* Clear master configuration */
    dev->masterCfg = 0;

    /* Remove synchronization objects if they exist */
    if (dev->semaphoreId != 0) {
        rtems_status_code status = rtems_semaphore_delete(dev->semaphoreId);
        if (status != RTEMS_SUCCESSFUL) {
            LOGE("SMBus: Failed to delete master semaphore %u: %u\n",
                 dev->semaphoreId, status);
        }
        dev->semaphoreId = 0;
    }

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
 * [HAL] Hardware abstraction layer - slave cleanup
 */
S32 smbusUnprobeSlave(SmbusDev_s *dev)
{
    if (dev == NULL || dev->regBase == NULL) {
        LOGE("SMBus: Invalid device structure\n");
        return -EINVAL;
    }

    /* Disable controller */
    smbusDisable(dev);

    /* Clear slave configuration */
    dev->slaveCfg = 0;

#if 0
    /* Remove interrupt handler */
    if (dev->irq != 0) {
        rtems_status_code status = rtems_interrupt_handler_remove(dev->irq, dev->isrHandler, dev);
        if (status != RTEMS_SUCCESSFUL) {
            LOGE("SMBus: Failed to remove slave interrupt handler %u: %u\n",
                 dev->irq, status);
        }
    }
#endif

    /* Note: Slave buffer deallocation is handled in smbusDeInit function */

    LOGD("SMBus: Slave unprobe completed successfully\n");
    return EXIT_SUCCESS;
}

/* ======================================================================== */
/*                    HAL Layer - Host Notify Protocol                        */
/* ======================================================================== */

/**
 * @brief SMBus Host Notify core function
 * @details Implements the SMBus Host Notify protocol at the HAL layer.
 *          This function sends a Host Notify command to notify the host
 *          about device status changes using the SMBus Host Notify protocol.
 * @param[in] regBase Pointer to SMBus register map base address
 * @param[in] data Pointer to Host Notify data structure containing device address and status data
 * @return EXIT_SUCCESS on successful transmission, negative error code on failure:
 *         -EINVAL: Invalid parameters (NULL regBase or data)
 *         -EBUSY: Bus is busy or controller is not ready
 *         -EIO: Transmission failed or timeout occurred
 *         -EPROTO: Protocol error during communication
 *
 * @note Implements SMBus Host Notify protocol according to specification
 * @note Uses SMBus Host Notify fixed address (0x08) for communication
 * @note Sends device address followed by 16-bit status data
 * @note Handles bus idle checking and transmission completion
 * @note Configures controller for Host Notify operation
 * @warning This function should only be called in Master mode
 * @warning Caller must ensure proper device locking before calling this function
 * [HAL] Hardware abstraction layer - Host Notify protocol operations
 */
static S32 smbusHostNotifyCore(volatile SmbusRegMap_s *regBase, SmbusHostNotifyData_s *data)
{
    S32 ret = EXIT_SUCCESS;
    SmbusIcEnableReg_u enable;
    SmbusDev_s tempDev;

    /* Parameter validation */
    if (regBase == NULL || data == NULL) {
        LOGE("SMBus: Invalid parameters for Host Notify\n");
        return -EINVAL;
    }

    /* Create temporary device structure for utility functions */
    memset(&tempDev, 0, sizeof(SmbusDev_s));
    tempDev.regBase = (volatile SmbusRegMap_s *)regBase;

    /* Step 1: Wait for bus to be idle */
    ret = smbusWaitBusNotBusy(&tempDev);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: Bus busy, Host Notify failed\n");
        return -EBUSY;
    }

    /* Step 2: Set target address to SMBus Host Notify fixed address (0x08) */
    SmbusIcTarReg_u tar;
    tar.value = 0;
    tar.fields.icTar = SMBUS_HOST_NOTIFY_ADDR;
    regBase->icTar.value = tar.value;

    /* Step 3: Enable SMBus controller */
    enable.value = 0;
    enable.fields.enable = 1;
    regBase->icEnable.value = enable.value;

    /* Step 4: Send device address (identify) */
    SmbusIcDataCmdReg_u dataCmd;
    dataCmd.value = 0;
    dataCmd.fields.dat = data->deviceAddr;
    dataCmd.fields.cmd = 0;  /* Write command */
    dataCmd.fields.stop = 0; /* Don't send stop yet */
    dataCmd.fields.restart = 0;

    regBase->icDataCmd.value = dataCmd.value;

    /* Wait for TX ready */
    ret = smbusCheckTxready(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: TX ready timeout for device address\n");
        goto error;
    }

    /* Step 5: Send 16-bit status data (low byte first) */
    /* Send low byte */
    dataCmd.fields.dat = (data->data & 0xFF);
    dataCmd.fields.stop = 0; /* Don't send stop yet */

    regBase->icDataCmd.value = dataCmd.value;

    /* Wait for TX ready */
    ret = smbusCheckTxready(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: TX ready timeout for status low byte\n");
        goto error;
    }

    /* Send high byte with STOP condition */
    dataCmd.fields.dat = ((data->data >> 8) & 0xFF);
    dataCmd.fields.stop = 1; /* Send STOP after this byte */

    regBase->icDataCmd.value = dataCmd.value;

    /* Step 6: Wait for transmission to complete */
    ret = waitTransmitComplete(regBase);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: Host Notify transmission failed\n");
        goto error;
    }

    LOGD("SMBus: Host Notify sent successfully for device 0x%02X, data 0x%04X\n",
         data->deviceAddr, data->data);
    return EXIT_SUCCESS;

error:
    /* Error handling: Send stop signal and disable controller */
    LOGE("SMBus: Host Notify failed, cleaning up\n");
    enable.value = regBase->icEnable.value;
    enable.fields.enable = 0;
    regBase->icEnable.value = enable.value;
    return ret;
}

/* ======================================================================== */
/*                    HAL Layer - Mode Switching                              */
/* ======================================================================== */

/**
 * @brief SMBus Master/Slave mode switch core function
 * @details Implements the core logic for switching between Master and Slave modes
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
 * [HAL] Hardware abstraction layer - mode switching operations
 */
static S32 smbusModeSwitchCore(SmbusDev_s *dev, SmbusMode_e targetMode)
{
    S32 ret = EXIT_SUCCESS;
    U32 speed_val, ti2c_poll, enable_sts;
    U32 max_t_poll_count;
    U32 i;

    /* Parameter validation */
    if (dev == NULL || dev->regBase == NULL) {
        LOGE("SMBus: Invalid device structure for mode switch\n");
        return -EINVAL;
    }

    if (targetMode != DW_SMBUS_MODE_MASTER && targetMode != DW_SMBUS_MODE_SLAVE) {
        LOGE("SMBus: Invalid target mode %d for mode switch\n", targetMode);
        return -EINVAL;
    }

    /* Check if already in target mode */
    if (dev->mode == targetMode) {
        LOGD("SMBus: Already in %s mode, no switch needed\n",
             targetMode == DW_SMBUS_MODE_MASTER ? "master" : "slave");
        return EXIT_SUCCESS;
    }

    LOGD("SMBus: Switching from %s to %s mode\n",
         dev->mode == DW_SMBUS_MODE_MASTER ? "master" : "slave",
         targetMode == DW_SMBUS_MODE_MASTER ? "master" : "slave");

    /* Step 1: Wait for bus to be idle before disable */
    ret = smbusWaitBusNotBusy(dev);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: Bus busy during mode switch\n");
        return -EBUSY;
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
            return -EINVAL;
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
        return -ETIMEDOUT;
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
    } else {
        /* Switch to slave mode */
        smbusConfigureSlave(dev);

        /* Apply slave configuration to hardware */
        dev->regBase->icCon.value = dev->slaveCfg;

        /* Set slave address for slave mode */
        ret = smbusSetSlaveAddr(dev);
        if (ret != EXIT_SUCCESS) {
            LOGE("SMBus: Failed to set slave address during mode switch: %d\n", ret);
            return ret;
        }

        LOGD("SMBus: Configured for slave mode, cfg=0x%08X, addr=0x%02X\n",
             dev->slaveCfg, dev->slaveAddr);
    }

    /* Step 8: Re-enable controller */
    smbusEnable(dev);
    /* Step 9: Configure proper interrupt mask for the target mode */
    if (targetMode == DW_SMBUS_MODE_MASTER) {
        /* Master mode: Enable interrupts needed for master operations */
        dev->regBase->icIntrMask = SMBUS_IC_INTR_TX_ABRT_MASK |      /* TX abort */
                                  SMBUS_IC_INTR_STOP_DET_MASK |     /* Stop detection */
                                  SMBUS_IC_INTR_ACTIVITY_MASK;      /* Activity detection */
        LOGD("SMBus: Set Master mode interrupt mask=0x%08X\n", dev->regBase->icIntrMask);
    } else {
        /* Slave mode: Enable only interrupts needed for slave operations */
        /* CRITICAL FIX: Use the correct interrupt mask with TX_EMPTY to handle timing issues */
        U32 slaveMask = SMBUS_IC_INTR_RX_FULL_MASK |      /* RX FIFO full - 0x004 */
                          SMBUS_IC_INTR_TX_EMPTY_MASK |     /* TX FIFO empty - 0x010 (handle timing) */
                          SMBUS_IC_INTR_RD_REQ_MASK |       /* Read request - 0x020 */
                          SMBUS_IC_INTR_TX_ABRT_MASK |      /* TX abort - 0x040 */
                          SMBUS_IC_INTR_RX_DONE_MASK |      /* RX done - 0x080 */
                          SMBUS_IC_INTR_STOP_DET_MASK;     /* Stop detection - 0x200 */

        /* Calculate: 0x004 | 0x010 | 0x020 | 0x040 | 0x080 | 0x200 = 0x2F4 */
        dev->regBase->icIntrMask = slaveMask;

        LOGI("✓ Slave mode interrupt mask set: 0x%08X (expected: 0x2F4)\n", dev->regBase->icIntrMask);

        /* Verify the write was successful */
        U32 readBack = dev->regBase->icIntrMask;
        if (readBack != slaveMask) {
            LOGE("✗ Slave interrupt mask write failed! Expected: 0x%08X, Read: 0x%08X\n", slaveMask, readBack);
        }
    }

    LOGI("SMBus: Successfully switched to %s mode\n",
         targetMode == DW_SMBUS_MODE_MASTER ? "master" : "slave");

    return EXIT_SUCCESS;
}

/* ======================================================================== */
/*                    HAL Layer - I2C Compatibility                           */
/* ======================================================================== */

/**
 * @brief I2C write operation for SMBus HAL layer
 * @details Implements I2C-compatible write operation at the HAL layer.
 *          This function provides direct I2C write capability without
 *          SMBus protocol overhead, enabling compatibility with I2C devices.
 * @param[in] dev Pointer to SMBus device structure
 * @param[in] slaveAddr 7-bit or 10-bit slave address
 * @param[in] dataBuf Pointer to data buffer to write
 * @param[in] length Number of bytes to write
 * @return EXIT_SUCCESS on successful write, negative error code on failure:
 *         -EINVAL: Invalid parameters (NULL dev or dataBuf, zero length)
 *         -EBUSY: Bus is busy or controller is not ready
 *         -EIO: I/O error during communication (NACK, timeout, bus error)
 *         -ETIMEDOUT: Transfer timeout
 *
 * @note Configures controller for master mode operation
 * @note Handles both 7-bit and 10-bit addressing
 * @note Performs basic I2C write without SMBus protocol overhead
 * @note Waits for bus idle before starting transfer
 * @note Configures appropriate timing based on device settings
 * @warning This function should only be called in Master mode
 * @warning Caller must ensure proper device locking before calling this function
 * [HAL] Hardware abstraction layer - I2C compatibility operations
 */
static S32 smbusI2cWrite(SmbusDev_s *dev, U16 slaveAddr, const U8 *dataBuf, U32 length)
{
    S32 ret = EXIT_SUCCESS;
    SmbusMsg_t msg;
    U32 msgCount = 1;

    /* Parameter validation */
    if (dev == NULL || dev->regBase == NULL || dataBuf == NULL) {
        LOGE("SMBus: Invalid parameters for I2C write\n");
        ret = -EINVAL;
        goto exit;
    }

    if (length == 0) {
        LOGE("SMBus: Zero length for I2C write\n");
        ret = -EINVAL;
        goto exit;
    }

    if (length > SMBUS_MAX_BLOCK_LEN) {
        LOGE("SMBus: Length too large for I2C write: %u > %u\n", length, SMBUS_MAX_BLOCK_LEN);
        ret = -EINVAL;
        goto exit;
    }

    /* Check device mode and handle accordingly */
    if (dev->mode == SMBUS_MODE_MASTER) {
        /* Master mode implementation */
        LOGD("SMBus: Master mode I2C write to addr=0x%02X, len=%u\n", slaveAddr, length);

        /* Additional validation: check if semaphore is valid */
        if (dev->semaphoreId == 0) {
            LOGE("SMBus: Invalid semaphoreId (0) - device not properly initialized\n");
            LOGE("SMBus: Device dump - dev=%p, channelNum=%d, workMode=%d, enabled=%d\n",
                 (void*)dev, dev->channelNum, dev->workMode, dev->enabled);
            LOGE("SMBus: Device structure size = %zu bytes\n", sizeof(SmbusDev_s));
            ret = -EINVAL;
            goto exit;
        }

        /* Configure message for I2C write */
        msg.addr = slaveAddr;
        msg.flags = 0;  /* Write operation */
        msg.len = length;
        msg.buf = (U8 *)dataBuf;  /* Remove const for compatibility */

        /* Wait for bus to be idle */
        ret = smbusWaitBusNotBusy(dev);
        if (ret != EXIT_SUCCESS) {
            LOGE("SMBus: Bus busy, I2C write failed\n");
            ret = -EBUSY;
            goto exit;
        }

        /* Use existing SMBus transfer functionality for I2C compatibility */
        ret = smbusDwXfer(dev, &msg, msgCount);
        if (ret < 0) {
            LOGE("SMBus: I2C write transfer failed, ret=%d\n", ret);
            ret = -EIO;
            goto exit;
        }

        LOGD("SMBus: Master I2C write completed successfully, addr=0x%02X, len=%d\n", slaveAddr, length);
    } else {
        /* Slave mode implementation - based on I2C driver pattern */
        LOGD("SMBus: Slave mode I2C write: addr=0x%02X, len=%u\n", slaveAddr, length);

        if (dev->workMode == 0) { /* Interrupt mode */
            /* Validate slave buffers are available */
            if (dev->slaveTxBuf == NULL) {
                LOGE("SMBus: Slave TX buffer not allocated\n");
                ret = -EINVAL;
                goto exit;
            }

            /* Clear slave TX buffer */
            memset(dev->slaveTxBuf, 0, SMBUS_MAX_BLOCK_LEN);

            /* Copy data to slave TX buffer */
            memcpy(dev->slaveTxBuf, (U8 *)dataBuf, length);

            /* Set slave TX parameters */
            dev->slaveValidTxLen = length;

            LOGD("SMBus: Slave mode configured %u bytes for transmission\n", length);

            ret = 0; /* Return immediately, actual success determined in ISR */
        } else {
            LOGE("SMBus: Slave polling mode not implemented\n");
            ret = -ENOTSUP;
            goto exit;
        }
    }

exit:
    return ret;
}

/**
 * @brief I2C read operation for SMBus HAL layer
 * @details Implements I2C-compatible read operation at the HAL layer.
 *          This function provides direct I2C read capability without
 *          SMBus protocol overhead, enabling compatibility with I2C devices.
 * @param[in] dev Pointer to SMBus device structure
 * @param[in] slaveAddr 7-bit or 10-bit slave address
 * @param[in,out] dataBuf Pointer to buffer to store read data
 * @param[in] length Number of bytes to read
 * @return EXIT_SUCCESS on successful read, negative error code on failure:
 *         -EINVAL: Invalid parameters (NULL dev or dataBuf, zero length)
 *         -EBUSY: Bus is busy or controller is not ready
 *         -EIO: I/O error during communication (NACK, timeout, bus error)
 *         -ETIMEDOUT: Transfer timeout
 *
 * @note Configures controller for master mode operation
 * @note Handles both 7-bit and 10-bit addressing
 * @note Performs basic I2C read without SMBus protocol overhead
 * @note Waits for bus idle before starting transfer
 * @note Configures appropriate timing based on device settings
 * @warning This function should only be called in Master mode
 * @warning Caller must ensure proper device locking before calling this function
 * [HAL] Hardware abstraction layer - I2C compatibility operations
 */
static S32 smbusI2cRead(SmbusDev_s *dev, U16 slaveAddr, U8 *dataBuf, U32 length)
{
    S32 ret = EXIT_SUCCESS;

    /* Parameter validation */
    if (dev == NULL || dev->regBase == NULL || dataBuf == NULL) {
        LOGE("SMBus: Invalid parameters for I2C read\n");
        ret = -EINVAL;
        goto exit;
    }

    if (length == 0) {
        LOGE("SMBus: Zero length for I2C read\n");
        ret = -EINVAL;
        goto exit;
    }

    /* ===== Mode-specific handling ===== */
    if (dev->mode == SMBUS_MODE_MASTER) {
        /* ===== MASTER MODE IMPLEMENTATION ===== */
        SmbusMsg_t msg;
        U32 msgCount = 1;

        /* Additional validation: check if semaphore is valid */
        if (dev->semaphoreId == 0) {
            LOGE("SMBus: Invalid semaphoreId (0) - device not properly initialized\n");
            LOGE("SMBus: Device dump - dev=%p, channelNum=%d, workMode=%d, enabled=%d\n",
                 (void*)dev, dev->channelNum, dev->workMode, dev->enabled);
            LOGE("SMBus: Device structure size = %zu bytes\n", sizeof(SmbusDev_s));
            ret = -EINVAL;
            goto exit;
        }

        /* Configure message for I2C read */
        msg.addr = slaveAddr;
        msg.flags = 1;  /* Read operation */
        msg.len = length;
        msg.buf = dataBuf;

        /* Wait for bus to be idle */
        ret = smbusWaitBusNotBusy(dev);
        if (ret != EXIT_SUCCESS) {
            LOGE("SMBus: Bus busy, I2C read failed\n");
            ret = -EBUSY;
            goto exit;
        }

        /* Use existing SMBus transfer functionality for I2C compatibility */
        ret = smbusDwXfer(dev, &msg, msgCount);
        if (ret < 0) {
            LOGE("SMBus: I2C read transfer failed, ret=%d\n", ret);
            ret = -EIO;
            goto exit;
        }

        LOGD("SMBus: I2C MASTER read completed successfully, addr=0x%02X, len=%d\n", slaveAddr, length);
        LOGD("SMBus: I2C MASTER read data: ");
        for (U32 i = 0; i < length && i < 8; i++) {  /* Log first 8 bytes */
            LOGD("0x%02X ", dataBuf[i]);
        }
        LOGD("\n");

    } else {
        /* ===== SLAVE MODE IMPLEMENTATION ===== */
        LOGD("SMBus: I2C SLAVE read operation\n");

        /* Note: In Slave mode, we read from the receive buffer that was filled by ISR */
        /* Based on I2C driver implementation pattern */

        /* For Slave mode, we need to handle interrupt-based read operations */
        if (dev->workMode == 0) {  /* Interrupt mode */
            /* Check if we have valid data in the slave RX buffer */
            if (dev->slaveValidRxLen == 0) {
                LOGD("SMBus: SLAVE mode - no data available in RX buffer (scan scenario)\n");
                ret = 0;  /* Scan scenario - return 0 similar to I2C implementation */
                goto exit;
            } else {
                /* Copy data from slave RX buffer to user buffer */
                U32 copyLen = (length < dev->slaveValidRxLen) ? length : dev->slaveValidRxLen;

                LOGD("SMBus: SLAVE mode - copying %u bytes from RX buffer (available: %u)\n",
                     copyLen, dev->slaveValidRxLen);

                /* Safety check: ensure slaveRxBuf is not NULL */
                if (dev->slaveRxBuf == NULL) {
                    LOGE("SMBus: SLAVE mode - slaveRxBuf is NULL\n");
                    ret = -EINVAL;
                    goto exit;
                }

                /* Copy data with bounds checking */
                memcpy(dataBuf, dev->slaveRxBuf, copyLen);

                /* CRITICAL DEBUG: Print slave RX buffer source data */
                LOGE("SMBus: SLAVE mode - slaveRxBuf source (%u bytes): ", copyLen);
                for (U32 i = 0; i < copyLen && i < 16; i++) {
                    LOGE("0x%02X ", dev->slaveRxBuf[i]);
                }
                if (copyLen > 16) {
                    LOGE("... (%u more bytes)", copyLen - 16);
                }
                LOGE("\n");

                /* CRITICAL DEBUG: Print dataBuf destination data */
                LOGE("SMBus: SLAVE mode - dataBuf destination (%u bytes): ", copyLen);
                for (U32 i = 0; i < copyLen && i < 16; i++) {
                    LOGE("0x%02X ", dataBuf[i]);
                }
                if (copyLen > 16) {
                    LOGE("... (%u more bytes)", copyLen - 16);
                }
                LOGE("\n");

                /* Clear the slave RX buffer after copying */
                memset(dev->slaveRxBuf, 0, dev->slaveValidRxLen);
                dev->slaveValidRxLen = 0;

                LOGD("SMBus: SLAVE mode - successfully copied %u bytes, buffer cleared\n", copyLen);

                ret = copyLen;  /* Return actual number of bytes copied */
                goto exit;
            }
        } else {
            /* Polling mode - not supported in SMBus implementation */
            LOGE("SMBus: SLAVE polling mode not supported\n");
            ret = -EPERM;  /* Operation not permitted */
            goto exit;
        }
    }

    ret = EXIT_SUCCESS;

exit:
    return ret;
}

/**
 * @brief I2C write-then-read operation for SMBus HAL layer
 * @details Implements I2C-compatible write-then-read operation at the HAL layer.
 *          This function provides direct I2C write-read capability without
 *          SMBus protocol overhead, enabling compatibility with I2C devices.
 * @param[in] dev Pointer to SMBus device structure
 * @param[in] slaveAddr 7-bit or 10-bit slave address
 * @param[in] writeBuf Pointer to data buffer to write
 * @param[in] writeLen Number of bytes to write
 * @param[out] readBuf Pointer to buffer to store read data
 * @param[in] readLen Number of bytes to read
 * @return EXIT_SUCCESS on successful operation, negative error code on failure:
 *         -EINVAL: Invalid parameters (NULL pointers, zero lengths)
 *         -EBUSY: Bus is busy or controller is not ready
 *         -EIO: I/O error during communication (NACK, timeout, bus error)
 *         -ETIMEDOUT: Transfer timeout
 *
 * @note Configures controller for master mode operation
 * @note Handles both 7-bit and 10-bit addressing
 * @note Performs combined write-then-read transaction with repeated start
 * @note Waits for bus idle before starting transfer
 * @note Configures appropriate timing based on device settings
 * @warning This function should only be called in Master mode
 * @warning Caller must ensure proper device locking before calling this function
 */
static S32 smbusI2cWriteRead(SmbusDev_s *dev, U16 slaveAddr, const U8 *writeBuf, U32 writeLen, U8 *readBuf, U32 readLen)
{
    S32 ret = EXIT_SUCCESS;
    SmbusMsg_t msgs[2];
    U32 msgCount = 2;

    /* Parameter validation */
    if (dev == NULL || dev->regBase == NULL || writeBuf == NULL || readBuf == NULL) {
        LOGE("SMBus: Invalid parameters for I2C write-read\n");
        return -EINVAL;
    }

    if (writeLen == 0 || readLen == 0) {
        LOGE("SMBus: Zero length for I2C write-read\n");
        return -EINVAL;
    }

    /* Configure messages for I2C write-then-read */
    /* First message: Write operation */
    msgs[0].addr = slaveAddr;
    msgs[0].flags = 0;  /* Write operation */
    msgs[0].len = writeLen;
    msgs[0].buf = (U8 *)writeBuf;  /* Remove const for compatibility */

    /* Second message: Read operation */
    msgs[1].addr = slaveAddr;
    msgs[1].flags = 1;  /* Read operation */
    msgs[1].len = readLen;
    msgs[1].buf = readBuf;

    /* Wait for bus to be idle */
    ret = smbusWaitBusNotBusy(dev);
    if (ret != EXIT_SUCCESS) {
        LOGE("SMBus: Bus busy, I2C write-read failed\n");
        return -EBUSY;
    }

    /* Use existing SMBus transfer functionality for I2C compatibility */
    ret = smbusDwXfer(dev, msgs, msgCount);
    if (ret < EXIT_SUCCESS ) {
        LOGE("SMBus: I2C write-read transfer failed, ret=%d\n", ret);
        return -EIO;
    }

    LOGD("SMBus: I2C write-read completed successfully, buf=0x%02X, write=%d, read=%d\n",
         msgs[1].buf[0], writeLen, readLen);
    return EXIT_SUCCESS;
}

/* ======================================================================== */
/*                    Async Interrupt Handler Functions                     */
/* ======================================================================== */

/**
 * @brief Read and clear interrupt bits for async SMBus transfers
 * @details Reads the interrupt status register and clears all pending
 *          interrupt bits by reading the appropriate clear registers.
 *          This function is essential for async interrupt operation.
 * @param[in] dev Pointer to SMBus device structure
 * @return Raw interrupt status before clearing
 * @note Based on I2C's i2c_dw_read_clear_intrbits implementation
 * @note Handles all interrupt types: RX_UNDER, RX_OVER, TX_OVER, TX_ABRT, etc.
 * @note Preserves TX abort source information before clearing
 * [ASYNC] Async interrupt support - interrupt status handling
 */
__attribute((unused)) static U32 smbusDwReadClearIntrbits(SmbusDev_s *dev)
{
    volatile SmbusRegMap_s *regBase = dev->regBase;
    U32 stat,dummy;

    if (regBase == NULL) {
        LOGE("SMBus: Invalid register base in ISR\n");
        return 0;
    }

    /*
     * The IC_INTR_STAT register just indicates "enabled" interrupts.
     * The unmasked raw version of interrupt status bits is available
     * in the IC_RAW_INTR_STAT register.
     *
     * That is,
     *   stat = readl(IC_INTR_STAT);
     * equals to,
     *   stat = readl(IC_RAW_INTR_STAT) & readl(IC_INTR_MASK);
     *
     * The raw version might be useful for debugging purposes.
     */
    stat = regBase->icIntrStat.value;

    /*
     * Do not use the IC_CLR_INTR register to clear interrupts, or
     * you'll miss some interrupts, triggered during the period from
     * readl(IC_INTR_STAT) to readl(IC_CLR_INTR).
     *
     * Instead, use the separately-prepared IC_CLR_* registers.
     */
    if (stat & SMBUS_IC_INTR_RX_UNDER_MASK)
        dummy = regBase->icClrRxUnder;
    if (stat & SMBUS_IC_INTR_RX_OVER_MASK)
        dummy = regBase->icClrRxOver;
    if (stat & SMBUS_IC_INTR_TX_OVER_MASK)
        dummy = regBase->icClrTxOver;
    if (stat & SMBUS_IC_INTR_RD_REQ_MASK)
        dummy = regBase->icClrRdReq;
    if (stat & SMBUS_IC_INTR_TX_ABRT_MASK) {
        /*
         * The IC_TX_ABRT_SOURCE register is cleared whenever
         * the IC_CLR_TX_ABRT is read.  Preserve it beforehand.
         */
        dev->abortSource = regBase->icTxAbrtSource.value;
        dummy = regBase->icClrTxAbrt;
    }
    if (stat & SMBUS_IC_INTR_RX_DONE_MASK)
        dummy = regBase->icClrRxDone;
    if (stat & SMBUS_IC_INTR_ACTIVITY_MASK)
        dummy = regBase->icClrActivity;
    if ((stat & SMBUS_IC_INTR_STOP_DET_MASK) &&
            ((dev->rxOutstanding == 0) || (stat & SMBUS_IC_INTR_RX_FULL_MASK)))
        dummy = regBase->icClrStopDet;
    if (stat & SMBUS_IC_INTR_START_DET_MASK)
        dummy = regBase->icClrStartDet;
    if (stat & SMBUS_IC_INTR_GEN_CALL_MASK)
        dummy = regBase->icClrGenCall;
    
    dummy = dummy;
    return stat;
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
 * [HAL] Hardware abstraction layer - Block Read protocol
 */
static U8 smbusDwRecvLen(SmbusDev_s *dev, U8 len)
{
    SmbusMsg_t *msgs = dev->msgs;
    U32 flags = msgs[dev->msgReadIdx].flags;
    U32 intrMask;

    /*
     * Adjust the buffer length and mask the flag
     * after receiving the first byte.
     */
    len += (flags & SMBUS_M_PEC) ? 2 : 1;  /* +1 for cmd, +1 for PEC if enabled */
    dev->masterTxBufLen = len - ((len < dev->rxOutstanding) ? len : dev->rxOutstanding);
    msgs[dev->msgReadIdx].len = len;
    msgs[dev->msgReadIdx].flags &= ~SMBUS_M_RECV_LEN;

    /*
     * Received buffer length, re-enable TX_EMPTY interrupt
     * to resume the SMBus Block Read transaction.
     * This is the key mechanism from DW I2C Databook.
     */
    intrMask = dev->regBase->icIntrMask;
    intrMask |= SMBUS_IC_INTR_TX_EMPTY_MASK;
    dev->regBase->icIntrMask = intrMask;

    LOGD("SMBus: Block Read len adjusted to %u, re-enabled TX_EMPTY\n", len);

    return len;
}

/**
 * @brief SMBus read handler (hardware-driven flow)
 * @details Strictly follows DW I2C Databook flow:
 *          RX_FULL interrupt → drain hardware FIFO → handle I2C_M_RECV_LEN → continue
 * @param[in] dev Pointer to SMBus device structure
 * @return void
 *
 * @note Does NOT use software counters for synchronization
 * @note Relies on hardware flow control via IC_RXFLR register
 * @note Automatically handles Block Read via I2C_M_RECV_LEN flag
 * @note Re-enables TX_EMPTY when length byte is received (critical!)
 * [HAL] Hardware abstraction layer - interrupt-driven read
 */
static void smbusDwRead(SmbusDrvData_s *pDrvData)
{
    volatile SmbusRegMap_s *regBase;
    SmbusDev_s *dev;
    SmbusMsg_t *msgs;
    U32 rxValid;
    U32 len;
    U8 *buf;

    if (pDrvData == NULL) {
        LOGE("SMBus: Invalid driver data in read ISR\n");
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

        LOGD("SMBus: Processing msg[%d], flags=0x%X, status=0x%X\n",
             dev->msgReadIdx, msgs[dev->msgReadIdx].flags, dev->status);

        /* Skip write messages */
        if (!(msgs[dev->msgReadIdx].flags & SMBUS_M_RD)) {
            continue;
        }

        /* Determine buffer and length based on read state */
        if (!(dev->status & SMBUS_STATUS_READ_IN_PROGRESS)) {
            /* Fresh read - use message buffer */
            len = msgs[dev->msgReadIdx].len;
            buf = msgs[dev->msgReadIdx].buf;
        } else {
            /* Continuation of partial read - use pDrvData->rxBuffer */
            len = pDrvData->rxLength;
            buf = pDrvData->rxBuffer;
        }

        /* Read actual FIFO level from hardware (KEY: no software counter) */
        rxValid = regBase->icRxflr;
        LOGD("SMBus: Hardware FIFO level=%u, remaining len=%u\n", rxValid, len);

        /* Drain FIFO while data is available and buffer has space */
        for (; len > 0 && rxValid > 0; len--, rxValid--) {
            flags = msgs[dev->msgReadIdx].flags;

            LOGD("SMBus: Reading byte, len=%u, rxValid=%u, flags=0x%X\n", 
                 len, rxValid, flags);

            /* Read data from hardware FIFO */
            tmp = regBase->icDataCmd.value;
            tmp &= 0xFF;  /* Extract data byte */

            /* Handle Block Read length byte (I2C_M_RECV_LEN) */
            if (flags & SMBUS_M_RECV_LEN) {
                /*
                 * CRITICAL: DW I2C Databook mechanism for Block Read
                 * 
                 * If IC_EMPTYFIFO_HOLD_MASTER_EN is set, the controller 
                 * can be disabled if STOP bit is set. But it is only set
                 * after receiving block data response length in Block Read.
                 * That needs to read another byte with STOP bit when the
                 * block data response length is invalid to complete transaction.
                 */
                if (!tmp || tmp > SMBUS_BLOCK_MAXLEN) {
                    tmp = 1;  /* Minimal valid length to complete transfer */
                }

                /* Adjust length and re-enable TX_EMPTY (KEY MECHANISM) */
                len = smbusDwRecvLen(dev, tmp);
            }

            /* Store received byte */
            *buf++ = tmp;
            dev->rxOutstanding--;
        }

        /* Check if current message is complete */
        if (len > 0) {
            /* 
             * Message not complete - save state and wait for next RX_FULL
             * This is the hardware flow control mechanism
             */
            dev->status |= SMBUS_STATUS_READ_IN_PROGRESS;
            pDrvData->rxLength = len;
            pDrvData->rxBuffer = buf;
            LOGD("SMBus: Read incomplete, %u bytes remaining\n", len);
            return;
        } else {
            /* Message complete - clear read-in-progress flag */
            dev->status &= ~SMBUS_STATUS_READ_IN_PROGRESS;
            LOGD("SMBus: Message[%d] read complete\n", dev->msgReadIdx);
        }
    }

    LOGD("SMBus: All read messages processed\n");
}

/**
 * @brief Initiate and continue low level SMBus master read/write transaction
 * @details This function is called from smbusDwIsr to pump SmbusMsg_t messages
 *          into the TX buffer. Handles messages longer than FIFO depth by
 *          processing them in multiple interrupt cycles.
 * @param[in] dev Pointer to SMBus device structure
 * @return void
 *
 * @note Based on Linux i2c_dw_xfer_msg with SMBus-specific enhancements
 * @note Follows DW_apb_i2c 2.03a databook recommendations
 * [HAL] Hardware abstraction layer - transfer message handling
 */
/**
 * @brief Initiate and continue low level SMBus master read/write transaction
 * @details This function is called from smbusDwIsr to pump SmbusMsg_t messages
 *          into the TX buffer. Handles messages longer than FIFO depth.
 * @param[in] dev Pointer to SMBus device structure
 * @note Based on Linux i2c_dw_xfer_msg, adapted for your project structure
 * @note Follows DW_apb_i2c 2.03a databook section 7.5
 */
/**
 * @brief Initiate and continue low level SMBus master read/write transaction
 * @details This function is called from smbusDwIsr to pump SmbusMsg_t messages
 *          into the TX buffer. Handles messages longer than FIFO depth by
 *          processing them in multiple interrupt cycles.
 * @param[in] dev Pointer to SMBus device structure
 * @return void
 *
 * @note Based on Linux i2c_dw_xfer_msg with SMBus-specific enhancements
 * @note Follows DW_apb_i2c 2.03a databook recommendations
 * [HAL] Hardware abstraction layer - transfer message handling
 */
static void smbusDwXferMsg(SmbusDev_s *dev)
{
    volatile SmbusRegMap_s *regBase;
    SmbusMsg_t *msgs;
    SmbusMsg_t *msg;
    U32 intrMask;
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
    
    /* Default interrupt mask for master mode */
    intrMask = SMBUS_IC_INTR_TX_EMPTY_MASK | 
               SMBUS_IC_INTR_TX_ABRT_MASK | 
               SMBUS_IC_INTR_STOP_DET_MASK |
               SMBUS_IC_INTR_RX_FULL_MASK;

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
        if (flags & SMBUS_M_RECV_LEN) {
            dev->status |= SMBUS_STATUS_WRITE_IN_PROGRESS;
            intrMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
            LOGD("SMBus: RECV_LEN mode, disabling TX_EMPTY\n");
            break;
        } else if (bufLen > 0) {
            /* More bytes to write in current message */
            dev->status |= SMBUS_STATUS_WRITE_IN_PROGRESS;
            LOGD("SMBus: %d bytes remaining in current msg\n", bufLen);
            break;
        } else {
            /* Current message complete */
            dev->status &= ~SMBUS_STATUS_WRITE_IN_PROGRESS;
            LOGD("SMBus: Message[%d] queued completely\n", dev->msgWriteIdx);
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
    
    /* Step 2: Now disable TX_EMPTY and wait for STOP_DET */
    intrMask &= ~SMBUS_IC_INTR_TX_EMPTY_MASK;
    intrMask |= SMBUS_IC_INTR_STOP_DET_MASK;
    
    LOGD("SMBus: All queued, TFE=%d, timeout=%d\n", 
         regBase->icStatus.fields.tfe, timeout);
    }

    /* Disable all interrupts if error occurred */
    if (dev->msgErr) {
        intrMask = 0;
        LOGE("SMBus: Error detected (0x%X), disabling all interrupts\n",
             dev->msgErr);
    }

    /* Update interrupt mask register */
    regBase->icIntrMask = intrMask;
    LOGD("SMBus: Updated interrupt mask=0x%08X\n", intrMask);
}

/**
 * @brief SMBus I2C reset function for hardware recovery
 * @details This function implements a comprehensive reset sequence for the SMBus controller
 *          following the I2C reset pattern. It handles both Master and Slave modes,
 *          performs hardware recovery, clock enable/reset, and reinitializes the controller.
 * @param[in] dev Pointer to SMBus device structure
 * @param[in] devId Device identifier for clock/reset operations
 * @return EXIT_SUCCESS on successful reset, negative error code on failure:
 *         -EINVAL: Invalid parameters (NULL dev)
 *         -EIO: Hardware reset or reinitialization failed
 *
 * @note Based on I2C reset pattern from i2c driver
 * @note Handles SDA stuck recovery for Master mode
 * @note Clears SMBus interrupt mask during reset
 * @note Reconfigures controller based on current mode
 * @note Supports both interrupt and polling modes for Slave
 * @warning This function performs hardware reset - ensure bus isolation if needed
 * @warning Controller will be temporarily disabled during reset sequence
 * [HAL] Hardware abstraction layer - I2C compatibility reset operations
 */
S32 smbusI2cReset(SmbusDev_s *dev, DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;

    /* Parameter validation */
    if (dev == NULL || dev->regBase == NULL) {
        LOGE("SMBus I2C Reset: Invalid device structure\n");
        return -EINVAL;
    }

    LOGD("SMBus I2C Reset: Starting reset for device %d, mode=%s\n",
         devId, (dev->mode == DW_SMBUS_MODE_MASTER) ? "MASTER" : "SLAVE");

    /* Step 1: Perform mode-specific cleanup (similar to i2c_dw_sda_stuck_recover/unprobe) */
    if (dev->mode == DW_SMBUS_MODE_MASTER) {
        /* Master mode: Enable comprehensive SDA stuck recovery and unprobe */
        LOGD("SMBus I2C Reset: Master mode - performing comprehensive SDA stuck recovery\n");

        /* [Enable comprehensive SDA stuck recovery logic] */
        SmbusIcEnableReg_u enable;
        SmbusIcConReg_u conTmp;
        SmbusIcStatusReg_u statusTmp;
        SmbusIcTxAbrtSourceReg_u abortTmp;
        U32 recoveryTimeout;

        /* Step 1: Disable controller */
        enable.value = dev->regBase->icEnable.value;
        enable.fields.enable = 0;
        dev->regBase->icEnable.value = enable.value;

        /* Step 2: Wait for controller to be fully disabled */
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(1));

        /* Step 3: Configure bus clear control in CON register */
        conTmp.value = dev->regBase->icCon.value;
        conTmp.value |= SMBUS_IC_CON_BUS_CLEAR_CTRL_MASK;
        dev->regBase->icCon.value = conTmp.value;

        /* Step 4: Re-enable controller */
        smbusEnable(dev);
        /* Step 5: Wait for stabilization */
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(1));

        /* Step 6: Check abort source for SDA stuck condition */
        abortTmp.value = dev->regBase->icTxAbrtSource.value;
        if (abortTmp.value & SMBUS_IC_TX_ABRT_SDA_STUCK_AT_LOW_MASK) {
            LOGD("SMBus I2C Reset: SDA stuck condition detected, starting recovery\n");

            /* Step 7: Start SDA stuck recovery */
            enable.value = dev->regBase->icEnable.value;
            enable.fields.sdaStuckRecoveryEnable = 1;
            dev->regBase->icEnable.value = enable.value;

            /* Step 8: Wait for recovery completion with timeout */
            recoveryTimeout = SMBUS_SDA_STUCK_RECOVERY_TIMEOUT_MS;
            while (recoveryTimeout > 0) {
                rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(1));

                enable.value = dev->regBase->icEnable.value;
                if (!(enable.value & (1U << 3))) { /* SDA_STUCK_RECOVERY bit is bit 3 */
                    /* Recovery completed successfully */
                    LOGD("SMBus I2C Reset: SDA stuck recovery completed successfully\n");
                    break;
                }
                recoveryTimeout--;
            }

            if (recoveryTimeout == 0) {
                /* SDA stuck recovery timeout */
                LOGE("SMBus I2C Reset: SDA stuck recovery timeout\n");
                return -SMBUS_ERR_TX_ABRT;
            } else {
                /* Step 9: Check final recovery status */
                statusTmp.value = dev->regBase->icStatus.value;
                if (statusTmp.value & SMBUS_IC_STATUS_SDA_STUCK_NOT_RECOVERED_MASK) {
                    /* SDA stuck cannot be recovered */
                    LOGE("SMBus I2C Reset: SDA stuck cannot be recovered\n");
                    return -SMBUS_ERR_TX_ABRT;
                }
            }
        } else {
            LOGD("SMBus I2C Reset: No SDA stuck condition detected\n");
        }

        /* Unprobe master to clean up current state */
        ret = smbusUnprobeMaster(dev);
        if (ret != EXIT_SUCCESS) {
            LOGW("SMBus I2C Reset: Master unprobe failed, continuing with reset: %d\n", ret);
            /* Continue with reset even if unprobe fails */
        }
    } else {
        /* Slave mode: Unprobe slave to clean up current state */
        LOGD("SMBus I2C Reset: Slave mode - unprobing slave\n");

        ret = smbusUnprobeSlave(dev);
        if (ret != EXIT_SUCCESS) {
            LOGW("SMBus I2C Reset: Slave unprobe failed, continuing with reset: %d\n", ret);
            /* Continue with reset even if unprobe fails */
        }
    }

    /* Step 2: Enable peripheral clock */
    LOGD("SMBus I2C Reset: Enabling peripheral clock for device %d\n", devId);
    ret = peripsClockEnable(devId);
    if (ret != EXIT_SUCCESS && ret != -EINVAL) {
        LOGE("SMBus I2C Reset: Clock enable failed for device %d: %d\n", devId, ret);
        ret = -EXIT_FAILURE;
        goto exit;
    }

    /* Step 3: Perform peripheral reset */
    LOGD("SMBus I2C Reset: Performing peripheral reset for device %d\n", devId);
    if (peripsReset(devId) != EXIT_SUCCESS) {
        LOGW("SMBus I2C Reset: Peripheral reset failed for device %d\n", devId);
        /* Continue even if reset fails, similar to I2C implementation */
    }

    /* Step 4: Clear SMBus interrupt mask (critical for SMBus compatibility) */
    LOGD("SMBus I2C Reset: Clearing SMBus interrupt mask\n");
    dev->regBase->icSmbusIntrMask = 0;

    /* Step 5: Reconfigure based on current mode */
    if (dev->mode == DW_SMBUS_MODE_MASTER) {
        /* Master mode reconfiguration */
        LOGD("SMBus I2C Reset: Reconfiguring master mode\n");

        smbusConfigureMaster(dev);

        /* Probe master to complete initialization */
        ret = smbusProbeMaster(dev);
        if (ret != EXIT_SUCCESS) {
            LOGE("SMBus I2C Reset: Master probe failed after reset: %d\n", ret);
            ret = -EXIT_FAILURE;
            goto exit;
        }

        LOGD("SMBus I2C Reset: Master mode reinitialization completed\n");
    } else {
        /* Slave mode reconfiguration */
        LOGD("SMBus I2C Reset: Reconfiguring slave mode\n");

        smbusConfigureSlave(dev);

        /* Check work mode and probe accordingly */
        if (dev->workMode == 0) { /* Interrupt mode */
            ret = smbusProbeSlave(dev);
            if (ret != EXIT_SUCCESS) {
                LOGE("SMBus I2C Reset: Slave probe failed after reset: %d\n", ret);
                ret = -EXIT_FAILURE;
                goto exit;
            }
            LOGD("SMBus I2C Reset: Slave interrupt mode reinitialization completed\n");
        } else {
            /* Polling mode not supported for SMBus */
            LOGE("SMBus I2C Reset: Slave polling mode not supported\n");
            ret = -EXIT_FAILURE;
            goto exit;
        }
    }

    /* Step 6: Final hardware state verification */
    /* Wait a moment for hardware to settle */
    udelay(1000);  /* 1ms delay */

    /* Verify controller is accessible */
    volatile U32 testRead = dev->regBase->icCon.value;
    if (testRead == 0xFFFFFFFF) {
        LOGE("SMBus I2C Reset: Hardware not accessible after reset\n");
        ret = -EXIT_FAILURE;
        goto exit;
    }

    LOGD("SMBus I2C Reset: Hardware verification passed (CON=0x%08X)\n", testRead);

    /* Reset completed successfully */
    LOGI("SMBus I2C Reset: Reset completed successfully for device %d, mode=%s\n",
         devId, (dev->mode == DW_SMBUS_MODE_MASTER) ? "MASTER" : "SLAVE");
    ret = EXIT_SUCCESS;

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
        .waitTransmitComplete = waitTransmitComplete,
        .enable = smbusEnable,
        .disable = smbusDisable,
        .devAddrAssignCore = smbusDevAddrAssignCore,
        .hostNotifyCore = smbusHostNotifyCore,
        .modeSwitchCore = smbusModeSwitchCore,
        .i2cWrite = smbusI2cWrite,
        .i2cRead = smbusI2cRead,
        .i2cWriteRead = smbusI2cWriteRead,
        .i2cReset = smbusI2cReset,
        .smbusDwRead = smbusDwRead,
        .smbusDwXferMsg = smbusDwXferMsg,
    };

    return &halOps;
}