/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_smbus_dw_slaver.c
 * @author wangkui (wangkui@starsmicrosystem.com)
 * @date 2025/10/20
 * @brief SMBus Slave API layer implementation
 *
 * @par ChangeLog:
 * Date         Author          Description
 * 2025/10/20   wangkui         Initial version
 * 2025/11/20   wangkui         porting marco style
 * 2025/11/23   wangkui         HAL operations optimization - eliminated redundant smbusGetHalOps() calls
 *                              Replaced direct HAL access with dev->halOps pointer for better performance
 *                              Reduced function call overhead by registering HAL ops during initialization
 * @note This file implements the SMBus Slave API layer, providing
 *       high-level Slave mode operations for SMBus communication.
 *       It works on top of the SMBus core layer and handles
 *       Slave-specific functionality including address resolution,
 *       response handling, ARP protocol support, and event processing
 *       in Slave mode.
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
/*                    Slave Configuration Functions                           */
/* ======================================================================== */

/**
 * @brief Configure SMBus controller in slave mode
 * @details Configures the DesignWare SMBus controller for slave operation.
 *          This function disables the controller, sets the appropriate
 *          configuration bits for slave mode, and configures the slave address.
 * @param[in] dev Pointer to the SMBus device structure
 * @return void
 *
 * @note Disables controller before reconfiguration
 * @note Sets fast mode (2) as default speed
 * @note Enables RESTART conditions for combined transactions
 * @note Configures slave address after mode configuration
 * @warning The function does not re-enable the controller after configuration
 */
static void smbusConfigureSlave(SmbusDev_s *dev)
{
    SmbusIcConReg_u con;

    /* Parameter validation */
    SMBUS_CHECK_PARAM_VOID(dev == NULL || dev->regBase == NULL,
                          "%s(): dev or dev->regBase is NULL", __func__);

    /* Check HAL operations */
    SMBUS_CHECK_PARAM_VOID(dev->halOps == NULL || dev->halOps->disable == NULL,
                          "%s(): dev->halOps or dev->halOps->disable is NULL", __func__);

    /* Disable before configuration */
    dev->halOps->disable(dev);

    /* Read current configuration */
    con.value = dev->regBase->icCon.value;

    /* Configure as slave */
    con.fields.masterMode = 0;
    con.fields.icSlaveDisable = 0;
    con.fields.icRestartEn = 1;
    con.fields.speed = 2;  /* Fast mode */

    /* Set address mode */
    if (dev->addrMode == 1) {
        con.fields.ic10bitaddrSlave = 1;
    } else {
        con.fields.ic10bitaddrSlave = 0;
    }

    /* Write configuration */
    dev->regBase->icCon.value = con.value;

    /* Configure optimal interrupt mask for Slave mode - 修复添加WR_REQ中断 */
    U32 intrMask = 0;
    intrMask |= SMBUS_INTR_WR_REQ;    // bit[15] - 写请求 (关键修复！)
    intrMask |= SMBUS_INTR_RD_REQ;    // bit[5] - Master 读请求
    intrMask |= SMBUS_INTR_RX_FULL;   // bit[2] - RX FIFO 满
    intrMask |= SMBUS_INTR_STOP_DET;  // bit[9] - STOP 条件
    intrMask |= SMBUS_INTR_TX_ABRT;   // bit[6] - 传输终止

    /* 添加Slave地址标签中断 (bits 16-19) - 使用标准定义 */
    intrMask |= SMBUS_INTR_SLV_ADDR1_TAG;  // bit[16] - Slave地址1标签

    dev->regBase->icIntrMask.value = intrMask;
    LOGD("SMBus Slave: Configured interrupt mask=0x%08X (WR_REQ+RD_REQ+RX_FULL+STOP_DET+TX_ABRT+ADDR_TAG)\n", intrMask);

    /* Set slave address */
    if (dev->halOps->setSlaveAddr != NULL) {
        dev->halOps->setSlaveAddr(dev);
    }
}

/**
 * @brief ARP event notification function
 * @details Notifies registered callbacks about ARP events such as address changes,
 *          device discovery, or protocol state changes. This function iterates
 *          through all registered callbacks and invokes them with the event data.
 * @param[in] notifyCb Pointer to ARP notification callback structure
 * @param[in] event Type of ARP event that occurred
 * @param[in] dev Pointer to device information structure
 * @return void
 */
/**
 * @brief Notify registered callbacks of ARP events
 * 
 * @param notifyCb Pointer to notification callback structure
 * @param event ARP event type
 * @param dev Pointer to ARP device data
 */
static void smbusArpNotifyEvent(SmbusArpNotifyCb_s *notifyCb, 
                               SmbusArpEvent_e event, 
                               const SmbusArpDev_s *dev)
{
    U32 i;

    /* Parameter validation */
    if (notifyCb == NULL || dev == NULL) {
        LOGE("%s(): Invalid parameters (notifyCb=%p, dev=%p)\n", 
             __func__, notifyCb, dev);
        return;
    }

    /* Validate callback count */
    if (notifyCb->callbackCount == 0) {
        LOGD("%s(): No callbacks registered for event %d\n", __func__, event);
        return;
    }

    LOGD("%s(): Notifying ARP event %d for device addr 0x%02X (count=%u)\n",
         __func__, event, dev->newAddr, notifyCb->callbackCount);

    /* Iterate through all registered callbacks */
    for (i = 0; i < notifyCb->callbackCount; i++) {
        if (notifyCb->callbacks[i] != NULL) {
            LOGD("%s(): Invoking callback[%u] for event %d\n", __func__, i, event);
            
            /* Invoke callback with event, device data, and user data */
            notifyCb->callbacks[i](event, dev, notifyCb->userData[i]);
        } else {
            LOGW("%s(): Callback[%u] is NULL, skipping\n", __func__, i);
        }
    }

    LOGD("%s(): ARP event notification completed\n", __func__);
}
 

/* ======================================================================== */
/*                        ARP Slave API Functions                           */
/* ======================================================================== */

/**
 * @brief Initialize SMBus ARP Slave LOGEc according to SMBus ARP protocol.
 *
 * This function implements the ARP slave initialization sequence shown in Figure 6‑13
 * ("ARP Slave Sequence"). It is responsible for configuring the DesignWare I2C/SMBus
 * controller (`DW_apb_i2c`) into proper SMBus ARP Slave mode, detecting whether a
 * Persistent Slave Address (PSA) or Dynamic Slave Address (DSA) exists, and managing
 * the Address Valid / Resolved flags.
 *
 * The function configures the controller registers in correct sequence:
 * 1. Reset HW ARP flags
 * 2. Load persistent address if available
 * 3. Enable ARP mode in IC_CON
 * 4. Wait for ARP or SMBus command per protocol state
 *
 * @param[in] devId      Target SMBus device identifier
 * @return  0 if successful; negative error code otherwise
 *
 * @note The function must be called once after hardware reset, before SMBus slave operations.
 * [SLAVE_API] Slave API layer - high-level Slave mode operations
 */
S32 smbusArpSlaveInit(DevList_e devId)
{
    SmbusDrvData_s *pDrvData = NULL;
    S32 ret = EXIT_SUCCESS;
    SmbusIcConReg_u con;

    /* Check driver lock and validate */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto exit;
    }

    SMBUS_CHECK_PARAM(pDrvData == NULL, -EINVAL,
                     "%s(): pDrvData is NULL", __func__);

    /* Reset HW ARP flags */
    pDrvData->arpEnabled = false;
    pDrvData->arpState = SMBUS_ARP_STATE_INIT;

    /* Enable SMBus controller in slave mode */
    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;
    SMBUS_CHECK_PARAM(regBase == NULL, -EINVAL,
                     "%s(): regBase is NULL", __func__);

    /* Configure controller for SMBus slave mode */
    SmbusDev_s *dev = &pDrvData->pSmbusDev;
    SMBUS_CHECK_PARAM(dev->halOps == NULL || dev->halOps->enable == NULL, -ENOSYS,
                     "%s(): dev->halOps or dev->halOps->enable is NULL", __func__);

    smbusConfigureSlave(dev);

    /* Clear all SMBus-related interrupt status */
    regBase->icClrSmbusIntr |= SMBUS_BUS_PROTOCOL_INT_MASK | SMBUS_ARP_INTR_MASK |
                                    SMBUS_CLOCK_EXT_INT_MASK;

    /* Enable SAR1 and SAR2 interrupts in I2C interrupt mask (0x30) */
    regBase->icIntrMask.value |= SMBUS_INTR_SLV_ADDR1_TAG | SMBUS_INTR_SLV_ADDR2_TAG;

    /* Enable ARP mode in IC_CON */
    con.value = regBase->icCon.value;
    con.fields.smbusArpEn = 1;
    regBase->icCon.value = con.value;

    /* Enable controller */
    dev->halOps->enable(dev);

    /* Mark ARP as initialized */
    pDrvData->arpInitialized = true;

    LOGD("%s(): ARP Slave initialization completed\n", __func__);

exit:
    devUnlockByDriver(devId);
    return ret;
}

/**
 * @brief Prepare ARP Slave for operation
 * [SLAVE_API] Slave API layer - high-level Slave mode operations
 */
S32 smbusArpSlaveHandlerPrep(DevList_e devId)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;

    /* Check driver lock and validate */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    SMBUS_CHECK_PARAM(pDrvData == NULL, -EINVAL,
                     "%s(): pDrvData is NULL", __func__);

    /* Check HAL operations */
    SMBUS_CHECK_PARAM(pDrvData->pSmbusDev.halOps->enable == NULL, -ENOSYS,
                     "%s(): dev->halOps or dev->halOps->enable is NULL", __func__);

    /* Check if ARP is initialized */
    if (!pDrvData->arpInitialized) {
        ret = -EAGAIN;
        goto unlock;
    }

    /* Prepare ARP Slave for operation by enabling appropriate interrupts */
    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;
    SMBUS_CHECK_PARAM(regBase == NULL, -EINVAL,
                     "%s(): regBase is NULL", __func__);

    /* Enable ARP-related interrupts */
    U32 mask = regBase->icSmbusIntrMask.value;
    mask |= SMBUS_ARP_INTR_MASK;
    regBase->icSmbusIntrMask.value = mask;

    /* Set ARP state to ready */
    pDrvData->arpState = SMBUS_ARP_STATE_READY;
    pDrvData->arpEnabled = true;

    pDrvData->pSmbusDev.halOps->enable(&pDrvData->pSmbusDev);
    LOGD("%s(): ARP Slave prepared for operation\n", __func__);

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

/**
 * @brief Handle ARP Slave reset command
 * [SLAVE_API] Slave API layer - high-level Slave mode operations
 */
S32 smbusArpSlaveHandlerReset(DevList_e devId, Bool directed, const SmbusUdid_s *udid)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;

    SMBUS_CHECK_PARAM(udid == NULL, -EINVAL,
                     "%s(): udid is NULL", __func__);

    /* Check driver lock and validate */
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    SMBUS_CHECK_PARAM(pDrvData == NULL, -EINVAL,
                     "%s(): pDrvData is NULL", __func__);

    /* Check HAL operations */
    SMBUS_CHECK_PARAM(pDrvData->pSmbusDev.halOps == NULL, -ENOSYS,
                     "%s(): pDrvData->pSmbusDev.halOps is NULL", __func__);

    /* Check if reset is directed to this device by comparing UDID */
    if (directed) {
        /* Compare UDID with device's UDID */
        if (memcmp(&pDrvData->udid, udid, sizeof(SmbusUdid_s)) != 0) {
            /* UDID doesn't match, this reset is not for us */
            LOGD("%s(): UDID mismatch, reset not for this device\n", __func__);
            ret = -ENOENT;
            goto unlock;
        }
    }

    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;
    SMBUS_CHECK_PARAM(regBase == NULL, -EINVAL,
                     "%s(): regBase is NULL", __func__);

    /* Reset ARP state machine */
    pDrvData->arpState = SMBUS_ARP_STATE_INIT;
    pDrvData->arpEnabled = false;
    pDrvData->arpInitialized = false;

    /* Clear all ARP-related interrupts */
    regBase->icClrSmbusIntr |= SMBUS_SLV_RX_PEC_NACK_BIT | SMBUS_ARP_PREPARE_CMD_DET_BIT |
                                    SMBUS_GET_UDID_CMD_DET_BIT | SMBUS_ARP_RST_CMD_DET_BIT |
                                    SMBUS_ASSIGN_ADDR_CMD_DET_BIT;

    /* Disable ARP-related interrupts */
    U32 mask = regBase->icSmbusIntrMask.value;
    mask &= ~(SMBUS_ARP_INTR_MASK);
    regBase->icSmbusIntrMask.value = mask;

    /* Reset assigned addresses if this was a directed reset */
    if (directed) {
        /* Clear SAR registers */
        regBase->icSar.value = 0;
        regBase->icSar2.value = 0;
        LOGD("%s(): Directed ARP reset completed\n", __func__);
    } else {
        LOGD("%s(): General ARP reset completed\n", __func__);
    }

    /* Re-initialize ARP if needed */
    if (pDrvData->pSmbusDev.halOps->enable != NULL) {
        pDrvData->arpInitialized = false;
        /* Note: Caller should call smbusArpSlaveInit() if re-initialization is needed */
    }

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

/* ======================================================================== */
/*                    SAR (Slave Address) Functions                         */
/* ======================================================================== */

/**
 * [SLAVE_API] Slave API layer - high-level Slave mode operations
 */
S32 smbusSarEnable(DevList_e devId, U32 sarNum)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;
    volatile SmbusRegMap_s *regBase = NULL;

    SMBUS_CHECK_PARAM_RETURN(sarNum > 2 || sarNum == 0, -EINVAL,
                            "%s(): invalid sarNum %u, must be 1 or 2", __func__, sarNum);

    ///< Check driver lock and validate
    SMBUS_CHECK_PARAM_RETURN(smbusDrvLockAndCheck(devId) != EXIT_SUCCESS, -EINVAL,
                            "%s(): smbusDrvLockAndCheck failed", __func__);

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        devUnlockByDriver(devId);
        ret = -EINVAL;
        goto exit;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        devUnlockByDriver(devId);
        ret = -EINVAL;
        goto exit;
    }

    SMBUS_CHECK_PARAM_RETURN(pDrvData == NULL, -EINVAL,
                            "%s(): pDrvData is NULL", __func__);

    /* Enable specified SAR (Slave Address Register) */
    SmbusIcEnableReg_u sarReg;
#if 1
    /* Enable SAR based on number */
    if (sarNum == 1) {
        /* Configure and enable SAR1 */
        sarReg.value = regBase->icEnable.value;
        sarReg.fields.icSarEn = 1;
        regBase->icEnable.value = sarReg.value;

        /* Enable SAR1 interrupts */
        U32 mask = regBase->icIntrMask.value;
        mask |= SMBUS_INTR_SLV_ADDR1_TAG;
        regBase->icIntrMask.value = mask;

        LOGD("%s(): SAR1 enabled\n", __func__);
    } else if (sarNum == 2) {
        /* Configure and enable SAR2 */
        sarReg.value = regBase->icEnable.value;
        sarReg.fields.icSar2En = 1;
        regBase->icEnable.value = sarReg.value;

        /* Enable SAR2 interrupts */
        U32 mask = regBase->icIntrMask.value;
        mask |= SMBUS_INTR_SLV_ADDR2_TAG;
        regBase->icIntrMask.value = mask;

        LOGD("%s(): SAR2 enabled\n", __func__);
    }
#endif
    /* Update device status */
    pDrvData->pSmbusDev.status |= SMBUS_STATUS_SLAVE_ENABLED_MASK;

exit:
    /* Unlock device */
    devUnlockByDriver(devId);

    return ret;
}

/**
 * @brief Disable SAR (Slave Address)
 * [SLAVE_API] Slave API layer - high-level Slave mode operations
 */
S32 smbusSarDisable(DevList_e devId, U32 sarNum)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;
    volatile SmbusRegMap_s *regBase = NULL;

    SMBUS_CHECK_PARAM(sarNum > 2 || sarNum == 0, -EINVAL,
                   "%s(): invalid sarNum %u, must be 1 or 2", __func__, sarNum);

    ///< Check driver lock and validate
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    /* Disable specified SAR (Slave Address Register) */
      SmbusIcEnableReg_u sarReg;
#if 1
    /* Disable SAR based on number */
    if (sarNum == 1) {
        /* Configure and disable SAR1 */
        sarReg.value = regBase->icEnable.value;
        sarReg.fields.icSarEn = 0;
        regBase->icEnable.value = sarReg.value;

        /* Disable SAR1 interrupts */
        U32 mask = regBase->icIntrMask.value;
        mask &= ~SMBUS_INTR_SLV_ADDR1_TAG;
        regBase->icIntrMask.value = mask;

        LOGD("%s(): SAR1 disabled\n", __func__);
    } else if (sarNum == 2) {
        /* Configure and disable SAR2 */
        sarReg.value = regBase->icEnable.value;
        sarReg.fields.icSar2En = 0;
        regBase->icEnable.value = sarReg.value;

        /* Disable SAR2 interrupts */
        U32 mask = regBase->icIntrMask.value;
        mask &= ~SMBUS_INTR_SLV_ADDR2_TAG;
        regBase->icIntrMask.value = mask;

        LOGD("%s(): SAR2 disabled\n", __func__);
    }
    #endif
    /* Update device status - check if both SARs are disabled */
    SmbusIcEnableReg_u sar1Reg = {.value = regBase->icEnable.value};
    SmbusIcEnableReg_u sar2Reg = {.value = regBase->icEnable.value};

    if (!sar1Reg.fields.icSarEn && !sar2Reg.fields.icSar2En) {
        pDrvData->pSmbusDev.status &= ~SMBUS_STATUS_SLAVE_ENABLED_MASK;
    }

unlock:
    /* Unlock device */
    devUnlockByDriver(devId);

exit:
    return ret;
}

/* ======================================================================== */
/*                    ARP Additional Functions                               */
/* ======================================================================== */

/**
 * @brief ARP Enable function
 * [SLAVE_API] Slave API layer - high-level Slave mode operations
 */
S32 smbusArpEnable(DevList_e devId, U32 sarNum)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;
    volatile SmbusRegMap_s *regBase = NULL;
    U32 sarTagBit = 0; 

    switch (sarNum) {
        case 1:
            sarTagBit = SMBUS_INTR_SLV_ADDR1_TAG;
            break;
        case 2:
            sarTagBit = SMBUS_INTR_SLV_ADDR2_TAG;
            break;
        default:
            SMBUS_CHECK_PARAM(true, -EINVAL,
                              "%s(): invalid sarNum %u, must be 1 or 2", __func__, sarNum);
            break;
    }
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto unlock; // 修正 goto exit -> goto unlock
    }

    SMBUS_CHECK_PARAM(pDrvData == NULL, -EINVAL,
                      "%s(): pDrvData is NULL", __func__);

    /* ========================================================== */
    /* 核心：分离 I2C 和 SMBus 寄存器操作                           */
    /* ========================================================== */
    
    ///< 2. 启用 IC_CON 寄存器中的 ARP 模式
    SmbusIcConReg_u con;
    con.value = regBase->icCon.value;
    con.fields.smbusArpEn = 1;
    regBase->icCon.value = con.value;

    ///< 3. 启用 SMBus 专用中断 (IC_SMBUS_INTR_MASK)
    {
        U32 smbusMask = regBase->icSmbusIntrMask.value;
        smbusMask |= SMBUS_ARP_INTR_MASK;
        regBase->icSmbusIntrMask.value = smbusMask;
    }

    ///< 4. 启用通用 I2C 中断 (IC_INTR_MASK) - 只处理地址标签位
    if (sarTagBit != 0) {
        U32 i2cMask = regBase->icIntrMask.value;
        i2cMask |= sarTagBit;                      ///< 启用对应的 Slave Address Tag 中断
        regBase->icIntrMask.value = i2cMask;
    }
    
    // 5. 更新驱动数据状态
    pDrvData->arpEnabled = true;
    if (pDrvData->arpState == SMBUS_ARP_STATE_INIT) {
        pDrvData->arpState = SMBUS_ARP_STATE_READY;
    }

    LOGD("%s(): ARP enabled for SAR%u\n", __func__, sarNum);

unlock:
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief Disable ARP functionality for a specific SAR
 * @param devId Device identifier
 * @param sarNum SAR number (1 or 2)
 * @return Status code
 */
S32 smbusArpDisable(DevList_e devId, U32 sarNum)
{
   S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;
    volatile SmbusRegMap_s *regBase = NULL;
    U32 sarTagBit = 0;
    
    ///< 使用宏来简化 SAR 地址标签位的获取
    switch (sarNum) {
        case 1:
            sarTagBit = SMBUS_INTR_SLV_ADDR1_TAG;
            break;
        case 2:
            sarTagBit = SMBUS_INTR_SLV_ADDR2_TAG;
            break;
        default:
            SMBUS_CHECK_PARAM(sarNum > 2 || sarNum == 0, -EINVAL,
                              "%s(): invalid sarNum %u, must be 1 or 2", __func__, sarNum);
            break;
    }
    ///< Check driver lock and validate
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto unlock;
    }

    SMBUS_CHECK_PARAM(pDrvData == NULL, -EINVAL,
                     "%s(): pDrvData is NULL", __func__);

    /* Disable ARP interrupts for the specified SAR */
    U32 smbusMask = regBase->icSmbusIntrMask.value;
    ///< 屏蔽所有 ARP 相关的 SMBus 专用中断
    smbusMask &= ~SMBUS_ARP_INTR_MASK;
    regBase->icSmbusIntrMask.value = smbusMask;

    /* Clear any pending ARP interrupts for the specified SAR */
    if (sarTagBit != 0) {
        U32 i2cMask = regBase->icIntrMask.value;
        // 屏蔽对应的 Slave Address Tag 中断
        i2cMask &= ~sarTagBit;
        regBase->icIntrMask.value = i2cMask;
    }

    regBase->icClrSmbusIntr |= SMBUS_ARP_INTR_MASK | SMBUS_SLV_RX_PEC_NACK_BIT;
    /* Check if both SARs are being disabled */
    bool otherSarEnabled = false;
    SmbusIcEnableReg_u enableReg = {.value = regBase->icEnable.value};

    if (sarNum == 1) {
        ///< 检查 SAR 2 是否启用。假设 icSarEn 对应 SAR1, icSar2En 对应 SAR2
        if (enableReg.fields.icSar2En) {
            otherSarEnabled = true;
        }
    } else if (sarNum == 2) {
        ///< 检查 SAR 1 是否启用
        if (enableReg.fields.icSarEn) {
            otherSarEnabled = true;
        }
    }
    ///< 如果没有其他 SAR 地址仍在使用 ARP 功能
    if (!otherSarEnabled) {
        ///< 禁用 ARP 模式
        SmbusIcConReg_u con;
        con.value = regBase->icCon.value;
        con.fields.smbusArpEn = 0;
        regBase->icCon.value = con.value;

        ///< 驱动数据状态更新
        pDrvData->arpEnabled = false;
        pDrvData->arpState = SMBUS_ARP_STATE_INIT;
    }
    LOGD("%s(): ARP disabled for SAR%u\n", __func__, sarNum);

unlock:
    /* Unlock device */
    devUnlockByDriver(devId);
exit:
    return ret;
}

/* ======================================================================== */
/*                    Slave API - ARP Functions                        */
/* ======================================================================== */
/**
 * @brief Slave side ARP address assignment completion handler
 * @details Handles the completion of ARP address assignment on the slave side.
 *          This function is called when an ARP Assign Address command is
 *          detected and processes the address assignment result for both SAR1
 *          and SAR2 registers. It validates the address resolution and
 *          notifies the system of successful address assignment.
 * @param[in] pDrvData Pointer to the SMBus driver private data structure
 * @return void
 *
 * @note Handles both SAR1 and SAR2 address resolution
 * @note Waits 200ms for address assignment to settle
 * @note Calls registered ARP failure handler if available
 * @note Logs assigned addresses for debugging
 * @warning This function should only be called in Slave mode
 * @warning Function modifies interrupt mask registers
 */
void smbusSlvArpAssignAddrFinishHandle(SmbusDrvData_s *pDrvData)
{
    volatile SmbusRegMap_s *regBase;
    U32 intrStatus;
    U32 icStatus;
    U32 intrMask;
    U16 assignedAddr;
    SmbusArpDev_s arpDev;

    /* Parameter validation */
    SMBUS_CHECK_PARAM_VOID(pDrvData == NULL || pDrvData->sbrCfg.regAddr == NULL,
                          "%s(): pDrvData or pDrvData->sbrCfg.regAddr is NULL", __func__);

    regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;

    /* Read SMBus interrupt status register */
    intrStatus = regBase->icSmbusIntrStat.value;
    
    if (intrStatus == 0) {
        return;  /* No SMBus interrupts pending */
    }

    LOGD("SMBus ARP Handler: intrStatus=0x%08x\n", intrStatus);

    /* ===== Handle ARP RESET Command ===== */
    if (intrStatus & SMBUS_ARP_RST_CMD_DET_BIT) {
        LOGI("SMBus ARP: RESET command detected\n");
        
        /* Clear assigned address and reset ARP state */
        pDrvData->arpState = SMBUS_ARP_STATE_RESET;
        
        /* Notify application of ARP reset event */
        memset(&arpDev, 0, sizeof(arpDev));
        memcpy(&arpDev.udid, &pDrvData->udid, sizeof(SmbusUdid_s));
        smbusArpNotifyEvent(&pDrvData->arpNotifyEvent, SMBUS_ARP_EVENT_RESET, &arpDev);
        
        /* Clear interrupt */
        regBase->icClrSmbusIntr |= SMBUS_ARP_RST_CMD_DET_BIT;
    }

    /* ===== Handle ARP PREPARE Command ===== */
    if (intrStatus & SMBUS_ARP_PREPARE_CMD_DET_BIT) {
        LOGI("SMBus ARP: PREPARE command detected\n");
        
        /* Update ARP state */
        pDrvData->arpState = SMBUS_ARP_STATE_PREPARED;
        
        /* Notify application of prepare event */
        memset(&arpDev, 0, sizeof(arpDev));
        memcpy(&arpDev.udid, &pDrvData->udid, sizeof(SmbusUdid_s));
        smbusArpNotifyEvent(&pDrvData->arpNotifyEvent, SMBUS_ARP_EVENT_PREPARE, &arpDev);
        
        /* Clear interrupt */
        regBase->icClrSmbusIntr |= SMBUS_ARP_PREPARE_CMD_DET_BIT;
    }

    /* ===== Handle ARP GET UDID Command ===== */
    if (intrStatus & SMBUS_GET_UDID_CMD_DET_BIT) {
        LOGI("SMBus ARP: GET_UDID command detected\n");
        
        /* Prepare UDID response (handled by hardware, but notify application) */
        memset(&arpDev, 0, sizeof(arpDev));
        memcpy(&arpDev.udid, &pDrvData->udid, sizeof(SmbusUdid_s));
        smbusArpNotifyEvent(&pDrvData->arpNotifyEvent, SMBUS_ARP_EVENT_GET_UDID, &arpDev);
        
        /* Clear interrupt */
        regBase->icClrSmbusIntr |= SMBUS_GET_UDID_CMD_DET_BIT;
    }

    /* ===== Handle ARP ASSIGN ADDRESS Command ===== */
    if (intrStatus & SMBUS_ASSIGN_ADDR_CMD_DET_BIT) {
        LOGI("SMBus ARP: ASSIGN_ADDR command detected\n");
        
        /* Temporarily disable ASSIGN_ADDR interrupt to prevent re-entry */
        intrMask = regBase->icSmbusIntrMask.value;
        intrMask &= ~SMBUS_ASSIGN_ADDR_CMD_DET_BIT;
        regBase->icSmbusIntrMask.value = intrMask;
        
        /* Clear the interrupt first */
        regBase->icClrSmbusIntr |= SMBUS_ASSIGN_ADDR_CMD_DET_BIT;
        
        /* Small delay to allow address assignment to settle (per DW_apb_i2c spec) */
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(20));
        
        /* Read IC_STATUS register to check address assignment status */
        icStatus = regBase->icStatus.value;
        
        LOGD("SMBus ARP: IC_STATUS=0x%08x after ASSIGN_ADDR\n", icStatus);
        
        /* Initialize ARP device structure */
        memset(&arpDev, 0, sizeof(arpDev));
        memcpy(&arpDev.udid, &pDrvData->udid, sizeof(SmbusUdid_s));
        arpDev.oldAddr = 0xFFFF;  /* No old address in this context */
        
        /* Check SAR (Primary Slave Address) assignment status */
        /* Bit 17: SMBUS_SLAVE_ADDR_VALID, Bit 18: SMBUS_SLAVE_ADDR_RESOLVED */
        if ((icStatus & (1U << 17)) && (icStatus & (1U << 18))) {
            /* Read assigned address from IC_SAR register */
            assignedAddr = (U16)(regBase->icSar.value & 0x3FF);
            
            LOGI("SMBus ARP: Primary address (SAR) assigned: 0x%03x\n", assignedAddr);
            
            /* Update device structure */
            arpDev.newAddr = assignedAddr;
            arpDev.addrValid = 1;
            
            /* Update driver state */
            pDrvData->arpState = SMBUS_ARP_STATE_ASSIGNED;
            pDrvData->assignedAddr = assignedAddr;
            
            /* Notify application of address assignment */
            smbusArpNotifyEvent(&pDrvData->arpNotifyEvent, 
                               SMBUS_ARP_EVENT_ADDRESS_CHANGED, 
                               &arpDev);
        }
        
        /* Check SAR2 (Secondary Slave Address) assignment status */
        /* Note: Based on typical DW_apb_i2c implementation, 
         * bits for SAR2 would be different - verify in your datasheet */
        /* Assuming bits 21-22 per your original code */
        if ((icStatus & (1U << 21)) && (icStatus & (1U << 22))) {
            /* Read assigned address from IC_SAR2 register */
            assignedAddr = (U16)(regBase->icSar2.value & 0x3FF);
            
            LOGI("SMBus ARP: Secondary address (SAR2) assigned: 0x%03x\n", assignedAddr);
            
            /* Update device structure for SAR2 */
            memset(&arpDev, 0, sizeof(arpDev));
            memcpy(&arpDev.udid, &pDrvData->udid, sizeof(SmbusUdid_s));
            arpDev.newAddr = assignedAddr;
            arpDev.addrValid = 1;
            arpDev.oldAddr = 0xFFFF;
            
            /* Notify application of SAR2 address assignment */
            smbusArpNotifyEvent(&pDrvData->arpNotifyEvent, 
                               SMBUS_ARP_EVENT_ADDRESS_CHANGED, 
                               &arpDev);
        }
        
        /* Re-enable ASSIGN_ADDR interrupt */
        intrMask = regBase->icSmbusIntrMask.value;
        intrMask |= SMBUS_ASSIGN_ADDR_CMD_DET_BIT;
        regBase->icSmbusIntrMask.value = intrMask;
    }

    /* ===== Handle Host Notify Master Detected ===== */
    if (intrStatus & SMBUS_HOST_NOTIFY_MST_DET_BIT) {
        LOGI("SMBus: HOST_NOTIFY_MST detected\n");
        regBase->icClrSmbusIntr |= SMBUS_HOST_NOTIFY_MST_DET_BIT;
    }

    /* ===== Handle Quick Command Detected ===== */
    if (intrStatus & SMBUS_QUICK_CMD_DET_BIT) {
        LOGI("SMBus: QUICK_CMD detected\n");
        regBase->icClrSmbusIntr |= SMBUS_QUICK_CMD_DET_BIT;
    }

    /* ===== Handle SMBus Alert Detected ===== */
    if (intrStatus & SMBUS_ALERT_DET_BIT) {
        LOGI("SMBus: ALERT detected\n");
        regBase->icClrSmbusIntr |= SMBUS_ALERT_DET_BIT;
    }

    /* ===== Handle SMBus Suspend Detected ===== */
    if (intrStatus & SMBUS_SUSPEND_DET_BIT) {
        LOGI("SMBus: SUSPEND detected\n");
        regBase->icClrSmbusIntr |= SMBUS_SUSPEND_DET_BIT;
    }

    /* ===== Handle Slave RX PEC NACK ===== */
    if (intrStatus & SMBUS_SLV_RX_PEC_NACK_BIT) {
        LOGE("SMBus: Slave RX PEC NACK detected - PEC error\n");
        regBase->icClrSmbusIntr |= SMBUS_SLV_RX_PEC_NACK_BIT;
    }

    /* ===== Handle Clock Extend Timeout ===== */
    if (intrStatus & SMBUS_SLV_CLOCK_EXTND_TIMEOUT_BIT) {
        LOGE("SMBus: Slave clock extend timeout\n");
        regBase->icClrSmbusIntr |= SMBUS_SLV_CLOCK_EXTND_TIMEOUT_BIT;
    }

    if (intrStatus & SMBUS_MST_CLOCK_EXTND_TIMEOUT_BIT) {
        LOGE("SMBus: Master clock extend timeout\n");
        regBase->icClrSmbusIntr |= SMBUS_MST_CLOCK_EXTND_TIMEOUT_BIT;
    }
}

/**
 * @brief Set UDID information for the specified index
 * @details Sets the UDID information to the SMBus controller according to
 *          SMBus 3.1 protocol. UDID contains 16-byte unique device identifier
 *          used for device identification in ARP protocol.
 * @param[in] devId SMBus device ID
 * @param[in] idx UDID index (supports multiple UDID storage)
 * @param[in] udidInfo Pointer to UDID information
 * @return 0 on success, negative error code on failure
 *
 * @note Writes UDID information to hardware registers IC_SMBUS_UDID_WORD0-3
 * @note Supports complete 16-byte UDID data write
 * @note Performs parameter validation and device locking
 * @warning idx parameter currently only supports 0 for main UDID storage
 * @warning Must be called after ARP initialization
 *
 */
S32 smbusArpUdidSet(DevList_e devId, U32 idx, const SmbusUdid_s *udidInfo)
{
    S32 ret = EXIT_SUCCESS;
    volatile SmbusRegMap_s *regBase = NULL;

    SMBUS_CHECK_PARAM_RETURN(udidInfo == NULL, -EINVAL,
                             "%s(): udidInfo is NULL", __func__);

    SMBUS_CHECK_PARAM_RETURN(idx > 0, -EINVAL,
                             "%s(): idx %u not supported, only idx=0 supported", __func__, idx);

    ///< Check driver lock and validate
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    ///< Convert UDID structure to 32-bit word array and write to hardware registers
    U32 *udidWords = (U32*)udidInfo;

    ///< Write UDID to hardware registers
    regBase->icSmbusUdidWord0.value = udidWords[0];
    regBase->icSmbusUdidWord1.value = udidWords[1];
    regBase->icSmbusUdidWord2.value = udidWords[2];
    regBase->icSmbusUdidWord3.value = udidWords[3];

    ///< Also save to driver data structure
    SmbusDrvData_s *pDrvData = NULL;
    if (getDevDriver(devId, (void**)&pDrvData) == EXIT_SUCCESS && pDrvData != NULL) {
        memcpy(&pDrvData->udid, udidInfo, sizeof(SmbusUdid_s));
    }

    LOGD("%s(): UDID set successfully for idx %u\n", __func__, idx);

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

/**
 * @brief Get UDID information for the specified index
 * @details Reads UDID information from the SMBus controller according to
 *          SMBus 3.1 protocol. UDID contains 16-byte unique device identifier
 *          used for device identification in ARP protocol.
 * @param[in] devId SMBus device ID
 * @param[in] idx UDID index
 * @param[out] udidInfo Output pointer to UDID information
 * @return 0 on success, negative error code on failure
 *
 * @note Reads 16-byte UDID from hardware registers IC_SMBUS_UDID_WORD0-3
 * @note Prioritizes reading from driver cache, falls back to hardware if needed
 * @note Performs complete parameter validation and error handling
 * @warning idx parameter currently only supports 0 for main UDID storage
 * @warning Caller must ensure udidInfo points to valid memory space
 *
 */
S32 smbusArpUdidGet(DevList_e devId, U32 idx, SmbusUdid_s *udidInfo)
{
    S32 ret = EXIT_SUCCESS;
    volatile SmbusRegMap_s *regBase = NULL;

    SMBUS_CHECK_PARAM_RETURN(udidInfo == NULL, -EINVAL,
                             "%s(): udidInfo is NULL", __func__);

    SMBUS_CHECK_PARAM_RETURN(idx > 0, -EINVAL,
                             "%s(): idx %u not supported, only idx=0 supported", __func__, idx);

    ///< Check driver lock and validate
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    ///< Prioritize reading UDID from driver cache
    SmbusDrvData_s *pDrvData = NULL;
    if (getDevDriver(devId, (void**)&pDrvData) == EXIT_SUCCESS && pDrvData != NULL) {
        if (pDrvData->udid.deviceAddr != 0 ||
            pDrvData->udid.vendorId != 0 ||
            pDrvData->udid.deviceId != 0) {
            memcpy(udidInfo, &pDrvData->udid, sizeof(SmbusUdid_s));
            LOGD("%s(): UDID retrieved from cache for idx %u\n", __func__, idx);
            goto unlock;
        }
    }

    ///< Read UDID from hardware registers
    U32 *udidWords = (U32*)udidInfo;

    udidWords[0] = regBase->icSmbusUdidWord0.value;
    udidWords[1] = regBase->icSmbusUdidWord1.value;
    udidWords[2] = regBase->icSmbusUdidWord2.value;
    udidWords[3] = regBase->icSmbusUdidWord3.value;

    ///< Also update driver cache
    if (pDrvData != NULL) {
        memcpy(&pDrvData->udid, udidInfo, sizeof(SmbusUdid_s));
    }

    LOGD("%s(): UDID retrieved from hardware for idx %u\n", __func__, idx);

unlock:
    devUnlockByDriver(devId);

exit:
    return ret;
}

/**
 * @brief Get address valid status for specified SAR device
 * @details Checks the valid status of the specified SAR device address according
 *          to SMBus ARP protocol. Address must satisfy both valid and resolved
 *          status to be considered usable.
 * @param[in] devId SMBus device ID
 * @param[in] sarNum SAR device number (1 or 2)
 * @return 1 if address is valid, 0 if address is invalid, negative error code on failure
 *
 * @note Reads SMBUS_SLAVE_ADDR_VALID and SMBUS_SLAVE_ADDR_RESOLVED bits from IC_STATUS register
 * @note SAR1 checks bits 17 and 18, SAR2 checks bits 21 and 22
 * @note Performs device locking for thread safety
 * @warning sarNum parameter must be 1 or 2, other values return error
 * @warning Only valid in Slave mode
 *
 * [SLAVE_API] Slave API layer - ARP status management
 */
S32 smbusArpAddrValidGetStatus(DevList_e devId, U32 sarNum)
{
    S32 ret = EXIT_SUCCESS;
    volatile SmbusRegMap_s *regBase = NULL;

    SMBUS_CHECK_PARAM(sarNum != 1 && sarNum != 2, -EINVAL,
                   "%s(): invalid sarNum %u, must be 1 or 2", __func__, sarNum);

    ///< Check driver lock and validate
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto exit;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto exit;
    }
    
    U32 status = regBase->icStatus.value;
    
    if (sarNum == 1) {
        ///< SAR1: check bit 17 (valid) and bit 18 (resolved)
        bool valid = (regBase->icStatus.fields.smbusSlaveAddrValid) != 0;
        bool resolved = (regBase->icStatus.fields.smbusSlaveAddrResolved) != 0;
        ret = (valid && resolved) ? 1 : 0;

        LOGD("%s(): SAR1 addr_valid=%d, addr_resolved=%d, status=%d\n",
             __func__, valid, resolved, ret);
    } else {
        ///< SAR2: check bit 21 (valid) and bit 22 (resolved)
        bool valid = (regBase->icStatus.fields.smbusSuspendStatus) != 0;
        bool resolved = (status & (1 << 22)) != 0;
        ret = (valid && resolved) ? 1 : 0;

        LOGD("%s(): SAR2 addr_valid=%d, addr_resolved=%d, status=%d\n",
             __func__, valid, resolved, ret);
    }

exit:
    devUnlockByDriver(devId);
    return ret;
}


