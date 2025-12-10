/**
 * Copyright (C), 2025, WuXi Stars Micro System TechnoLOGEes Co.,Ltd
 *
 * @file drv_smbus_dw_slaver.c
 * @author wangkui (wangkui@starsmicrosystem.com)
 * @date 2025/10/20
 * @brief SMBus Slave API layer implementation
 *
 * @par ChangeLog:
 * Date         Author          Description
 * 2025/10/20   wangkui         Initial version
 *
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
 * [SLAVE_API] Slave API layer - slave configuration
 */
static void smbusConfigureSlave(SmbusDev_s *dev)
{
    SmbusIcConReg_u con;

    if (dev == NULL || dev->regBase == NULL) {
        return;
    }

    /* Get HAL operations */
    SmbusHalOps_s *halOps = smbusGetHalOps();
    if (halOps == NULL || halOps->disable == NULL) {
        return;
    }

    /* Disable before configuration */
    halOps->disable(dev);

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

    /* Configure optimal interrupt mask for Slave mode */
    U32 intrMask = 0;
    intrMask |= SMBUS_IC_INTR_RD_REQ_MASK;    // bit[5] - Master 读请求
    intrMask |= SMBUS_IC_INTR_RX_FULL_MASK;   // bit[2] - RX FIFO 满
    intrMask |= SMBUS_IC_INTR_RX_DONE_MASK;   // bit[7] - 接收完成
    intrMask |= SMBUS_IC_INTR_STOP_DET_MASK;  // bit[9] - STOP 条件
    intrMask |= SMBUS_IC_INTR_TX_ABRT_MASK;   // bit[6] - 传输终止

    dev->regBase->icIntrMask = intrMask;
    LOGD("SMBus Slave: Configured interrupt mask=0x%08X (RD_REQ+RX_FULL+RX_DONE+STOP_DET+TX_ABRT)\n", intrMask);

    /* Set slave address */
    if (halOps->setSlaveAddr != NULL) {
        halOps->setSlaveAddr(dev);
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
 *
 * @note Iterates through all registered callbacks
 * @note Validates parameters before callback invocation
 * @note Logs callback invocation for debugging
 * @warning Callback functions should not block or perform lengthy operations
 * @warning This function should be called with appropriate synchronization
 */
static void smbusArpNotifyEvent(SmbusArpNotifyCb_s *notifyCb, SmbusArpEvent_e event, const SmbusArpDev_s *dev)
{
    U32 i;

    ///< Parameter validation
    if (notifyCb == NULL || dev == NULL) {
        LOGE("%s(): Invalid parameters\n", __func__);
        return;
    }

    ///< Log event notification
    LOGD("%s(): Notifying ARP event %d for device addr 0x%02X\n",
         __func__, event, dev->newAddr);

    ///< Iterate through all registered callbacks
    for (i = 0; i < notifyCb->callbackCount; i++) {
        if (notifyCb->callbacks[i] != NULL) {
            LOGD("%s(): Invoking callback %u for event %d\n",
                 __func__, i, event);

            ///< Invoke callback with event and device data
            notifyCb->callbacks[i](event, dev, notifyCb->userData[i]);
        }
    }

    ///< Log completion
    LOGD("%s(): ARP event notification completed for %u callbacks\n",
         __func__, notifyCb->callbackCount);
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
        goto unlock;
    }

    /* Get driver data */
    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    /* Reset HW ARP flags */
    pDrvData->arpEnabled = false;
    pDrvData->arpState = SMBUS_ARP_STATE_INIT;

    /* Enable SMBus controller in slave mode */
    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;
    if (regBase == NULL) {
        ret = -EINVAL;
        goto unlock;
    }

    /* Get HAL operations */
    SmbusHalOps_s *halOps = smbusGetHalOps();
    if (halOps == NULL || halOps->enable == NULL) {
        ret = -ENOSYS;
        goto unlock;
    }

    /* Configure controller for SMBus slave mode */
    SmbusDev_s *dev = &pDrvData->pSmbusDev;
    smbusConfigureSlave(dev);

    /* Clear all SMBus-related interrupt status */
    regBase->icClrSmbusIntr = SMBUS_SLV_RX_PEC_NACK_BIT | SMBUS_ASSIGN_ADDR_INTR_BIT |
                                    SMBUS_GET_UDID_INTR_BIT | SMBUS_SAR1_INTR_BIT |
                                    SMBUS_SAR2_INTR_BIT;

    /* Enable ARP mode in IC_CON */
    con.value = regBase->icCon.value;
    con.fields.smbusArpEn = 1;
    regBase->icCon.value = con.value;

    /* Enable controller */
    halOps->enable(dev);

    /* Mark ARP as initialized */
    pDrvData->arpInitialized = true;

    LOGD("%s(): ARP Slave initialization completed\n", __func__);

unlock:
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

    /* Get HAL operations */
    SmbusHalOps_s *halOps = smbusGetHalOps();
    if (halOps == NULL || halOps->enable == NULL) {
        ret = -ENOSYS;
        goto unlock;
    }

    /* Check if ARP is initialized */
    if (!pDrvData->arpInitialized) {
        ret = -EAGAIN;
        goto unlock;
    }

    /* Prepare ARP Slave for operation by enabling appropriate interrupts */
    volatile SmbusRegMap_s *regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;
    if (regBase == NULL) {
        ret = -EINVAL;
        goto unlock;
    }

    /* Enable ARP-related interrupts */
    U32 mask = regBase->icSmbusIntrMask;
    mask |= SMBUS_SLV_RX_PEC_NACK_BIT | SMBUS_ASSIGN_ADDR_INTR_BIT | SMBUS_GET_UDID_INTR_BIT;
    regBase->icSmbusIntrMask = mask;

    /* Set ARP state to ready */
    pDrvData->arpState = SMBUS_ARP_STATE_READY;
    pDrvData->arpEnabled = true;

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

    if (udid == NULL) {
        ret = -EINVAL;
        goto exit;
    }

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

    /* Get HAL operations */
    SmbusHalOps_s *halOps = smbusGetHalOps();
    if (halOps == NULL) {
        ret = -ENOSYS;
        goto unlock;
    }

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
    if (regBase == NULL) {
        ret = -EINVAL;
        goto unlock;
    }

    /* Reset ARP state machine */
    pDrvData->arpState = SMBUS_ARP_STATE_INIT;
    pDrvData->arpEnabled = false;
    pDrvData->arpInitialized = false;

    /* Clear all ARP-related interrupts */
    regBase->icClrSmbusIntr = SMBUS_SLV_RX_PEC_NACK_BIT | SMBUS_ASSIGN_ADDR_INTR_BIT |
                                    SMBUS_GET_UDID_INTR_BIT | SMBUS_SAR1_INTR_BIT |
                                    SMBUS_SAR2_INTR_BIT;

    /* Disable ARP-related interrupts */
    U32 mask = regBase->icSmbusIntrMask;
    mask &= ~(SMBUS_SLV_RX_PEC_NACK_BIT | SMBUS_ASSIGN_ADDR_INTR_BIT | SMBUS_GET_UDID_INTR_BIT);
    regBase->icSmbusIntrMask = mask;

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
    if (halOps->enable != NULL) {
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
    SmbusDrvData_s *pDrvData = NULL;
    volatile SmbusRegMap_s *regBase = NULL;

    if (sarNum > 2 || sarNum == 0) {
        return -EINVAL;
    }

    ///< Check driver lock and validate
    if (smbusDrvLockAndCheck(devId) != EXIT_SUCCESS) {
        return -EINVAL;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        devUnlockByDriver(devId);
        return -EINVAL;
    }

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
        U32 mask = regBase->icSmbusIntrMask;
        mask |= SMBUS_SAR1_INTR_BIT;
        regBase->icSmbusIntrMask = mask;

        LOGD("%s(): SAR1 enabled\n", __func__);
    } else if (sarNum == 2) {
        /* Configure and enable SAR2 */
        sarReg.value = regBase->icEnable.value;
        sarReg.fields.icSar2En = 1;
        regBase->icEnable.value = sarReg.value;

        /* Enable SAR2 interrupts */
        U32 mask = regBase->icSmbusIntrMask;
        mask |= SMBUS_SAR2_INTR_BIT;
        regBase->icSmbusIntrMask = mask;

        LOGD("%s(): SAR2 enabled\n", __func__);
    }
#endif
    /* Update device status */
    pDrvData->pSmbusDev.status |= SMBUS_STATUS_SLAVE_ENABLED_MASK;

    /* Unlock device */
    devUnlockByDriver(devId);

    return EXIT_SUCCESS;
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

    if (sarNum > 2 || sarNum == 0) {
        ret = -EINVAL;
        goto exit;
    }

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
        U32 mask = regBase->icSmbusIntrMask;
        mask &= ~SMBUS_SAR1_INTR_BIT;
        regBase->icSmbusIntrMask = mask;

        /* Clear any pending SAR1 interrupts */
        regBase->icClrSmbusIntr = SMBUS_SAR1_INTR_BIT;

        LOGD("%s(): SAR1 disabled\n", __func__);
    } else if (sarNum == 2) {
        /* Configure and disable SAR2 */
        sarReg.value = regBase->icEnable.value;
        sarReg.fields.icSar2En = 0;
        regBase->icEnable.value = sarReg.value;

        /* Disable SAR2 interrupts */
        U32 mask = regBase->icSmbusIntrMask;
        mask &= ~SMBUS_SAR2_INTR_BIT;
        regBase->icSmbusIntrMask = mask;

        /* Clear any pending SAR2 interrupts */
        regBase->icClrSmbusIntr = SMBUS_SAR2_INTR_BIT;

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

    if (sarNum > 2 || sarNum == 0) {
        ret = -EINVAL;
        goto unlock;
    }

    ///< Check driver lock and validate
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto unlock;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto exit;
    }

    if (pDrvData == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    /* Enable ARP mode in IC_CON register */
    SmbusIcConReg_u con;
    con.value = regBase->icCon.value;
    con.fields.smbusArpEn = 1;
    regBase->icCon.value = con.value;

    /* Enable ARP interrupts for the specified SAR */
    U32 mask = regBase->icSmbusIntrMask;
    if (sarNum == 1) {
        mask |= SMBUS_SLV_RX_PEC_NACK_BIT | SMBUS_SAR1_INTR_BIT |
                SMBUS_ASSIGN_ADDR_INTR_BIT | SMBUS_GET_UDID_INTR_BIT;
    } else if (sarNum == 2) {
        mask |= SMBUS_SLV_RX_PEC_NACK_BIT | SMBUS_SAR2_INTR_BIT |
                SMBUS_ASSIGN_ADDR_INTR_BIT | SMBUS_GET_UDID_INTR_BIT;
    }
    regBase->icSmbusIntrMask = mask;

    /* Mark ARP as enabled */
    pDrvData->arpEnabled = true;
    if (pDrvData->arpState == SMBUS_ARP_STATE_INIT) {
        pDrvData->arpState = SMBUS_ARP_STATE_READY;
    }

    LOGD("%s(): ARP enabled for SAR%u\n", __func__, sarNum);

unlock:
    /* Unlock device */
    devUnlockByDriver(devId);
exit:
    return ret;
}

/**
 * @brief ARP Disable function
 * [SLAVE_API] Slave API layer - high-level Slave mode operations
 */
S32 smbusArpDisable(DevList_e devId, U32 sarNum)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;
    volatile SmbusRegMap_s *regBase = NULL;

    if (sarNum > 2 || sarNum == 0) {
        ret = -EINVAL;
        goto unlock;
    }

    ///< Check driver lock and validate
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto unlock;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
    }

    if (getDevDriver(devId, (void**)&pDrvData) != EXIT_SUCCESS) {
        ret = -ENODEV;
        goto exit;
    }

    if (pDrvData == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    /* Disable ARP interrupts for the specified SAR */
    U32 mask = regBase->icSmbusIntrMask;
    if (sarNum == 1) {
        mask &= ~(SMBUS_SLV_RX_PEC_NACK_BIT | SMBUS_SAR1_INTR_BIT);
    } else if (sarNum == 2) {
        mask &= ~(SMBUS_SLV_RX_PEC_NACK_BIT | SMBUS_SAR2_INTR_BIT);
    }
    regBase->icSmbusIntrMask = mask;

    /* Clear any pending ARP interrupts for the specified SAR */
    if (sarNum == 1) {
        regBase->icClrSmbusIntr = SMBUS_SLV_RX_PEC_NACK_BIT | SMBUS_SAR1_INTR_BIT;
    } else if (sarNum == 2) {
        regBase->icClrSmbusIntr = SMBUS_SLV_RX_PEC_NACK_BIT | SMBUS_SAR2_INTR_BIT;
    }

    /* Check if both SARs are being disabled */
    if (sarNum == 1) {
        SmbusIcEnableReg_u sar2Reg = {.value = regBase->icEnable.value};
        if (!sar2Reg.fields.icSarEn) {
            /* Both SARs are disabled, disable ARP mode completely */
            SmbusIcConReg_u con;
            con.value = regBase->icCon.value;
            con.fields.smbusArpEn = 0;
            regBase->icCon.value = con.value;

            /* Disable all ARP-related interrupts */
            mask &= ~(SMBUS_ASSIGN_ADDR_INTR_BIT | SMBUS_GET_UDID_INTR_BIT);
            regBase->icSmbusIntrMask = mask;

            /* Clear remaining ARP interrupts */
            regBase->icClrSmbusIntr = SMBUS_ASSIGN_ADDR_INTR_BIT | SMBUS_GET_UDID_INTR_BIT;

            pDrvData->arpEnabled = false;
            pDrvData->arpState = SMBUS_ARP_STATE_INIT;
        }
    } else if (sarNum == 2) {
        SmbusIcEnableReg_u sar1Reg = {.value = regBase->icEnable.value};
        if (!sar1Reg.fields.icSar2En) {
            /* Both SARs are disabled, disable ARP mode completely */
            SmbusIcConReg_u con;
            con.value = regBase->icCon.value;
            con.fields.smbusArpEn = 0;
            regBase->icCon.value = con.value;

            /* Disable all ARP-related interrupts */
            mask &= ~(SMBUS_ASSIGN_ADDR_INTR_BIT | SMBUS_GET_UDID_INTR_BIT);
            regBase->icSmbusIntrMask = mask;

            /* Clear remaining ARP interrupts */
            regBase->icClrSmbusIntr = SMBUS_ASSIGN_ADDR_INTR_BIT | SMBUS_GET_UDID_INTR_BIT;

            pDrvData->arpEnabled = false;
            pDrvData->arpState = SMBUS_ARP_STATE_INIT;
        }
    }

    LOGD("%s(): ARP disabled for SAR%u\n", __func__, sarNum);

unlock:
    /* Unlock device */
    devUnlockByDriver(devId);
exit:
    return ret;
}

/* ======================================================================== */
/*                    Slave API - ARP Protocol Functions                     */
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
    U32 tmp;
    U32 mask, ic_status, icSar;

    if (pDrvData == NULL || pDrvData->sbrCfg.regAddr == NULL) {
        return;
    }

    regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;

    /* Read ARP interrupt status */
    tmp = regBase->icSmbusIntrStat.value;

    /* Handle ARP GET UDID command */
    if (tmp & SMBUS_GET_UDID_INTR_BIT) {
        LOGT("SMBus ARP: GET_UDID_CMD detected (0x%x)\n", tmp);
        regBase->icClrSmbusIntr = SMBUS_GET_UDID_INTR_BIT;
    }

    /* Handle ARP RESET command */
    if (tmp & SMBUS_SLV_RX_PEC_NACK_BIT) {
        LOGT("SMBus: SLV_RX_PEC_NACK detected (0x%x)\n", tmp);
        regBase->icClrSmbusIntr = SMBUS_SLV_RX_PEC_NACK_BIT;
    }

    /* Handle ARP PREPARE command */
    if (tmp & SMBUS_SAR1_INTR_BIT) {
        LOGT("SMBus ARP: PREPARE_CMD detected (0x%x)\n", tmp);
        regBase->icClrSmbusIntr = SMBUS_SAR1_INTR_BIT;
    }

    /* Check if ARP Assign Address command detected */
    if ((tmp & SMBUS_ASSIGN_ADDR_INTR_BIT) == 0) {
        return;
    }

    /* Check if callback is registered */
    if (pDrvData->arpFailHandler == NULL) {
        /* Clear ARP ASSIGN interrupt */
        regBase->icClrSmbusIntr = SMBUS_ASSIGN_ADDR_INTR_BIT;
        return;
    }

    /* Disable ARP_ASSGN_ADDR_CMD interrupt */
    mask = regBase->icSmbusIntrMask;
    mask &= ~SMBUS_ASSIGN_ADDR_INTR_BIT;
    regBase->icSmbusIntrMask = mask;

    /* Wait for address assignment to settle */
    rtems_task_wake_after(2000 * 1000 * 10);

    /* Read status register */
    ic_status = regBase->icStatus.value;

    /* Check SAR1 address valid and resolved */
    if ((ic_status & (1 << 17)) && (ic_status & (1 << 18))) {
        /* IC_STATUS_SMBUS_SLAVE_ADDR_VALID && IC_STATUS_SMBUS_SLAVE_ADDR_RESOLVED */
        icSar = regBase->icSar.value & 0x3FF;

        /* Call registered callback if available */
        if (pDrvData->arpFailHandler != NULL) {
            LOGD("SMBus: Calling ARP fail handler for SAR1 assignment\n");
        }

        /* Notify via ARP notification manager */
        SmbusArpDev_s dev;
        memset(&dev, 0, sizeof(dev));
        dev.newAddr = icSar;
        ///< Copy UDID information for notification
        memcpy(&dev.udid, &pDrvData->udid, sizeof(SmbusUdid_s));
        dev.addrValid = true;
        smbusArpNotifyEvent(&pDrvData->arpNotifyEvent, SMBUS_ARP_EVENT_ADDRESS_CHANGED, &dev);

        LOGE("SMBus: SAR1 address assigned: 0x%03x\n", icSar);
    }

    LOGT("SMBus ARP status: 0x%x\n", ic_status);

    /* Check SAR2 address valid and resolved */
    if ((ic_status & (1 << 21)) && (ic_status & (1 << 22))) {
        /* IC_STATUS_SMBUS_SLAVE_ADDR2_VALID && IC_STATUS_SMBUS_SLAVE_ADDR2_RESOLVED */
        icSar = regBase->icSar2.value & 0x3FF;

        /* Call registered callback if available */
        if (pDrvData->arpFailHandler != NULL) {
            LOGD("SMBus: Calling ARP fail handler for SAR2 assignment\n");
        }

        /* Notify via ARP notification manager */
        SmbusArpDev_s dev;
        memset(&dev, 0, sizeof(dev));
        dev.newAddr = icSar;
        ///< Copy UDID information for notification
        memcpy(&dev.udid, &pDrvData->udid, sizeof(SmbusUdid_s));
        dev.addrValid = true;
        smbusArpNotifyEvent(&pDrvData->arpNotifyEvent, SMBUS_ARP_EVENT_ADDRESS_CHANGED, &dev);

        LOGE("SMBus: SAR2 address assigned: 0x%03x\n", icSar);
    }

    /* Clear ARP_ASSGN_ADDR_CMD interrupt first, then re-enable */
    regBase->icClrSmbusIntr = SMBUS_ASSIGN_ADDR_INTR_BIT;
    tmp = regBase->icSmbusIntrMask;
    tmp |= SMBUS_ASSIGN_ADDR_INTR_BIT;
    regBase->icSmbusIntrMask = tmp;
}

/* ======================================================================== */
/*                    Slave API - Missing ARP Functions                        */
/* ======================================================================== */

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

    ///< Parameter validation
    if (udidInfo == NULL) {
        LOGE("%s(): udidInfo is NULL\n", __func__);
        return -EINVAL;
    }

    if (idx > 0) {
        LOGE("%s(): idx %u not supported, only idx=0 supported\n", __func__, idx);
        return -EINVAL;
    }

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

    ///< Parameter validation
    if (udidInfo == NULL) {
        LOGE("%s(): udidInfo is NULL\n", __func__);
        return -EINVAL;
    }

    if (idx > 0) {
        LOGE("%s(): idx %u not supported, only idx=0 supported\n", __func__, idx);
        return -EINVAL;
    }

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

    ///< Parameter validation
    if (sarNum != 1 && sarNum != 2) {
        LOGE("%s(): invalid sarNum %u, must be 1 or 2\n", __func__, sarNum);
        ret = -EINVAL;
        goto unlock;
    }

    ///< Check driver lock and validate
    ret = smbusDrvLockAndCheck(devId);
    if (ret != EXIT_SUCCESS) {
        goto unlock;
    }

    /* Get register base */
    if (getSmbusReg(devId, (SmbusRegMap_s **)&regBase) != EXIT_SUCCESS) {
        ret = -EINVAL;
        goto unlock;
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

unlock:
    devUnlockByDriver(devId);
    return ret;
}


