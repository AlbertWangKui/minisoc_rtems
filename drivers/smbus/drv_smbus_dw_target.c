/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_smbus_dw_target.c
 * @author wangkui (wangkui@starsmicrosystem.com)
 * @date 2025/10/20
 * @brief SMBus target API layer implementation
 *
 * @par ChangeLog:
 * Date         Author          Description
 * 2025/10/20   wangkui         Initial version
 * 2025/11/20   wangkui         porting marco style
 * 2025/11/23   wangkui         HAL operations optimization - eliminated redundant smbusGetHalOps() calls
 *                              Replaced direct HAL access with dev->halOps pointer for better performance
 *                              Reduced function call overhead by registering HAL ops during initialization
 * 2025/12/01   wangkui         refactor and simplify the code as for core function
 * 2025/12/11   wangkui         correct the issue in AI review
 * 2025/12/15   wangkui         replace slave with target naming
 * @note This file implements the SMBus target API layer, providing
 *       high-level target mode operations for SMBus communication.
 *       It works on top of the SMBus core layer and handles
 *       target-specific functionality including address resolution,
 *       response handling, ARP protocol support, and event processing
 *       in target mode.
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

/**
 * @brief Wait for the current SMBus transaction to complete on the bus.
 *        This function should be called after all data has been written to the TX FIFO
 *        or after all expected data has been read from the RX FIFO.
 * @param pDrvData Pointer to the driver's private data structure.
 * @return 0 on success, -ETIMEDOUT on failure.
 */
static inline S32 smbusIsBusBusy(volatile SmbusRegMap_s *regBase)
{
    SmbusIcStatusReg_u status;
    U32 timeout = SMBUS_TRANSACTION_TIMEOUT_US; ///< Timeout may need to be longer for full transaction

    while (timeout--) {
        status.value = smbusReadReg(&regBase->icStatus.value);
        ///< We wait for the 'activity' bit to be cleared, indicating the
        if (!status.fields.activity) {
            return EXIT_SUCCESS;
        }
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(10));
    }
    return -ETIMEDOUT;
}

/**
 * @brief [Internal] Update local target address
 * @details Safely updates the hardware Slave Address Register (SAR).
 *          Usually called during initialization or after ARP address assignment completion.
 * @param[in] pDrv Pointer to driver private data
 * @param[in] addr New address (7-bit)
 * @return S32 0 on success, <0 on failure
 */
static S32 smbusSetLocalAddress(SmbusDrvData_s *pDrv, U8 addr)
{
    volatile SmbusRegMap_s *regBase = pDrv->pSmbusDev.regBase;
    SmbusDev_s *pDev = &pDrv->pSmbusDev;
    U32 timeout = SMBUS_ADDR_UPDATE_TIMEOUT_CNT;

    ///< 1. Parameter validation
    SMBUS_CHECK_PARAM_RETURN( pDev->halOps == NULL || regBase == NULL ||
                            addr > SMBUS_MAX_TARGET_ADDRESS, -EINVAL,
                            "SMBus: Invalid local address 0x%02X or invalid register\n", addr);

    ///< 2. Check if currently busy (forcing modification during transfer may cause bus errors)
    if (smbusIsBusBusy(regBase)) {
        LOGW("SMBus: Warning - Changing address while bus is busy\n");
        return -EBUSY;
        ///< Strategy: Can return -EBUSY, or force continue (depending on ARP protocol requirements)
    }

    ///< 3. Disable controller (DesignWare IP requires Disable before modifying SAR)
    pDev->halOps->disable(pDev);
    while ((smbusReadReg(&regBase->icEnableStatus) & SMBUS_IC_ENABLE_STATUS_IC_EN)  && --timeout > 0) {
        ///< Wait for hardware to completely shutdown
        udelay(10);
    }
    if (timeout == 0) {
        LOGE("SMBus: Failed to disable controller for address update\n");
        return -ETIMEDOUT;
    }

    ///< 4. Update SAR (target Address Register)
    SmbusIcSarReg_u sarReg;
    sarReg.value = smbusReadReg(&regBase->icSar.value);
    sarReg.fields.icSar = addr;
    smbusWriteReg(&regBase->icSar.value, sarReg.value);

    ///< 5. Re-enable controller
    pDev->halOps->enable(pDev);

    LOGD("SMBus: Local target Address updated to 0x%02X\n", addr);
    return EXIT_SUCCESS;
}

/**
 * @brief Target-side ARP address assignment handler function
 * @details Handles target-side ARP address assignment commands. This function is called when
 *          receiving an ARP Assign Address command, parses the UDID and new address from the command,
 *          validates if the UDID matches the local device, updates the hardware address register if
 *          matched, and notifies the upper application that address assignment is complete.
 * @param[in] pDrv Pointer to SMBus driver private data structure
 * @param[in] payload Pointer to ARP Assign Address command payload data, containing target UDID and newly assigned address
 * @return void
 *
 * @note This function is only used in target mode
 * @note Address update is hardware-level, directly modifies SAR register
 * @note User callback function will be triggered after successful address assignment
 * @note Function will return silently if UDID doesn't match local device
 *
 * @par Processing Flow:
 * 1. Parse ARP command payload, extract target UDID and new address
 * 2. Compare target UDID with local UDID, confirm if command is for this device
 * 3. If UDID matches, call smbusSetLocalAddress to update hardware address
 * 4. Update driver internal state, record new ARP status and current address
 * 5. Notify application via callback function that address assignment is complete
 *
 * @warning This function assumes the passed payload format is correct
 * @warning Hardware address update operation is atomic
 * @warning Callback function will not be triggered on update failure
 *
 * @par Dependent Functions:
 * - parseAssignPacket: Parse ARP Assign Address command packet
 * - isMyUdid: Compare if UDID matches local device
 * - smbusSetLocalAddress: Update hardware slave address register
 *
 * @par Thread Safety:
 * This function is not thread-safe, should be called in SMBus interrupt context or locked environment
 */
S32 smbusArptargetHandleAssignAddr(SmbusDrvData_s *pDrv, const U8 *payload)
{
    ///< 1. Parse Payload, extract UDID and New Address
    S32 ret = EXIT_SUCCESS;
    SmbusUdid_s targetUdid;
    U8 newAddr = SMBUS_TARGET_INVALID_NEW_ADDR;
    parseAssignPacket(payload, &targetUdid, &newAddr);

    ///< 2. Check if UDID matches local device
    if (isMyUdid(&pDrv->udid, &targetUdid)) {

        ///< 3. Core step: Directly call internal function to modify hardware address
        ret = smbusSetLocalAddress(pDrv, newAddr);

        if (ret == EXIT_SUCCESS) {
            ///< 4. Update internal state
            pDrv->arpState = SMBUS_ARP_EVENT_ASSIGN;
            pDrv->assignedAddr = newAddr;
            LOGI("SMBus ARP: Assigned new address 0x%02X for matching UDID\n", newAddr);
        }
    }
    return ret;
}

/**
 * @brief Logic specifically for handling ARP Reset interrupts
 * @note Reuses smbusSetLocalAddress but doesn't parse Payload
 */
static void smbustargetHandleArpReset(SmbusDrvData_s *pDrv)
{
    SmbusArpPayload_s arpNotifyDev;

    LOGI("SMBus target: Handling ARP Reset...\n");
    {
        /* 2. Correctly update software state machine (distinguished from Assign) */
        pDrv->arpState = SMBUS_ARP_STATE_RESET; // or SMBUS_ARP_STATE_DEFAULT
        pDrv->assignedAddr = SMBUS_ARP_DEFAULT_ADDR;
        
        LOGI("SMBus target: Address reset to Default (0x%02X)\n", SMBUS_ARP_DYNAMIC_ADDR);

        /* 4. Notify upper application (construct corresponding Event data structure) */
        memset(&arpNotifyDev, 0, sizeof(arpNotifyDev));

        /* Fill current device UDID */
        memcpy(&arpNotifyDev.udid, &pDrv->udid, sizeof(SmbusUdid_s));

        /* In Reset event, newAddr is usually filled with 0x61 or 0 to indicate unassigned */
        arpNotifyDev.newAddr = SMBUS_ARP_DEFAULT_ADDR;
        arpNotifyDev.oldAddr = SMBUS_TARGET_RESET_OLD_ADDR; // indicates reset
        arpNotifyDev.addrValid = 0;    // address is no longer in "assigned" valid state

        /* Send RESET event */
        smbusTriggerTargetEvent(pDrv, SMBUS_ARP_EVENT_RESET, &arpNotifyDev, sizeof(arpNotifyDev));
    } 
    return;
}

/* ======================================================================== */
/*                    target API - ARP Functions                        */
/* ======================================================================== */
/**
 * @brief target side ARP address assignment completion handler
 * @details Handles the completion of ARP address assignment on the target side.
 *          This function is called when an ARP Assign Address command is
 *          detected and processes the address assignment result for both SAR1
 *          and SAR2 registers. It validates the address resolution and
 *          notifies the system of successful address assignment.
 * @param[in] pDrvData Pointer to the SMBus driver private data structure
 * @return void
 * @note Calls registered ARP failure handler if available
 * @note Logs assigned addresses for debugging
 * @warning This function should only be called in target mode
 * @warning Function modifies interrupt mask registers
 */
void smbusTarArpAssignAddrFinishHandle(SmbusDrvData_s *pDrvData)
{
    volatile SmbusRegMap_s *regBase;
    U32 icStatus;
    U16 assignedAddr;
    SmbusArpPayload_s arpPayload;

    /* Parameter validation */
    SMBUS_CHECK_PARAM_VOID(pDrvData == NULL || pDrvData->sbrCfg.regAddr == NULL,
                          "pDrvData is NULL or regAddr is NULL");

    regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;
    /* * At this point, the ISR has already detected SMBUS_ASSIGN_ADDR_CMD_DET_BIT.
     * We need to verify if the hardware successfully updated its address.
     */

    /* Read IC_STATUS to check ARP resolution status */
    icStatus = smbusReadReg(&regBase->icStatus.value);

    /* * Check if the target Address has been Resolved.
     * Bit 17: SMBUS_TARGET_ADDR_VALID - Hardware has a valid address
     * Bit 18: SMBUS_TARGET_ADDR_RESOLVED - ARP process successfully resolved an address
     */
    bool addrResolved = (icStatus & (1U << 18)) != 0;
    bool addrValid = (icStatus & (1U << 17)) != 0;

    if (addrResolved && addrValid) {
        /* Success! Hardware accepted the Assign Address command */

        /* Read the new address from the target Address Register (IC_SAR) */
        assignedAddr = (U16)(smbusReadReg(&regBase->icSar.value) & SMBUS_TENBIT_ADDRESS_MASK); ///< Mask to 10 bits just in case

        LOGI("SMBus target: ARP Address Assigned! New Address: 0x%02X\n", assignedAddr);

        /* Update Driver State */
        pDrvData->arpState = SMBUS_ARP_STATE_ASSIGNED;
        pDrvData->assignedAddr = (U8)assignedAddr;

        /* Prepare Notification Payload */
        memset(&arpPayload, 0, sizeof(arpPayload));
        memcpy(&arpPayload.udid, &pDrvData->udid, sizeof(SmbusUdid_s));
        arpPayload.newAddr = (U8)assignedAddr;
        arpPayload.addrValid = 1;  
        
        /* Notify Application */
        smbusTriggerTargetEvent(pDrvData, 
                               SMBUS_ARP_EVENT_ASSIGN, 
                               &arpPayload, sizeof(arpPayload));

    } else {
        /* * The interrupt triggered, but the status bits don't show resolution.
         * This could happen if the Assign Address command was for a DIFFERENT UDID.
         * In that case, we remain at the default address or our previous state.
         */
        LOGD("SMBus target: ASSIGN_ADDR interrupt received, but not for me (Status: 0x%08X)\n", icStatus);
    }

    /* * Clear the interrupt bit specifically for Assign Address.
     * Note: In the main ISR, we might clear all bits at the end,
     * but clearing it here explicitly is safe practice for specific handlers.
     */
    smbusWriteReg(&regBase->icClrSmbusIntr, SMBUS_ASSIGN_ADDR_CMD_DET_BIT);
}
/**
 * @brief Trigger target event callback
 * @details Triggers appropriate SMBus callback for target events
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] eventType Event type to trigger
 * @param[in] data Event data pointer
 * @param[out] len Data length
 * @return void
 */
void smbusTriggerTargetEvent(SmbusDrvData_s *pDrvData, U32 eventType, void *data, U32 len)
{
    SMBUS_CHECK_PARAM_VOID(pDrvData == NULL, "pDrvData is NULL");

    DevList_e devId = pDrvData->devId;
    SmbusEventData_u eventData;
    bool triggerCallback = true; ///< Default to trigger user callback
    memset(&eventData, 0, sizeof(SmbusEventData_u));

    switch (eventType) {
        
        /* ==========================================================
         * Case 1: ARP Assign Address (Integrated handling logic)
         * ========================================================== */
        case SMBUS_ARP_EVENT_ASSIGN: 
            /* Data length validation: UDID(16) + Addr(1) = 17 bytes */
            if (data == NULL || len < 17) {
                LOGW("SMBus ARP: Invalid payload len %d\n", len);
                return; 
            }
            S32 ret = smbusArptargetHandleAssignAddr(pDrvData, (const U8 *)data);
            if (ret != EXIT_SUCCESS) {
                LOGE("SMBus ARP: Failed to set hardware address 0x%02X\n", pDrvData->assignedAddr);
                return;
            }
            LOGI("SMBus ARP: Assigned new address 0x%02X\n", pDrvData->assignedAddr);
            /* 5. Prepare callback data */
            eventData.arp.newAddr = pDrvData->assignedAddr;
            memcpy((void *)&eventData.arp.udid, (const void *)&pDrvData->udid, sizeof(SmbusUdid_s));
            break;
        case SMBUS_ARP_EVENT_RESET:
            /* Reset received, hardware address may have been reset to 0x61 (handled by caller or here) */
            /* Mainly responsible for notifying upper layer */
            eventData.arp.newAddr = SMBUS_ARP_DEFAULT_ADDR; /* Default */
            memcpy((void *)&eventData.arp.udid, (const void *)&pDrvData->udid, sizeof(SmbusUdid_s));
            break;
        case SMBUS_ARP_EVENT_GET_UDID:
            /* Only notify upper layer that UDID was read */
            memcpy((void *)&eventData.arp.udid, (const void *)&pDrvData->udid, sizeof(SmbusUdid_s));
            break;
        /* ==========================================================
         * Case 2: target Read Request
         * ========================================================== */
        case SMBUS_EVENT_TARGET_READ_REQ:
            eventData.targetReq.flags = SMBUS_M_RD;
            eventData.targetReq.data  = NULL;
            eventData.targetReq.len   = 0;
            /* Default state setting, waiting for user to call smbusTargetSetResponse in callback to modify */
            break;

        /* ==========================================================
         * Case 3: target Write Request / Data Transfer
         * ========================================================== */
        case SMBUS_EVENT_TARGET_WRITE_REQ:
            LOGD("SMBUS_EVENT_TARGET_WRITE_REQ OCCURE:%d\r\n", eventData.targetReq.len);
            eventData.targetReq.flags = 0;
            eventData.targetReq.data  = (U8 *)data;
            eventData.targetReq.len   = len;
            break;

        case SMBUS_EVENT_TX_DONE:
        case SMBUS_EVENT_RX_DONE:
        case SMBUS_EVENT_TARGET_DONE:
            eventData.transfer.buffer = (U8 *)data;
            eventData.transfer.len    = len;
            break;

        /* ==========================================================
         * Case 4: Errors
         * ========================================================== */
        case SMBUS_EVENT_PEC_ERROR:
        case SMBUS_EVENT_ERROR:
            eventData.error.errorCode = eventType;
            eventData.error.statusReg = (U32)(uintptr_t)data;
            break;

        /* Other events... */
        default:
            /* For events that don't need special data packing, try to convert data pointer */
             if (data != NULL && len <= 4) {
                eventData.value = (U32)(uintptr_t)data;
            }
            break;
    }
    if (triggerCallback && pDrvData->callback) {
        pDrvData->callback(devId, eventType, &eventData, pDrvData->userData);
    }
}
/**
 * @brief Handle target specific SMBus interrupts
 * @details Processes SMBus-specific interrupts in target mode
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @param[in] smbusIntrStat SMBus interrupt status
 * @return void
 */
void smbusHandleTargetSpecificInterrupts(SmbusDrvData_s *pDrvData,
                                        volatile SmbusRegMap_s *regBase,
                                        U32 smbusIntrStat)
{
    SmbusArpPayload_s arpPayload;
    /* 1. Handle Quick Command */
    if (smbusIntrStat & SMBUS_QUICK_CMD_DET_BIT) {
        LOGD("SMBus target: Quick Command Detected\n");
        /* Clear interrupt immediately */
        regBase->icClrSmbusIntr = SMBUS_QUICK_CMD_DET_BIT;
        
        /* Trigger event notification to upper layer. Note: Hardware can hardly distinguish Quick Read/Write,
         * usually determined by whether there is subsequent data flow, or directly notify upper layer "was pinged" */
        smbusTriggerTargetEvent(pDrvData, SMBUS_EVENT_QUICK_CMD, NULL, 0);
    }

    /* 2. Handle ARP Reset (Master broadcast reset) - High priority */
    if (smbusIntrStat & SMBUS_ARP_RST_CMD_DET_BIT) {
        LOGI("SMBus target: ARP Reset Command Detected!\n");
        
        /* [Key operation] Hardware usually automatically resets address to 0x61 (Default),
           but software layer state (such as UDID association state, flags in pDrvData) needs to be reset */
        if (pDrvData->pSmbusDev.smbFeatures.arpEnb) {
            smbustargetHandleArpReset(pDrvData); 
            LOGD("SMBus target: ARP state reset to default (unassigned)\n");
        }
        /* Clear this specific bit */
        regBase->icClrSmbusIntr = SMBUS_ARP_RST_CMD_DET_BIT;
    }

    /* 3. Handle ARP Prepare (ARP process start) */
    if (smbusIntrStat & SMBUS_ARP_PREPARE_CMD_DET_BIT) {
        LOGI("SMBus target: ARP Prepare Command Detected\n");
        /* Can mark state machine to enter "ARP Ready" state here */
        pDrvData->arpState = SMBUS_ARP_STATE_PREPARED;
        regBase->icClrSmbusIntr = SMBUS_ARP_PREPARE_CMD_DET_BIT;
    }

    /* 4. Handle ARP Get UDID (Master reads UDID) */
    if (smbusIntrStat & SMBUS_GET_UDID_CMD_DET_BIT) {
        LOGI("SMBus target: ARP Get UDID Command Detected\n");
        /* DW IP has UDID register configured, hardware will automatically respond.*/
        /* Assume smbusTarArpAssignAddrFinishHandle has updated targetAddr */
        arpPayload.newAddr = SMBUS_TARGET_INVALID_ADDR_TAG;
        arpPayload.addrValid = 0;
        memcpy(&arpPayload.udid, &pDrvData->udid, sizeof(SmbusUdid_s));
        smbusTriggerTargetEvent(pDrvData, SMBUS_ARP_EVENT_GET_UDID, &arpPayload, sizeof(arpPayload));
        regBase->icClrSmbusIntr |= SMBUS_GET_UDID_CMD_DET_BIT;
    }

    /* 5. Handle ARP Assign Address (Address assignment completed) */
    if (smbusIntrStat & SMBUS_ASSIGN_ADDR_CMD_DET_BIT) {
        LOGI("SMBus target: ARP Assign Address Completed\n");
        
        /* Call existing handler function to update software state (such as updating pDrvData->targetAddr) */
        smbusTarArpAssignAddrFinishHandle(pDrvData);
        smbusIntrStat &= ~SMBUS_ASSIGN_ADDR_CMD_DET_BIT;
    }

    /* 6. Handle ARP/Transfer failure (PEC NACK) */
    if (smbusIntrStat & SMBUS_SLV_RX_PEC_NACK_BIT) {
        LOGE("SMBus target: RX PEC Error (NACK sent)\n");
        /* * Triggered when target receives data and PEC verification fails.
         * At this point, target has already sent NACK to Master.
         * Software needs to discard the buffer data just received, because it is corrupted.
         */
        pDrvData->pSmbusDev.targetValidRxLen = 0; 
        
        /* Trigger error event */
        smbusTriggerTargetEvent(pDrvData, SMBUS_EVENT_PEC_ERROR, NULL, 0);
    }

    /* 7. Handle Suspend */
    if (smbusIntrStat & SMBUS_SUSPEND_DET_BIT) {
        LOGI("SMBus target: Suspend Condition Detected\n");
        smbusTriggerTargetEvent(pDrvData, SMBUS_EVENT_SUSPEND, NULL, 0);
    }

    /* 8. Handle Alert (Master responded to Alert Response Address) */
    if (smbusIntrStat & SMBUS_ALERT_DET_BIT) {
        LOGD("SMBus target: Alert Response Detected\n");
        /* Means that the ALERT# we sent has been handled by Master */
    }

    /* 9. Handle Timeout */
    if (smbusIntrStat & (SMBUS_SLV_CLOCK_EXTND_TIMEOUT_BIT | SMBUS_MST_CLOCK_EXTND_TIMEOUT_BIT)) {
        LOGE("SMBus target: Clock Extend Timeout!\n");
        smbusTriggerTargetEvent(pDrvData, SMBUS_EVENT_SLV_CLK_EXT_TIMEOUT, NULL, 0);
    }

    /* 10. Clear all handled interrupts */
    /* Note: Quick Command has been cleared separately above, clear the remaining ones here */
    U32 handledMask = smbusIntrStat & ~SMBUS_QUICK_CMD_DET_BIT;
    if (handledMask) {
        regBase->icClrSmbusIntr = handledMask;
        LOGD("SMBus target: Cleared SMBus interrupts: 0x%08X\n", handledMask);
    }
}

/* ======================================================================== */
/*                    target Mode Response Functions                          */
/* ======================================================================== */
/**
 * @brief [target] Set response data
 * @details Call this function in target_READ_REQ callback to fill TX buffer.
 *          Supports dynamic response to different commands' data. Optional timeout auto-NACK.
 *
 * @param[in] devId Device ID
 * @param[in] data  Pointer to data to be sent (can be NULL)
 * @param[in] len   Data length (0-32 bytes), 0 means prepare NACK
 * @param[in] status Operation status (0=send success, <0=NACK/error)
 * @return S32  0 success, <0 failure
 *
 * @note Must be called in target_READ_REQ callback
 * @note Optional parameter status: 0=normal response, -NACK/-TIMEOUT=reject request
 */
S32 smbusTargetSetResponse(DevList_e devId, const U8 *data, U32 len, S32 status)
{
    SmbusDrvData_s *pDrvData = NULL;
    SmbusDev_s *dev = NULL;
    S32 ret = EXIT_SUCCESS;
    volatile SmbusRegMap_s *regBase = NULL;

    /* ========== Parameter validity check ========== */
    /* Device ID range check */
    if (devId < DEVICE_SMBUS0 || devId >= DEVICE_SMBUS6) {
        LOGE("%s: Invalid devId %d\n", __func__, devId);
        return -EINVAL;
    }

    /* Data length check */
    if (len > SMBUS_BLOCK_MAXLEN) {
        LOGE("%s: Data length %u exceeds maximum %u\n", __func__, len, SMBUS_BLOCK_MAXLEN);
        return -ERANGE;
    }

    /* Use funcRunBeginHelper to uniformly check driver matching, initialization status and acquire device lock */
    ret = funcRunBeginHelper(devId, DRV_ID_DW_I2C, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) return ret;

    /* Get SMBus device pointer */
    dev = &pDrvData->pSmbusDev;

    /* Get register base address */
    regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;
    SMBUS_CHECK_PARAM(regBase == NULL, -EINVAL, "Register base address is NULL");

    /* Verify if device is in target mode */
    if (pDrvData->sbrCfg.masterMode != SMBUS_MODE_TARGET) {
        LOGE("%s: Device not in target mode, current mode=%d\n", __func__, pDrvData->sbrCfg.masterMode);
        ret = -EINVAL;
        goto unlock;
    }

    /* Verify if target TX buffer exists */
    if (dev->targetTxBuf == NULL) {
        LOGE("%s: target TX buffer not allocated\n", __func__);
        ret = -ENOMEM;
        goto unlock;
    }

    LOGD("%s: Setting target response - len=%u, status=%d, txBuf=%p\n",
         __func__, len, status, dev->targetTxBuf);

    /* ========== Process response based on status parameter ========== */
    if (status < 0) {
        /* Abnormal status, send NACK response */
        LOGD("%s: Negative status detected, preparing NACK response (status=%d)\n", __func__, status);

        /* Disable TX_EMPTY interrupt to prevent further transmission */
        U32 currentMask = smbusReadReg(&regBase->icIntrMask.value);
        currentMask &= ~SMBUS_INTR_TX_EMPTY;
        smbusWriteReg(&regBase->icIntrMask.value, currentMask);
        /* Clear TX buffer state */
        dev->targetValidTxLen = 0;
        dev->msgErr = status;  /* Record error status */
        LOGD("%s: NACK prepared - TX_EMPTY disabled (mask=0x%08X)\n", __func__, currentMask);

    } else {
        /* Normal response, verify data validity */
        if ((len > 0) && (data != NULL)) {
            /* Copy data to target TX buffer */
            LOGD("%s: Copying %u bytes to target TX buffer\n", __func__, len);
            __asm__ volatile("dsb st" : : : "memory");
            /* Copy data to buffer */
            memcpy(dev->targetTxBuf, data, len);

            /* Add memory barrier again to ensure copy completion */
            __asm__ volatile("dsb st" : : : "memory");
            /* Update TX buffer length and index */
            dev->targetValidTxLen = len;
            dev->txIndex = 0;

            LOGD("%s: target TX data prepared - validTxLen=%u, txIndex=0\n",
                 __func__, dev->targetValidTxLen);

            /* Debug: display buffer content */
            if (len <= 8) {
                LOGD("%s: TX buffer content: ", __func__);
                for (U32 i = 0; i < len; i++) {
                    LOGD("0x%02X ", dev->targetTxBuf[i]);
                }
                LOGD("\n");
            }

            /* Enable TX_EMPTY interrupt to handle subsequent data transmission */
            U32 currentMask = smbusReadReg(&regBase->icIntrMask.value);
            currentMask |= SMBUS_INTR_TX_EMPTY;
            smbusWriteReg(&regBase->icIntrMask.value, currentMask);

            LOGD("%s: TX_EMPTY enabled for data transmission (mask=0x%08X)\n",
                 __func__, currentMask);

        } else {
                /* Data invalid or length is 0, prepare NACK response */
            LOGD("%s: No data or zero length - preparing NACK response (len=%u, data=%p)\n",
                __func__, len, data);

            /* Disable TX_EMPTY interrupt */
            U32 currentMask = smbusReadReg(&regBase->icIntrMask.value);
            currentMask &= ~SMBUS_INTR_TX_EMPTY;
            smbusWriteReg(&regBase->icIntrMask.value, currentMask);

            /* Clear TX buffer state */
            dev->targetValidTxLen = 0;
            dev->txIndex = 0;

            LOGD("%s: NACK prepared - TX buffer cleared (mask=0x%08X)\n", __func__, currentMask);
        }
    }
    /* ========== Release resources and return ========== */
    LOGD("%s: target response setup completed successfully\n", __func__);
exit:
unlock:
    /* Release device lock */
    funcRunEndHelper(devId);

    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Failed to set target response, ret=%d, devId=%d\n", __func__, ret, devId);
    }
    return ret;
}

/**
 * @brief SMBus control operation interface implementation
 * @details Implements the unified control interface for SMBus hardware and protocol features.
 *          This function handles locking, parameter validation, HAL layer calls, and error
 *          conversion according to the single responsibility principle. It supports SAR, ARP,
 *          Host Notify, Alert, and bus recovery functionalities by delegating to the HAL layer.
 *
 * @param[in] devId SMBus device identifier specifying the target device instance
 * @param[in] cmd Control command type from SmbusCmd_e enumeration, including:
 *              - SMBUS_CMD_HW_ENABLE/DISABLE: Hardware enable/disable operations
 *              - SMBUS_CMD_SAR_*: Slave Address Register related operations
 *              - SMBUS_CMD_ARP_*: Address Resolution Protocol operations
 *              - SMBUS_CMD_HOST_NOTIFY_*: Host Notify operations
 *              - SMBUS_CMD_ALERT_*: Alert response operations
 *              - SMBUS_CMD_BUS_RECOVERY: I2C bus recovery operations
 * @param[in,out] param Control parameter union pointer, using appropriate members based on cmd type:
 *                     - SAR commands: Use sarConfig member
 *                     - ARP commands: Use arp member
 *                     - Host Notify commands: Use hostNotify member
 *                     - Alert commands: Use alertResponse member
 *                     - Bus recovery: Use busRecovery member
 *                     - Simple enable commands: Use enable member
 *
 * @return S32 Operation result, returns 0 (EXIT_SUCCESS) on success, negative error code on failure:
 *             - (-EINVAL) Invalid parameters: Parameter is NULL or parameter value is invalid
 *             - (-ENOTSUP) Operation not supported: HAL layer control interface not implemented or command not supported
 *             - (-EBUSY) Device busy: Device is occupied by other operations
 *             - (-ENODEV) Device not initialized: Invalid device ID or device not properly initialized
 *             - (-EIO) Generic I/O error: Hardware access failure or other low-level errors
 *             - Other negative error codes returned by specific command HAL layer implementations
 *
 * @note This is a synchronous blocking interface that waits for operation completion
 * @note param must be properly set with corresponding union members based on cmd type
 * @note Some commands require prior device initialization via smbusInit
 * @note Function provides thread-safe operation with internal locking mechanism
 * @note Commands are delegated to HAL layer for hardware-specific implementation
 * @warning Caller must ensure param pointer is valid; simple commands without parameters can pass NULL
 * @warning Some commands may affect bus state, recommended to call when bus is idle
 */
S32 smbusControl(DevList_e devId, SmbusCmd_e cmd, SmbusParam_u *param)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;
    SmbusDev_s *dev = NULL;

    /* 1. Acquire lock and verify device state (Context Setup & Locking) */
    ret = funcRunBeginHelper(devId, DRV_ID_DW_I2C, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) {
        return ret;
    }

    if (pDrvData == NULL) {
        ret = -EINVAL;
        goto unlock;
    }
    dev = &pDrvData->pSmbusDev;

    /* 2. Parameter validation (Check if Param is NULL based on Cmd type) */
    /* Some commands don't need param (like HwEnable, BusRecovery, etc.) */
    bool needParam = (cmd != SMBUS_CMD_HW_ENABLE && cmd != SMBUS_CMD_HW_DISABLE &&
                      cmd != SMBUS_CMD_ARP_DISABLE && cmd != SMBUS_CMD_HOST_NOTIFY_DISABLE && 
                      cmd != SMBUS_CMD_ALERT_DISABLE && cmd != SMBUS_CMD_ARP_ENABLE && 
                      cmd != SMBUS_CMD_HOST_NOTIFY_ENABLE && cmd != SMBUS_CMD_ALERT_ENABLE);

    if (needParam && param == NULL) {
        LOGE("SMBus: Control cmd %d requires params but NULL provided\n", cmd);
        ret = -EINVAL;
        goto unlock;
    }

    /* 3. Call HAL layer (Core Logic Execution) */
    if (dev->halOps && dev->halOps->control) {
        /* Directly pass through S32 error code returned from HAL layer */
        ret = dev->halOps->control(dev, cmd, param);
        LOGE("SMBus: Control cmd %d executed via HAL, ret=%d\n", cmd, ret);
    } else {
        ret = -ENOTSUP;
    }

unlock:
    /* 4. Release lock (Cleanup) */
    funcRunEndHelper(devId);
    
    return ret;
}