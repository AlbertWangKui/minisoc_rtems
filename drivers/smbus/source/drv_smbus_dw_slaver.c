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
 * 2025/12/01   wangkui         refactor and simplify the code as for core function
 * 2025/12/11   wangkui         correct the issue in AI review
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
        status.value = regBase->icStatus.value;
        ///< We wait for the 'activity' bit to be cleared, indicating the
        if (!status.fields.activity) {
            return EXIT_SUCCESS; 
        }
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(10));
    }
    return -ETIMEDOUT;
}

/**
 * @brief [Internal] 修改本机 Slave 地址
 * @details 安全地更新硬件的从机地址寄存器 (SAR)。
 * 通常在初始化或 ARP 地址分配完成后调用。
 * @param pDrv  驱动私有数据指针
 * @param addr  新地址 (7-bit)
 * @return S32  0 成功, <0 失败
 */
static S32 smbusSetLocalAddress(SmbusDrvData_s *pDrv, U8 addr)
{
    volatile SmbusRegMap_s *regBase = pDrv->pSmbusDev.regBase;
    U32 timeout = 1000;

    ///< 1. 参数校验
    if (addr > 0x7F) {
        LOGE("SMBus: Invalid local address 0x%02X\n", addr);
        return -EINVAL;
    }

    ///< 2. 检查当前是否忙 (如果正在传输，强制修改可能导致总线错误)
    if (smbusIsBusBusy(regBase)) {
        LOGW("SMBus: Warning - Changing address while bus is busy\n");
        return -EBUSY;
        ///< 策略：可以选择返回 -EBUSY，或者强行继续（视 ARP 协议要求而定）
    }

    ///< 3. 禁用控制器 (DesignWare IP 修改 SAR 必须先 Disable)
    regBase->icEnable.fields.enable = 0;

    while ((regBase->icEnableStatus & 0x01)  && --timeout > 0) {
        ///< 等待硬件完全关闭
        udelay(10);
    }
    if (timeout == 0) {
        LOGE("SMBus: Failed to disable controller for address update\n");
        return -ETIMEDOUT;
    }

    ///< 4. 更新 SAR (Slave Address Register)
    regBase->icSar.fields.icSar = addr;

    ///< 5. 重新启用控制器
    regBase->icEnable.fields.enable = 1;

    LOGD("SMBus: Local Slave Address updated to 0x%02X\n", addr);
    return EXIT_SUCCESS;
}

/**
 * @brief 从机端ARP地址分配处理函数
 * @details 处理从机端的ARP地址分配命令。当接收到ARP Assign Address命令时调用此函数，
 *          解析命令中的UDID和新地址，验证UDID是否匹配本机，如果匹配则更新硬件地址寄存器，
 *          并通知上层应用程序地址分配完成。
 * @param[in] pDrv 指向SMBus驱动私有数据结构的指针
 * @param[in] payload 指向ARP Assign Address命令负载数据的指针，包含目标UDID和新分配的地址
 * @return void
 *
 * @note 此函数仅在从机模式下使用
 * @note 地址更新是硬件级别的，直接修改SAR寄存器
 * @note 成功分配地址后会触发用户回调函数
 * @note 如果UDID不匹配本机，函数将静默返回
 *
 * @par 处理流程：
 * 1. 解析ARP命令负载，提取目标UDID和新地址
 * 2. 比较目标UDID与本机UDID，确认是否为发给本机的命令
 * 3. 如果UDID匹配，调用smbusSetLocalAddress更新硬件地址
 * 4. 更新驱动内部状态，记录新的ARP状态和当前地址
 * 5. 通过回调函数通知应用程序地址分配完成
 *
 * @warning 此函数假设传入的payload格式正确
 * @warning 硬件地址更新操作具有原子性
 * @warning 更新失败时不会触发回调函数
 *
 * @par 依赖函数：
 * - parseAssignPacket: 解析ARP Assign Address命令包
 * - isMyUdid: 比较UDID是否匹配本机
 * - smbusSetLocalAddress: 更新硬件从机地址寄存器
 *
 * @par 线程安全性：
 * 此函数不是线程安全的，应该在SMBus中断上下文或已加锁的环境中调用
 */
S32 smbusArpSlaveHandleAssignAddr(SmbusDrvData_s *pDrv, const U8 *payload)
{
    ///< 1. 解析 Payload，提取 UDID 和 New Address
    S32 ret = EXIT_SUCCESS;
    SmbusUdid_s targetUdid;
    U8 newAddr = 0x00;
    parseAssignPacket(payload, &targetUdid, &newAddr);

    ///< 2. 检查 UDID 是否匹配本机
    if (isMyUdid(&pDrv->udid, &targetUdid)) {

        ///< 3. 核心步骤：直接调用内部函数修改硬件地址
        ret = smbusSetLocalAddress(pDrv, newAddr);

        if (ret == EXIT_SUCCESS) {
            ///< 4. 更新内部状态
            pDrv->arpState = SMBUS_ARP_EVENT_ASSIGN;
            pDrv->assignedAddr = newAddr;
            LOGI("SMBus ARP: Assigned new address 0x%02X for matching UDID\n", newAddr);        
        }
    }
    return ret;
}

/**
 * @brief 专门处理 ARP Reset 中断的逻辑
 * @note 复用 smbusSetLocalAddress，但不解析 Payload
 */
static void smbusSlaveHandleArpReset(SmbusDrvData_s *pDrv)
{
    SmbusArpPayload_s arpNotifyDev;

    LOGI("SMBus Slave: Handling ARP Reset...\n");
    {
        /* 2. 正确更新软件状态机 (与 Assign 区分开) */
        pDrv->arpState = SMBUS_ARP_STATE_RESET; // 或者 SMBUS_ARP_STATE_DEFAULT
        pDrv->assignedAddr = SMBUS_ARP_DEFAULT_ADDR;
        
        LOGI("SMBus Slave: Address reset to Default (0x%02X)\n", SMBUS_ARP_DYNAMIC_ADDR);

        /* 4. 通知上层应用 (构造对应的 Event 数据结构) */
        memset(&arpNotifyDev, 0, sizeof(arpNotifyDev));
        
        /* 填充当前设备的 UDID */
        memcpy(&arpNotifyDev.udid, &pDrv->udid, sizeof(SmbusUdid_s));
        
        /* Reset 事件中，newAddr 通常填 0x61 或 0 来表示未分配 */
        arpNotifyDev.newAddr = SMBUS_ARP_DEFAULT_ADDR; 
        arpNotifyDev.oldAddr = 0xFFFF; // 表示复位
        arpNotifyDev.addrValid = 0;    // 地址不再是"已分配"的有效状态

        /* 发送 RESET 事件 */
        smbusTriggerSlaveEvent(pDrv, SMBUS_ARP_EVENT_RESET, &arpNotifyDev, sizeof(arpNotifyDev));
    } 
    return;
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
 * @note Calls registered ARP failure handler if available
 * @note Logs assigned addresses for debugging
 * @warning This function should only be called in Slave mode
 * @warning Function modifies interrupt mask registers
 */
void smbusSlvArpAssignAddrFinishHandle(SmbusDrvData_s *pDrvData)
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
    icStatus = regBase->icStatus.value;
    
    /* * Check if the Slave Address has been Resolved.
     * Bit 17: SMBUS_SLAVE_ADDR_VALID - Hardware has a valid address
     * Bit 18: SMBUS_SLAVE_ADDR_RESOLVED - ARP process successfully resolved an address
     */
    bool addrResolved = (icStatus & (1U << 18)) != 0;
    bool addrValid = (icStatus & (1U << 17)) != 0;

    if (addrResolved && addrValid) {
        /* Success! Hardware accepted the Assign Address command */
        
        /* Read the new address from the Slave Address Register (IC_SAR) */
        assignedAddr = (U16)(regBase->icSar.value & 0x3FF); ///< Mask to 10 bits just in case

        LOGI("SMBus Slave: ARP Address Assigned! New Address: 0x%02X\n", assignedAddr);

        /* Update Driver State */
        pDrvData->arpState = SMBUS_ARP_STATE_ASSIGNED;
        pDrvData->assignedAddr = (U8)assignedAddr;

        /* Prepare Notification Payload */
        memset(&arpPayload, 0, sizeof(arpPayload));
        memcpy(&arpPayload.udid, &pDrvData->udid, sizeof(SmbusUdid_s));
        arpPayload.newAddr = (U8)assignedAddr;
        arpPayload.addrValid = 1;  
        
        /* Notify Application */
        smbusTriggerSlaveEvent(pDrvData, 
                               SMBUS_ARP_EVENT_ASSIGN, 
                               &arpPayload, sizeof(arpPayload));

    } else {
        /* * The interrupt triggered, but the status bits don't show resolution.
         * This could happen if the Assign Address command was for a DIFFERENT UDID.
         * In that case, we remain at the default address or our previous state.
         */
        LOGD("SMBus Slave: ASSIGN_ADDR interrupt received, but not for me (Status: 0x%08X)\n", icStatus);
    }

    /* * Clear the interrupt bit specifically for Assign Address.
     * Note: In the main ISR, we might clear all bits at the end, 
     * but clearing it here explicitly is safe practice for specific handlers.
     */
    regBase->icClrSmbusIntr = SMBUS_ASSIGN_ADDR_CMD_DET_BIT;
}
/**
 * @brief Trigger slave event callback
 * @details Triggers appropriate SMBus callback for slave events
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] eventType Event type to trigger
 * @param[in] data Event data pointer
 * @param[out] len Data length
 * @return void
 */
void smbusTriggerSlaveEvent(SmbusDrvData_s *pDrvData, U32 eventType, void *data, U32 len)
{
    SMBUS_CHECK_PARAM_VOID(pDrvData == NULL, "pDrvData is NULL");

    DevList_e devId = pDrvData->devId;
    SmbusEventData_u eventData;
    bool triggerCallback = true; ///< 默认触发用户回调
    memset(&eventData, 0, sizeof(SmbusEventData_u));

    switch (eventType) {
        
        /* ==========================================================
         * Case 1: ARP Assign Address (集成你的处理逻辑)
         * ========================================================== */
        case SMBUS_ARP_EVENT_ASSIGN: 
            /* 数据长度校验: UDID(16) + Addr(1) = 17 bytes */
            if (data == NULL || len < 17) {
                LOGW("SMBus ARP: Invalid payload len %d\n", len);
                return; 
            }
            S32 ret = smbusArpSlaveHandleAssignAddr(pDrvData, (const U8 *)data);
            if (ret != EXIT_SUCCESS) {
                LOGE("SMBus ARP: Failed to set hardware address 0x%02X\n", pDrvData->assignedAddr);
                return;
            }
            LOGI("SMBus ARP: Assigned new address 0x%02X\n", pDrvData->assignedAddr);
            /* 5. 准备回调数据 */
            eventData.arp.newAddr = pDrvData->assignedAddr;
            memcpy((void *)&eventData.arp.udid, (const void *)&pDrvData->udid, sizeof(SmbusUdid_s));
            break;
        case SMBUS_ARP_EVENT_RESET:
            /* 收到 Reset，硬件地址可能已被重置为 0x61 (由调用者处理或在此处理) */
            /* 这里主要负责通知上层 */
            eventData.arp.newAddr = 0x61; /* Default */
            memcpy((void *)&eventData.arp.udid, (const void *)&pDrvData->udid, sizeof(SmbusUdid_s));
            break;
        case SMBUS_ARP_EVENT_GET_UDID:
            /* 仅通知上层被读取了 UDID */
            memcpy((void *)&eventData.arp.udid, (const void *)&pDrvData->udid, sizeof(SmbusUdid_s));
            break;
        /* ==========================================================
         * Case 2: Slave Read Request
         * ========================================================== */
        case SMBUS_EVENT_SLAVE_READ_REQ:
            eventData.slaveReq.flags = SMBUS_M_RD;
            eventData.slaveReq.data  = NULL;
            eventData.slaveReq.len   = 0;
            /* 默认状态设置，等待用户在回调中调用 smbusSlaveSetResponse 修改 */
            break;

        /* ==========================================================
         * Case 3: Slave Write Request / Data Transfer
         * ========================================================== */
        case SMBUS_EVENT_SLAVE_WRITE_REQ:
            LOGD("SMBUS_EVENT_SLAVE_WRITE_REQ OCCURE:%d\r\n", eventData.slaveReq.len);
            eventData.slaveReq.flags = 0;
            eventData.slaveReq.data  = (U8 *)data;
            eventData.slaveReq.len   = len;
            break;

        case SMBUS_EVENT_TX_DONE:
        case SMBUS_EVENT_RX_DONE:
        case SMBUS_EVENT_SLAVE_DONE:
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

        /* 其他事件... */
        default:
            /* 对于不需要特殊数据打包的事件，尝试转换 data 指针 */
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
 * @brief Handle slave specific SMBus interrupts
 * @details Processes SMBus-specific interrupts in slave mode
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @param[in] smbusIntrStat SMBus interrupt status
 * @return void
 */
void smbusHandleSlaveSpecificInterrupts(SmbusDrvData_s *pDrvData,
                                        volatile SmbusRegMap_s *regBase,
                                        U32 smbusIntrStat)
{
    SmbusArpPayload_s arpPayload;
    /* 1. 处理 Quick Command */
    if (smbusIntrStat & SMBUS_QUICK_CMD_DET_BIT) {
        LOGD("SMBus Slave: Quick Command Detected\n");
        /* 立即清除中断 */
        regBase->icClrSmbusIntr = SMBUS_QUICK_CMD_DET_BIT;
        
        /* 触发事件通知上层 注意：硬件很难区分 Quick Read/Write，通常通过后续是否有数据流判断，
         *或者直接作为 Event 通知上层 "被 Ping 了" */
        smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_QUICK_CMD, NULL, 0);
    }

    /* 2. 处理 ARP Reset (Master 广播复位) - 优先级高 */
    if (smbusIntrStat & SMBUS_ARP_RST_CMD_DET_BIT) {
        LOGI("SMBus Slave: ARP Reset Command Detected!\n");
        
        /* [关键操作] 硬件通常会自动重置地址到 0x61 (Default)，
           但软件层面的状态（如 UDID 关联状态、pDrvData 中的标志位）需要重置 */
        if (pDrvData->pSmbusDev.smbFeatures.arpEnb) {
            smbusSlaveHandleArpReset(pDrvData); 
            LOGD("SMBus Slave: ARP state reset to default (unassigned)\n");
        }
        /* Clear this specific bit */
        regBase->icClrSmbusIntr = SMBUS_ARP_RST_CMD_DET_BIT;
    }

    /* 3. 处理 ARP Prepare (ARP 流程开始) */
    if (smbusIntrStat & SMBUS_ARP_PREPARE_CMD_DET_BIT) {
        LOGI("SMBus Slave: ARP Prepare Command Detected\n");
        /* 可以在这里标记状态机进入 "ARP Ready" 状态 */
        pDrvData->arpState = SMBUS_ARP_STATE_PREPARED;
        regBase->icClrSmbusIntr = SMBUS_ARP_PREPARE_CMD_DET_BIT;
    }

    /* 4. 处理 ARP Get UDID (Master 读取 UDID) */
    if (smbusIntrStat & SMBUS_GET_UDID_CMD_DET_BIT) {
        LOGI("SMBus Slave: ARP Get UDID Command Detected\n");
        /* DW IP 配置了 UDID 寄存器，硬件会自动回传。*/
        /* 假设 smbusSlvArpAssignAddrFinishHandle 已经更新了 slaveAddr */
        arpPayload.newAddr = 0xFF;
        arpPayload.addrValid = 0;
        memcpy(&arpPayload.udid, &pDrvData->udid, sizeof(SmbusUdid_s));
        smbusTriggerSlaveEvent(pDrvData, SMBUS_ARP_EVENT_GET_UDID, &arpPayload, sizeof(arpPayload));
        regBase->icClrSmbusIntr |= SMBUS_GET_UDID_CMD_DET_BIT;
    }

    /* 5. 处理 ARP Assign Address (地址分配完成) */
    if (smbusIntrStat & SMBUS_ASSIGN_ADDR_CMD_DET_BIT) {
        LOGI("SMBus Slave: ARP Assign Address Completed\n");
        
        /* 调用已有的处理函数更新软件状态（如更新 pDrvData->slaveAddr） */
        smbusSlvArpAssignAddrFinishHandle(pDrvData);
        smbusIntrStat &= ~SMBUS_ASSIGN_ADDR_CMD_DET_BIT;
    }

    /* 6. 处理 ARP/传输 失败 (PEC NACK) */
    if (smbusIntrStat & SMBUS_SLV_RX_PEC_NACK_BIT) {
        LOGE("SMBus Slave: RX PEC Error (NACK sent)\n");
        /* * 当 Slave 接收数据且 PEC 校验失败时触发。
         * 此时 Slave 已经向 Master 发送了 NACK。
         * 软件需要丢弃刚才接收到的缓冲区数据，因为它是损坏的。
         */
        pDrvData->pSmbusDev.slaveValidRxLen = 0; 
        
        /* 触发错误事件 */
        smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_PEC_ERROR, NULL, 0);
    }

    /* 7. 处理 Suspend (挂起) */
    if (smbusIntrStat & SMBUS_SUSPEND_DET_BIT) {
        LOGI("SMBus Slave: Suspend Condition Detected\n");
        smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_SUSPEND, NULL, 0);
    }

    /* 8. 处理 Alert (Master 响应了 Alert Response Address) */
    if (smbusIntrStat & SMBUS_ALERT_DET_BIT) {
        LOGD("SMBus Slave: Alert Response Detected\n");
        /* 意味着我们发出的 ALERT# 已经被 Master 处理 */
    }

    /* 9. 处理 Timeout */
    if (smbusIntrStat & (SMBUS_SLV_CLOCK_EXTND_TIMEOUT_BIT | SMBUS_MST_CLOCK_EXTND_TIMEOUT_BIT)) {
        LOGE("SMBus Slave: Clock Extend Timeout!\n");
        smbusTriggerSlaveEvent(pDrvData, SMBUS_EVENT_SLV_CLK_EXT_TIMEOUT, NULL, 0);
    }

    /* 10. 清除所有已处理的中断 */
    /* 注意：Quick Command 已经在上面单独清除了，这里清除剩余的 */
    U32 handledMask = smbusIntrStat & ~SMBUS_QUICK_CMD_DET_BIT;
    if (handledMask) {
        regBase->icClrSmbusIntr = handledMask;
        LOGD("SMBus Slave: Cleared SMBus interrupts: 0x%08X\n", handledMask);
    }
}

/* ======================================================================== */
/*                    Slave Mode Response Functions                          */
/* ======================================================================== */
/**
 * @brief [Slave] 设置响应数据
 * @details 在SLAVE_READ_REQ回调中调用此函数填充TX缓冲。
 *          支持动态响应不同命令的数据。可选超时自动NACK。
 *
 * @param[in] devId 设备ID
 * @param[in] data  指向要发送的数据指针（可为NULL）
 * @param[in] len   数据长度(0-32字节)，0表示准备NACK
 * @param[in] status 操作状态 (0=发送成功, <0=NACK/错误)
 * @return S32  0成功, <0失败
 *
 * @note 必须在SLAVE_READ_REQ回调中调用
 * @note 可选参数status：0=正常响应, -NACK/-TIMEOUT=拒绝请求
 */
S32 smbusSlaveSetResponse(DevList_e devId, const U8 *data, U32 len, S32 status)
{
    SmbusDrvData_s *pDrvData = NULL;
    SmbusDev_s *dev = NULL;
    S32 ret = EXIT_SUCCESS;
    volatile SmbusRegMap_s *regBase = NULL;

    /* ========== 参数有效性检查 ========== */
    /* 设备ID范围检查 */
    if (devId < DEVICE_SMBUS0 || devId >= DEVICE_SMBUS6) {
        LOGE("%s: Invalid devId %d\n", __func__, devId);
        return -EINVAL;
    }

    /* 数据长度检查 */
    if (len > SMBUS_BLOCK_MAXLEN) {
        LOGE("%s: Data length %u exceeds maximum %u\n", __func__, len, SMBUS_BLOCK_MAXLEN);
        return -ERANGE;
    }

    /* 使用funcRunBeginHelper统一检查驱动匹配、初始化状态和获取设备锁 */
    ret = funcRunBeginHelper(devId, DRV_ID_DW_I2C, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) return ret;

    /* 获取SMBus设备指针 */
    dev = &pDrvData->pSmbusDev;

    /* 获取寄存器基地址 */
    regBase = (volatile SmbusRegMap_s *)pDrvData->sbrCfg.regAddr;
    SMBUS_CHECK_PARAM(regBase == NULL, -EINVAL, "Register base address is NULL");

    /* 验证设备是否处于从机模式 */
    if (pDrvData->sbrCfg.masterMode != SMBUS_MODE_SLAVE) {
        LOGE("%s: Device not in slave mode, current mode=%d\n", __func__, pDrvData->sbrCfg.masterMode);
        ret = -EINVAL;
        goto unlock;
    }

    /* 验证是否有从机TX缓冲区 */
    if (dev->slaveTxBuf == NULL) {
        LOGE("%s: Slave TX buffer not allocated\n", __func__);
        ret = -ENOMEM;
        goto unlock;
    }

    LOGD("%s: Setting slave response - len=%u, status=%d, txBuf=%p\n",
         __func__, len, status, dev->slaveTxBuf);

    /* ========== 根据status参数处理响应 ========== */
    if (status < 0) {
        /* 状态异常，发送NACK响应 */
        LOGD("%s: Negative status detected, preparing NACK response (status=%d)\n", __func__, status);

        /* 禁用TX_EMPTY中断以防止继续发送 */
        U32 currentMask = regBase->icIntrMask.value;
        currentMask &= ~SMBUS_INTR_TX_EMPTY;
        regBase->icIntrMask.value = currentMask;
        /* 清除TX缓冲状态 */
        dev->slaveValidTxLen = 0;
        dev->msgErr = status;  /* 记录错误状态 */
        LOGD("%s: NACK prepared - TX_EMPTY disabled (mask=0x%08X)\n", __func__, currentMask);

    } else {
        /* 正常响应，检验数据有效性 */
        if ((len > 0) && (data != NULL)) {
            /* 将数据复制到从机TX缓冲 */
            LOGD("%s: Copying %u bytes to slave TX buffer\n", __func__, len);
            __asm__ volatile("dsb st" : : : "memory");
            /* 复制数据到缓冲区 */
            memcpy(dev->slaveTxBuf, data, len);

            /* 再次添加内存屏障确保复制完成 */
            __asm__ volatile("dsb st" : : : "memory");
            /* 更新TX缓冲长度和索引 */
            dev->slaveValidTxLen = len;
            dev->txIndex = 0;

            LOGD("%s: Slave TX data prepared - validTxLen=%u, txIndex=0\n",
                 __func__, dev->slaveValidTxLen);

            /* 调试：显示缓冲区内容 */
            if (len <= 8) {
                LOGD("%s: TX buffer content: ", __func__);
                for (U32 i = 0; i < len; i++) {
                    LOGD("0x%02X ", dev->slaveTxBuf[i]);
                }
                LOGD("\n");
            }

            /* 启用TX_EMPTY中断以处理后续数据发送 */
            U32 currentMask = regBase->icIntrMask.value;
            currentMask |= SMBUS_INTR_TX_EMPTY;
            regBase->icIntrMask.value = currentMask;

            LOGD("%s: TX_EMPTY enabled for data transmission (mask=0x%08X)\n",
                 __func__, currentMask);

        } else {
                /* 数据无效或长度为0，准备NACK响应 */
            LOGD("%s: No data or zero length - preparing NACK response (len=%u, data=%p)\n",
                __func__, len, data);

            /* 禁用TX_EMPTY中断 */
            U32 currentMask = regBase->icIntrMask.value;
            currentMask &= ~SMBUS_INTR_TX_EMPTY;
            regBase->icIntrMask.value = currentMask;

            /* 清除TX缓冲状态 */
            dev->slaveValidTxLen = 0;
            dev->txIndex = 0;

            LOGD("%s: NACK prepared - TX buffer cleared (mask=0x%08X)\n", __func__, currentMask);
        }
    }
    /* ========== 释放资源和返回 ========== */
    LOGD("%s: Slave response setup completed successfully\n", __func__);
exit:
unlock:
    /* 释放设备锁 */
    funcRunEndHelper(devId);

    if (ret != EXIT_SUCCESS) {
        LOGE("%s: Failed to set slave response, ret=%d, devId=%d\n", __func__, ret, devId);
    }
    return ret;
}
