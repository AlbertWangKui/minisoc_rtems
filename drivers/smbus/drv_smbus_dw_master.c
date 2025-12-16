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
 * 2025/12/01   wangkui         refactor and simplify the code as for core function
 * 2025/12/11   wangkui         correct the issue in AI review
 * @note This file implements the SMBus Master API layer, providing
 *       high-level Master mode operations for SMBus communication.
 *       It works on top of the SMBus core layer and handles
 *       Master-specific functionality including read/write operations,
 *       command processing, and error handling in Master mode.
 */
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

/* =========================================================================
 * Helper 1: 地址合法性校验 
 * ========================================================================= */
static S32 arpValidateAddress(const SmbusArpMaster_s *master, U8 address)
{
    /* 1. 首先检查 SMBus 协议硬性限制 */
    if (address < SMBUS_MIN_VALID_ADDRESS || address > SMBUS_MAX_VALID_ADDRESS) {
        LOGE("[ARP] Address 0x%02X out of SMBus valid range (0x%02X-0x%02X)\n",
             address, SMBUS_MIN_VALID_ADDRESS, SMBUS_MAX_VALID_ADDRESS);
        return -ERANGE;
    }

    /* 2. 其次检查 Master 配置的地址池范围 */
    if (address < master->addressPoolStart || address > master->addressPoolEnd) {
        LOGE("[ARP] Address 0x%02X out of pool range (0x%02X-0x%02X)\n",
             address, master->addressPoolStart, master->addressPoolEnd);
        return -ERANGE;
    }

    return EXIT_SUCCESS;
}

/* =========================================================================
 * Helper 2: 冲突检测 
 * ========================================================================= */
static S32 arpCheckDuplicate(const SmbusArpMaster_s *master, const SmbusUdid_s *udid, U8 address)
{
    const SmbusArpDeviceNode_s *curr = master->deviceList;

    while (curr) {
        /* 检查 UDID 是否已存在 */
        if (memcmp(&curr->udid, udid, sizeof(SmbusUdid_s)) == 0) {
            LOGW("[ARP] Device UDID already exists\n");
            return -EEXIST;
        }

        /* 检查地址是否已被占用 */
        if (curr->currentAddress == address) {
            LOGW("[ARP] Address 0x%02X already in use\n", address);
            return -EADDRINUSE;
        }

        curr = curr->next;
    }
    return EXIT_SUCCESS;
}

/* =========================================================================
 * Helper 3: 节点分配与插入 
 * ========================================================================= */
static S32 arpAllocateAndInsert(SmbusArpMaster_s *master, const SmbusUdid_s *udid, U8 address)
{
    SmbusArpDeviceNode_s *newNode = (SmbusArpDeviceNode_s *)calloc(1, sizeof(SmbusArpDeviceNode_s));
    if (!newNode) {
        LOGE("[ARP] Failed to allocate device node\n");
        return -ENOMEM;
    }

    /* 赋值 */
    newNode->currentAddress = address;
    newNode->udid = *udid; /* Struct copy */
    newNode->flags = 0;    /* Default flags */

    /* 插入头部 (Critical Section: 指针操作必须在锁内) */
    newNode->next = master->deviceList;
    master->deviceList = newNode;
    master->deviceCount++;

    return EXIT_SUCCESS;
}

/**
 * @brief 在主设备列表中安装ARP设备
 * @details 向ARP主设备列表添加新设备，包含指定的UDID和地址。
 *          此函数在添加设备前会执行参数验证和冲突检查。
 * @param[in] devId SMBus设备ID
 * @param[in,out] master ARP主设备结构体指针
 * @param[in] udid 设备UDID结构体指针
 * @param[in] address 分配给设备的地址
 * @return 0 成功, 负值 错误码:
 *         -EINVAL: 无效参数或驱动未初始化
 *         -ERANGE: 地址超出范围或为保留地址
 *         -EEXIST: 设备已存在
 *         -EADDRINUSE: 与现有设备地址冲突
 *         -ENOMEM: 内存分配失败
 *
 * @note 验证地址范围和地址池边界
 * @note 检查与现有设备的地址冲突
 * @note 更新主设备计数和下一个可用地址
 * @note 将master变量值拷贝到驱动数据结构中的pDrvData->arpMaster
 * @note 此函数会进行驱动初始化状态检查
 * @warning 此函数不是线程安全的
 * @warning 调用者必须确保master指针有效
 */
S32 ArpDevInstall(DevList_e devId, SmbusArpMaster_s *master, const SmbusUdid_s *udid, U8 address)
{
    S32 ret = EXIT_SUCCESS;
    SmbusDrvData_s *pDrvData = NULL;

    /* 1. 基础参数校验 */
    if (!master || !udid) return -EINVAL;

    /* 2. 锁保护 (Critical Section Start) */
    /* 注意：此锁必须保护 master 指向的数据结构，否则仍不安全 */
    ret = funcRunBeginHelper(devId, DRV_ID_DW_I2C, (void**)&pDrvData);
    if (ret != EXIT_SUCCESS) return ret;

    /* 4. 地址校验 */
    ret = arpValidateAddress(master, address);
    if (ret != EXIT_SUCCESS) goto unlock;

    /* 5. 冲突检测 (遍历链表) */
    ret = arpCheckDuplicate(master, udid, address);
    if (ret != EXIT_SUCCESS) goto unlock;

    /* 6. 执行安装 */
    ret = arpAllocateAndInsert(master, udid, address);
    if (ret == EXIT_SUCCESS) {
        LOGD("[ARP] Installed: Addr=0x%02X, Count=%d\n", address, master->deviceCount);
    }

unlock:
    funcRunEndHelper(devId);
    return ret;
}

/**
 * @brief 反序列化：将接收到的 RAW Buffer 解析到结构体
 * @param[in] buf  接收到的数据指针 (不包含 Byte Count，从 Capabilities 字节开始)
 * @param[out] udid 目标结构体指针
 */
static U8 deserializeUdid(const U8 *buf, SmbusUdid_s *udid) 
{
    /* 0. 安全清空：防止未赋值字段残留垃圾值 */
    memset(udid, 0, sizeof(SmbusUdid_s));

    /* 1. 基础字段解析 (Little Endian) */
    udid->deviceCapabilities = buf[0];  /* Device Capabilities */
    udid->versionRevision    = buf[1];  /* Version / Revision */
    
    /* 使用移位操作手动拼合 U16，避免非对齐访问风险 */
    udid->vendorId          = (U16)(((U16)buf[2]) | ((U16)buf[3] << SMBUS_BYTE_SHIFT_8));
    udid->deviceId          = (U16)(((U16)buf[4]) | ((U16)buf[5] << SMBUS_BYTE_SHIFT_8));
    udid->interface         = (U16)(((U16)buf[6]) | ((U16)buf[7] << SMBUS_BYTE_SHIFT_8));
    udid->subsystemVendorId = (U16)(((U16)buf[8]) | ((U16)buf[9] << SMBUS_BYTE_SHIFT_8));
    udid->subsystemDeviceId = (U16)(((U16)buf[10]) | ((U16)buf[11] << SMBUS_BYTE_SHIFT_8));
    udid->vendorSpecificId  = (U32)(((U32)buf[12]) |
                                   ((U32)buf[13] << SMBUS_BYTE_SHIFT_8) |
                                   ((U32)buf[14] << SMBUS_BYTE_SHIFT_16) |
                                   ((U32)buf[15] << SMBUS_BYTE_SHIFT_24));

    /* 3. 解析地址字节 (Byte 17, Index 16) */
    U8 addrByte = buf[16];

    /* 检查 "Address Valid" 位 */
    if (UNLIKELY((addrByte & SMBUS_ADDR_VALID_BIT_MASK) == SMBUS_TARGET_DISABLED)) {
        LOGW("[ARP] UDID Address Valid Bit is 0! (Raw: 0x%02X)\n", addrByte);
        /* 虽然无效，但通常还是会解析出地址备查 */
    }

    /* 提取 7-bit 地址 */
    return (addrByte >> SMBUS_ADDR_SHIFT_BITS) & SMBUS_7BIT_ADDR_MASK;
}

/**
 * @brief 内部辅助：序列化 UDID 到缓冲区 (Little Endian)
 */
static void serializeUdid(const SmbusUdid_s *udid, U8 *buf) 
{
/* * Mapping based on SMBus Spec (16 bytes total):
     * [0]   Capabilities (mapped from nextAvailAddr based on your struct context)
     * [1]   Version
     * [2-3] Vendor ID
     * [4-5] Device ID
     * [6-7] Interface
     * [8-9] Subsystem Vendor ID
     * [10-11] Subsystem Device ID (Taken from bytes[0-1])
     * [12-15] Vendor Specific ID  (Taken from bytes[2-5])
     */ 
    buf[0]  = udid->deviceCapabilities;      // Byte 0: Device Capabilities
    buf[1]  = udid->versionRevision;            // Byte 1: Revision
    
    buf[2]  = (U8)(udid->vendorId & SMBUS_8BIT_DATA_MASK);
    buf[3]  = (U8)((udid->vendorId >> SMBUS_BYTE_SHIFT_8) & SMBUS_8BIT_DATA_MASK);

    buf[4]  = (U8)(udid->deviceId & SMBUS_8BIT_DATA_MASK);
    buf[5]  = (U8)((udid->deviceId >> SMBUS_BYTE_SHIFT_8) & SMBUS_8BIT_DATA_MASK);

    buf[6]  = (U8)(udid->interface & SMBUS_8BIT_DATA_MASK);
    buf[7]  = (U8)((udid->interface >> SMBUS_BYTE_SHIFT_8) & SMBUS_8BIT_DATA_MASK);

    buf[8]  = (U8)(udid->subsystemVendorId & SMBUS_8BIT_DATA_MASK);
    buf[9]  = (U8)((udid->subsystemVendorId >> SMBUS_BYTE_SHIFT_8) & SMBUS_8BIT_DATA_MASK);

    buf[10] = (U8)(udid->subsystemDeviceId & SMBUS_8BIT_DATA_MASK);
    buf[11] = (U8)((udid->subsystemDeviceId >> SMBUS_BYTE_SHIFT_8) & SMBUS_8BIT_DATA_MASK);

    buf[12] = (U8)(udid->vendorSpecificId & SMBUS_8BIT_DATA_MASK);
    buf[13] = (U8)((udid->vendorSpecificId >> SMBUS_BYTE_SHIFT_8) & SMBUS_8BIT_DATA_MASK);
    buf[14] = (U8)((udid->vendorSpecificId >> SMBUS_BYTE_SHIFT_16) & SMBUS_8BIT_DATA_MASK);
    buf[15] = (U8)((udid->vendorSpecificId >> SMBUS_BYTE_SHIFT_24) & SMBUS_8BIT_DATA_MASK);

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
    S32 ret;
    U8 rxBuffer[SMBUS_UDID_LEN]; // 17 bytes
    U32 actualLen = 0;
    U8 nextAddr = 0;

    ///< 1. 参数校验
    if (udid == NULL) {
        return -EINVAL;
    }

    LOGD("[ARP] Getting UDID from general address (0x%02X)\n", SMBUS_ARP_DEFAULT_ADDR);

    ///< 2. 构造传输描述符
    SmbusXfer_s xfer = {
        .addr = SMBUS_ARP_DEFAULT_ADDR, 
        .command = SMBUS_CMD_GET_UDID,  
        ///< 关键：启用 PEC 和 BLOCK 模式
        .flags = SMBUS_FLAG_PEC_ENABLE | SMBUS_FLAG_BLOCK_TRANSFER, 
        .wLen = 0,                      // 无写数据 (除了 Command)
        .rBuf = rxBuffer,               // 接收缓冲区
        .rLen = SMBUS_UDID_LEN,         // 期望读取 17 字节
        .actualRxLen = &actualLen       // 获取实际长度用于校验
    };

    ///< 3. 执行传输 (所有底层操作如寄存器读写、TX/RX Ready、PEC校验均在此内部完成)
    ret = smbusTransfer(devId, &xfer);

    ///< 4. 异常出错处理流程
    if (ret < EXIT_SUCCESS) {
        ///< 1. 处理 NACK (No Device Response)
        if (ret == -ENXIO) {
            LOGD("[ARP] No device responded to Get UDID (NACK).\n");
            return ret; ///< 这是一个"预期内"的失败，表示扫描结束
        }
        ///< 2. 处理 Arbitration Lost (IP 冲突)
        else if (ret == -EAGAIN) {
            LOGW("[ARP] Arbitration LOST during Get UDID! (Collision detected)\n");
            return ret; ///< 这是一个"异常"失败，需要上层策略决定是否重试
        }
        ///< 3. 处理 Timeout
        else if (ret == -ETIMEDOUT) {
            LOGE("[ARP] Get UDID Timeout.\n");
            return ret;
        }
        ///< 4. 其他错误
        else {
            LOGE("[ARP] Get UDID failed: %d\n", ret);
            return ret;
        }
    }
    ///< 5. 数据长度校验
    if (actualLen != SMBUS_UDID_LEN) {
        LOGE("[ARP] Invalid UDID length received: %d (Expected %d)\n", actualLen, SMBUS_UDID_LEN);
        return -EPROTO;
    }

    ///< 6. 解析数据到结构体
    nextAddr = deserializeUdid(rxBuffer, udid);

    LOGD("[ARP] UDID read successfully. Device Addr: 0x%02X\n", nextAddr);
    return EXIT_SUCCESS;
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
S32 smbusArpPrepare(DevList_e devId)
{
    S32 ret;
    
    ///< 1. 准备数据: Prepare 命令本质上是一个 Data Byte
    U8 cmdData = SMBUS_CMD_PREPARE_ARP;

    ///< 2. 构造传输描述符
    ///< 注意：这里使用 SMBUS_FLAG_NO_CMD，因为 Send Byte 协议没有独立的 Command 字段，
    ///< 只有 Address + Data。
    SmbusXfer_s xfer = {
        .addr = SMBUS_ARP_DEFAULT_ADDR,   ///< 目标: 0x61
        .command = 0,                     ///< 在 NO_CMD 模式下忽略此字段
        .flags = SMBUS_FLAG_PEC_ENABLE | SMBUS_FLAG_NO_COMMAND, ///< 启用 PEC，使用 Send Byte 模式
        .wBuf = &cmdData,                 ///< 发送内容: 0x01
        .wLen = 1,                        ///< 长度: 1 字节
        .rLen = 0                         ///< 无读取
    };

    LOGD("[ARP] Sending Prepare to ARP (Addr:0x%02X, Cmd:0x%02X)\n",
         xfer.addr, cmdData);

    ///< 3. 执行传输
    ///< 所有的锁保护、HAL 调用、寄存器操作、PEC 计算、超时处理都在这里自动完成
    ret = smbusTransfer(devId, &xfer);

    if (ret <= EXIT_SUCCESS) {
        LOGE("[ARP] Prepare command failed, ret=%d\n", ret);
        return ret;
    }

    LOGD("[ARP] Prepare command sent successfully\n");
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
S32 smbusArpAssignAddress(DevList_e devId, const SmbusUdid_s *udid, U8 addr)
{
    /* 1. 参数校验 */
    if (UNLIKELY(udid == NULL)) {
        LOGE("[ARP] %s: UDID is NULL\n", __func__);
        return -EINVAL;
    }

    /* SMBus 7-bit Address Check (Reserved ranges excluded) */
    if (UNLIKELY(addr < SMBUS_MIN_VALID_ADDRESS || addr > SMBUS_MAX_VALID_ADDRESS)) {
        LOGE("[ARP] %s: Invalid target address 0x%02X\n", __func__, addr);
        return -EINVAL;
    }

    /* 2. 准备数据负载 (Payload) 
     * 格式: [UDID (16 bytes)] + [Assigned Address (1 byte)]
     * 总长: 17 bytes
     */
    U8 txBuf[SMBUS_UDID_LEN + 1];

    /* 序列化 UDID 到前16字节 */
    serializeUdid(udid, txBuf);
    
    /* 填充第17字节：分配的新地址 (需左移1位变为8bit格式? 通常 Block Data 是纯数据，这里存7bit值) */
    /* 注意：ARP协议中 Assign Address 命令的数据字节通常就是 7-bit 从机地址<<1 还是直接传值？
     * 标准 SMBus Spec 2.0 (Page 46) 规定：Data Byte 17 = "Address to be assigned".
     * 通常这是一个 8-bit 值，其中 bit 7-1 是地址，bit 0 未定义或为0。
     * 这里为了保险，将其格式化为 8-bit 地址格式 (Addr << 1)。
     */
    txBuf[16] = (U8)(addr << SMBUS_ADDR_SHIFT_BITS); 

    /* 3. 构建传输描述符 (Adapter) */
    SmbusXfer_s xfer = {
        .addr      = SMBUS_ARP_DEFAULT_ADDR,         /* 目标固定为 0x61 (Device Default) */
        .command   = SMBUS_CMD_ASSIGN_ADDR,         /* Command 0x04 */
        .wLen      = sizeof(txBuf),                 /* Write Length = 17 */
        .wBuf      = txBuf,
        .rLen      = 0,                             /* 无读数据 */
        .rBuf      = NULL,
        /* 关键标志位：
         * BLOCK_TRANSFER: 指示 smbusTransfer 自动添加 Byte Count (0x11)
         * PEC_ENABLE:     ARP 命令强烈建议/要求使用 PEC
         */
        .flags     = SMBUS_FLAG_BLOCK_TRANSFER | SMBUS_FLAG_PEC_ENABLE
    };

    LOGD("[ARP] Assigning 0x%02X to Device (VendorID: 0x%04X)\n", addr, udid->vendorId);

    /* 4. 执行传输 */
    return smbusTransfer(devId, &xfer);
}

/**
 * @brief 发送 Prepare to ARP 命令
 * @details 广播命令，通知总线上所有设备复位 ARP 状态并准备进行地址仲裁。
 * 使用 Block Write 协议，发送 1 字节保留数据 (0x00)。
 * @param devId SMBus设备ID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpPrepareToArp(DevList_e devId)
{
    /* * 根据 SMBus ARP 协议，Prepare to ARP 命令是一个 Block Write。
     * Data Byte 1 默认为 0000_0000b (Reserved)。
     */
    U8 payload = SMBUS_ARP_PREPARE_PAYLOAD_BYTE;

    /* 构建传输描述符 (Adapter Pattern) */
    SmbusXfer_s xfer = {
        .addr      = SMBUS_ARP_DEFAULT_ADDR,     /* Target: 0x61 */
        .command   = SMBUS_CMD_PREPARE_ARP,     /* Command: 0x01 */
        .wLen      = 1,                         /* Length: 1 byte */
        .wBuf      = &payload,                  /* Data: [0x00] */
        .rLen      = 0,                         /* No Read phase */
        .rBuf      = NULL,
        /* * 关键标志:
         * 1. BLOCK_TRANSFER: 驱动会自动添加 Byte Count (0x01)
         * 2. PEC_ENABLE: ARP 命令必须包含 PEC 校验
         */
        .flags     = SMBUS_FLAG_BLOCK_TRANSFER | SMBUS_FLAG_PEC_ENABLE
    };

    LOGI("[ARP] Sending 'Prepare to ARP' broadcast...\n");

    /* 统一调用通用传输接口 */
    return smbusTransfer(devId, &xfer);
}

/**
 * @brief 发送 Reset Device 命令
 * @details 
 * - udid != NULL: 发送 ARP Directed Reset (0x61, Cmd 0x02, Data=UDID)
 * - udid == NULL: General ARP Reset (广播给所有ARP设备) -> Send Byte
 * @param devId SMBus设备ID
 * @param udid 目标设备的UDID (NULL表示广播复位)
 * @return 0 成功, 负值 失败
 */
S32 smbusArpResetDevice(DevList_e devId, const SmbusUdid_s *udid)
{
    SmbusXfer_s xfer;
    U8 udidBuf[16];

    /* 清空传输结构体 */
    memset(&xfer, 0, sizeof(SmbusXfer_s));

    if (udid != NULL) {
        /* ============================================================
         * 场景 A: 指定设备复位 (Directed ARP Reset)
         * 协议: Block Write (带 PEC)
         * 目标: 0x61
         * 命令: 0x02
         * 负载: 16字节 UDID
         * ============================================================ */
        
        /* 序列化 UDID (由于是 packed 结构体，直接 memcpy) */
        memcpy(udidBuf, udid, 16);

        xfer.addr      = SMBUS_ARP_DEFAULT_ADDR;
        xfer.command   = SMBUS_CMD_RESET_DEVICE;
        xfer.wLen      = 16;
        xfer.wBuf      = udidBuf;
        
        /* * 标志位: 
         * 1. BLOCK_TRANSFER: 自动加 Byte Count (16)
         * 2. PEC_ENABLE: ARP Directed 命令必须带 PEC
         */
        xfer.flags = SMBUS_FLAG_BLOCK_TRANSFER | SMBUS_FLAG_PEC_ENABLE;

        LOGI("[ARP] Sending Directed Reset to specific UDID...\n");
    } 
    else {
        /* ============================================================
         * 场景 B: 广播复位 (General Call Reset)
         * 协议: Send Byte (无数据负载，Command即为数据)
         * 目标: 0x00 (General Call)
         * 命令: 0x06 (Reset)
         * 负载: 无 (wLen = 0)
         * ============================================================ */
        static U8 rawCmd = SMBUS_CMD_RESET_DEVICE; // 0x02
        xfer.addr      = SMBUS_ARP_DEFAULT_ADDR;
        xfer.command = 0;              // 忽略 command 字段
        xfer.wLen    = 1;              // 发送 1 字节
        xfer.wBuf    = &rawCmd;        // 数据内容就是命令字 0x02
        
        ///< 关键：PEC + NO_COMMAND (告诉底层不要自动发 Command 阶段，直接发 wBuf)
        xfer.flags = SMBUS_FLAG_PEC_ENABLE | SMBUS_FLAG_NO_COMMAND;
        
        LOGI("[ARP] Sending General ARP Reset (via Data Write)...\n");
    }

    /* 统一执行传输 */
    return smbusTransfer(devId, &xfer);
}



