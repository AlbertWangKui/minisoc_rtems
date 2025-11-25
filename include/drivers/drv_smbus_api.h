/**
 * @file drv_smbus_api.h
 * @author wangkui (wangkui@starsmicrosystem.com)
 * @date 2025/10/15
 * @brief SMBUS API 头文件
 *
 * @par ChangeLog
 *
 * Date         Author          Description
 * 2025/10/15   wangkui         the first version
 * 2025/10/16   wangkui		    Doxygen style review, bug fixes, and missing definitions added.
 * 2025/10/17   wangkui         Comprehensive refactoring based on new coding standards.
 *
 */

#ifndef __DRV_SMBUS_API_H__
#define __DRV_SMBUS_API_H__

#include <stdbool.h> ///<<For bool type
#include <stdint.h>  ///<<For standard integer types
#include "common_defines.h"
#include "bsp_device.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/*                           宏定义                                        */
/* ======================================================================== */

/** 仲裁优先级定义 */
#define SMBUS_ARP_PRIORITY_LOW             (0)     /**< 低优先级 */
#define SMBUS_ARP_PRIORITY_NORMAL          (1)     /**< 普通优先级 */
#define SMBUS_ARP_PRIORITY_HIGH            (2)     /**< 高优先级 */
#define SMBUS_ARP_PRIORITY_CRITICAL        (3)     /**< 关键优先级 */

/** 仲裁策略 */
#define SMBUS_ARP_STRATEGY_IMMEDIATE_RETRY (0x01)  /**< 立即重试 */
#define SMBUS_ARP_STRATEGY_BACKOFF_RETRY   (0x02)  /**< 退避重试 */
#define SMBUS_ARP_STRATEGY_SWITCH_TO_SLAVE (0x04)  /**< 切换到Slave */
#define SMBUS_ARP_STRATEGY_NOTIFY_USER     (0x08)  /**< 通知用户 */
#define SMBUS_ARP_STRATEGY_PRIORITY_BOOST  (0x10)  /**< 提升优先级 */

/** SMBus Block 传输最大长度 */
#define SMBUS_BLOCK_MAXLEN (32)

#define SMBUS_ARP_MAX_RETRIES           3      /**< 默认最大重试次数 */
#define SMBUS_ARP_BASE_BACKOFF_MS       10     /**< 基础退避时间 */
#define SMBUS_ARP_MAX_BACKOFF_MS        1000   /**< 最大退避时间 */
#define SMBUS_ARP_CONSECUTIVE_FAIL_TH   5      /**< 连续失败阈值 */
#define SMBUS_ARP_BUS_BUSY_TIMEOUT_MS   100    /**< 总线忙超时 */

/* ======================================================================== */
/*                           ARP相关数据结构                                 */
/* ======================================================================== */

/**
 * @brief SMBus 工作模式枚举
 */
typedef enum SmbusMode {
    DW_SMBUS_MODE_SLAVE = 0,      /**< 从机模式 */
    DW_SMBUS_MODE_MASTER,        /**< 主机模式 */
    DW_SMBUS_MODE_RESERVED      /**< 保留字段 */
} SmbusMode_e;

/**
 * @brief ARP失败原因枚举
 */
typedef enum SmbusArpFailReason {
    SMBUS_ARP_FAIL_REASON_UNKNOWN = 0,     /**< 未知原因 */
    SMBUS_ARP_FAIL_REASON_COLLISION,       /**< SDA冲突 */
    SMBUS_ARP_FAIL_REASON_START_LOST,      /**< START条件丢失 */
    SMBUS_ARP_FAIL_REASON_STOP_LOST,       /**< STOP条件丢失 */
    SMBUS_ARP_FAIL_REASON_DATA_LOST,       /**< 数据仲裁丢失 */
    SMBUS_ARP_FAIL_REASON_ADDR_NACK,       /**< 地址未应答 */
    SMBUS_ARP_FAIL_REASON_MULTI_MASTER,    /**< 多Master冲突 */
    SMBUS_ARP_FAIL_REASON_BUS_BUSY,        /**< 总线忙 */
    SMBUS_ARP_FAIL_REASON_TX_ERR,          /**< TX error */
    SMBUS_ARP_FAIL_REASON_RESERVED         /**< 保留字段 */
} SmbusArpFailReason_e;

/**
 * @brief ARP处理动作枚举
 */
typedef enum SmbusArpAction {
    SMBUS_ARP_ACTION_NONE = 0,             /**< 无动作 */
    SMBUS_ARP_ACTION_RETRY,                /**< 重试 */
    SMBUS_ARP_ACTION_BACKOFF_RETRY,        /**< 退避重试 */
    SMBUS_ARP_ACTION_ABORT,                /**< 放弃 */
    SMBUS_ARP_ACTION_SWITCH_SLAVE,         /**< 切换到Slave */
    SMBUS_ARP_ACTION_WAIT_NOTIFY,          /**< 等待通知 */
    SMBUS_ARP_ACTION_ESCALATE,             /**< 上报 */
    SMBUS_ARP_ACTION_RESERVED              /**< 保留字段 */
} SmbusArpAction_e;

/**
 * @brief ARP状态枚举
 */
typedef enum SmbusArpState {
    SMBUS_ARP_STATE_IDLE = 0,              /**< 空闲 */
    SMBUS_ARP_STATE_WAITING,               /**< 等待总线 */
    SMBUS_ARP_STATE_COMPETING,             /**< 竞争中 */
    SMBUS_ARP_STATE_WON,                   /**< 仲裁获胜 */
    SMBUS_ARP_STATE_LOST,                  /**< 仲裁失败 */
    SMBUS_ARP_STATE_RETRYING,              /**< 重试中 */
    SMBUS_ARP_STATE_RESERVED               /**< 保留字段 */
} SmbusArpState_e;

typedef enum {
    SMBUS_ARP_STATE_RESET = 0,
    SMBUS_ARP_STATE_WAIT_ARP,
    SMBUS_ARP_STATE_PREPARE_ARP_CMD,
    SMBUS_ARP_STATE_PROCESS_ARP,
    SMBUS_ARP_STATE_PSA_ASSIGNED,
    SMBUS_ARP_STATE_OPERATIONAL,
} SmbusArpFsm_e;

/**
 * @brief SMBus ARP失败事件结构体
 */
typedef struct SmbusArpFailEvent {
    SmbusArpFailReason_e reason;                    /**< 失败原因 */
    U8                   retryCount;                /**< 当前重试次数 */
    U32                  timestamp;                 /**< 事件时间戳 */
    U8                   slaveAddr;                 /**< 目标地址 */
    U8                   busStatus;                 /**< 总线状态 */
    U32                  txAbrtSource;              /**< TX_ABRT_SOURCE寄存器值 */
    U32                  rawIntrStat;               /**< RAW_INTR_STAT寄存器值 */
} SmbusArpFailEvent_s;

/**
 * @brief ARP失败处理上下文
 */
typedef struct SmbusArpContext {
    U8                  maxRetries;                   /**< 最大重试次数 */
    U32                 backoffTimeMs;                /**< 退避时间(ms) */
    U32                 lastFailTime;                 /**< 上次失败时间 */
    U8                  consecutiveFailures;          /**< 连续失败次数 */
    Bool                multiMasterMode;              /**< 多主模式标志 */
} SmbusArpContext_s;

/**
 * @brief UDID (Unique Device Identifier) 结构
 * @note 总共 16 字节，符合 SMBus 规范
 */
typedef struct SmbusUdid {
    U8  nextAvailAddr;         /**< [0] 设备地址分配 */
    U8  version;               /**< [1] 版本信息 */
    U16 vendorId;              /**< [2-3] 供应商ID */
    U16 deviceId;              /**< [4-5] 设备ID */
    U16 interface;             /**< [6-7] 接口类型 */
    U16 subsystemVendorId;     /**< [8-9] 子系统供应商ID */
    U8 bytes[16];              /**< [10-26] 子系统地址 */
    U8 deviceAddr;             /**< [27] 供应商特定数据 */
} __attribute__((packed)) SmbusUdid_s;

/**
 * @brief ARP配置参数
 */
typedef struct SmbusArpConfig {
    U8  maxRetries;          /**< 最大重试次数 */
    U8  retryStrategy;       /**< 重试策略 (SMBUS_ARP_STRATEGY_...) */
    U8  defaultPriority;     /**< 默认优先级 (SMBUS_ARP_PRIORITY_...) */
    U8  autoSwitchToSlave;   /**< 失败后自动切换到Slave */
    U16 retryDelayMin;       /**< 最小重试延迟 */
    U16 retryDelayMax;       /**< 最大重试延迟 */
    U16 backoffMultiplier;   /**< 退避倍数 */
    U16 failureThreshold;    /**< 失败阈值 */
    U32 busyWaitTimeout;     /**< 总线忙等待超时 */
    Bool hasPersistentAddress; /**< 是否有持久地址 */
    U32 persistentAddress;   /**< 持久地址值 */
    SmbusUdid_s udid;        /**< 设备UDID */

} SmbusArpConfig_s;

/**
 * @brief SMBus ARP 设备信息结构体
 * @note 用于描述参与 ARP 协议的设备基本信息。
 */
typedef struct SmbusArpDev {
    SmbusUdid_s udid;         /**< 设备能力描述 */
    U8          addrValid;    /**< 地址有效位 */
    U16         oldAddr;      /**< 旧的从设备地址 (若无则为 0xFFFF) */
    U16         newAddr;      /**< 新的从设备地址 */
    U8          devId[8];     /**< 设备唯一标识符 (UDID) */
} __attribute__((packed)) SmbusArpDev_s;

/**
 * @brief SMBus ARP 事件类型枚举
 */
typedef enum SmbusArpEvent {
    SMBUS_ARP_EVENT_DEVICE_ADDED,    /**< 新设备被检测到 */
    SMBUS_ARP_EVENT_DEVICE_REMOVED,  /**< 设备被移除 */
    SMBUS_ARP_EVENT_ADDRESS_CHANGED, /**< 设备地址变化 */
    SMBUS_ARP_EVENT_RESET,           /**< ARP 表或总线被重置 */
    SMBUS_ARP_EVENT_PREPARE,         /**< 准备ARP命令 */
    SMBUS_ARP_EVENT_GET_UDID,        /**< 获取UDID */
    SMBUS_ARP_EVENT_RESERVED,        /**< 保留字段 */
    SMBUS_ARP_EVENT_ASSIGN,          /**< ARP 地址分配完成 */
    SMBUS_ARP_EVENT_DISCOVER,        /**< ARP Discover 新设备（你枚举中已有）*/
} SmbusArpEvent_e;

typedef enum SmbusEvent {
    SMBUS_EVENT_TX_DONE,          /**< Master TX 完成 */
    SMBUS_EVENT_RX_DONE,          /**< Master RX 完成 */
    SMBUS_EVENT_ERROR,            /**< 通信错误（NACK/Timeout/Arbitration lost 等） */
    SMBUS_EVENT_PEC_ERROR,        /**< PEC 校验失败 */
    SMBUS_EVENT_SLAVE_READ_REQ,   /**< 主机发起读请求（Slave 需应答数据）*/
    SMBUS_EVENT_SLAVE_WRITE_REQ,  /**< 主机发起写请求（Slave 接收数据）*/
    SMBUS_EVENT_SLAVE_DONE,       /**< Slave 读写传输结束 */
    SMBUS_EVENT_ALERT,            /**< SMBus ALERT# 信号 */
    SMBUS_EVENT_HOST_NOTIFY,      /**< Host Notify 事件 */
    SMBUS_EVENT_GENERAL_CALL,     /**< General Call 命令 */
    SMBUS_EVENT_SUSPEND,          /**< SMBus Suspend detected */
    SMBUS_EVENT_TX_ABORT,         /**< TX Abort (丢失仲裁等) */
    SMBUS_EVENT_MST_CLK_EXT_TIMEOUT,    /**< Master clock extend timeout */
    SMBUS_EVENT_MST_CLK_LOW_TIMEOUT,    /**< Master clock low timeout */
    SMBUS_EVENT_SLV_CLK_EXT_TIMEOUT,    /**< Slave clock extend timeout */
    SMBUS_EVENT_SLV_CLK_LOW_TIMEOUT,    /**< Slave clock low timeout */
} SmbusEvent_e;

/* ======================================================================== */
/*                         协议处理相关数据结构                              */
/* ======================================================================== */

/**
 * @brief SMBUS 协议类型枚举
 */
typedef enum {
    SMBUS_PROTOCOL_QUICK_CMD,           /**< Quick Command */
    SMBUS_PROTOCOL_SEND_BYTE,           /**< Send Byte */
    SMBUS_PROTOCOL_RECEIVE_BYTE,        /**< Receive Byte */
    SMBUS_PROTOCOL_WRITE_BYTE,          /**< Write Byte */
    SMBUS_PROTOCOL_READ_BYTE,           /**< Read Byte */
    SMBUS_PROTOCOL_WRITE_WORD,          /**< Write Word */
    SMBUS_PROTOCOL_READ_WORD,           /**< Read Word */
    SMBUS_PROTOCOL_BLOCK_WRITE,         /**< Block Write */
    SMBUS_PROTOCOL_BLOCK_READ,          /**< Block Read */
    SMBUS_PROTOCOL_PROCESS_CALL,        /**< Process Call (Word) */
    SMBUS_PROTOCOL_BLOCK_PROCESS_CALL,  /**< Block Process Call */
} SmbusProtocol_e;

/**
 * @brief SMBUS 基础参数
 */
typedef struct SmbusBaseParam {
    U8  slaveAddr;          /**< 从设备地址 (7-bit) */
    U8  enablePec;          /**< 是否使能PEC */
    U8  reserved[2];        /**< 保留字段，用于对齐 */
    U32 timeout;            /**< 超时时间 */
} SmbusBaseParam_s;

/**
 * @brief Quick Command 特定参数
 */
typedef struct SmbusQuickParam {
    U8 rwBit;               /**< R/W位 (0=写, 1=读) */
} SmbusQuickParam_s;

/**
 * @brief Byte 操作特定参数
 */
typedef struct SmbusByteOpParam {
    U8 data;                /**< 数据字节 */
} SmbusByteOpParam_s;

/**
 * @brief Data 操作特定参数 (Byte/Word Read/Write)
 */
typedef struct SmbusDataOpParam {
    U8  cmdCode;            /**< SMBus 命令码 */
    U8  *buffer;            /**< 数据缓冲区指针(读/写) */
    U32 length;             /**< 数据长度(字节) */
    U32 *readLen;           /**< (输出)实际读取的字节数，仅读操作使用 */
} SmbusDataOpParam_s;

/**
 * @brief Block 操作特定参数
 */
typedef struct SmbusBlockOpParam {
    U8  cmdCode;            /**< SMBus 命令码 */
    U8  *buffer;            /**< 数据缓冲区指针 */
    U32 length;             /**< 数据块长度 (写操作时为输入，读操作时为缓冲区大小) */
    U32 *readLen;           /**< (输出)实际读取的字节数，仅读操作使用 */
} SmbusBlockOpParam_s;

/**
 * @brief Process Call 特定参数
 */
typedef struct SmbusProcessCallOpParam {
    U8   cmdCode;           /**< 过程调用的命令码 */
    const U8 *wbuffer;      /**< 指向包含待写入数据的缓冲区的指针 */
    U32  wlen;              /**< 从 wbuffer 写入的字节数 */
    U8   *rbuffer;          /**< 指向用于存储接收数据的缓冲区的指针 */
    U32  rlen;              /**< 接收缓冲区的最大大小 */
    U32  *readLen;          /**< (输出)实际读取到 rbuffer 中的字节数 */
} SmbusProcessCallOpParam_s;

/**
 * @brief SMBUS 统一参数结构体
 */
typedef struct SmbusParam {
    SmbusProtocol_e  protocol;    /**< 协议类型 */
    SmbusBaseParam_s base;        /**< 基础参数 */
    
    union {
        SmbusQuickParam_s           quick;        /**< Quick Command 参数 */
        SmbusByteOpParam_s          byte;         /**< Send/Receive Byte 参数 */
        SmbusDataOpParam_s          data;         /**< Byte/Word Read/Write 参数 */
        SmbusBlockOpParam_s         block;        /**< Block Read/Write 参数 */
        SmbusProcessCallOpParam_s   processCall;  /**< Process Call 参数 */
    } param;   
} SmbusParam_s;

/**
 * @brief I2C Block Write 数据结构
 */
typedef struct SmbusI2CBlockWriteData {
    U8  slaveAddr;          /**< 从机地址 */
    U8  cmdCode;            /**< 命令代码 */
    U8  length;             /**< 数据长度 */
    U8  *dataBuf;           /**< 数据缓冲区指针 */
} SmbusI2CBlockWriteData_s;

/**
 * @brief I2C Block Read 数据结构
 */
typedef struct SmbusI2CBlockReadData {
    U8  slaveAddr;          /**< 从机地址 */
    U8  cmdCode;            /**< 命令代码 */
    U8  *readLength;        /**< 读取到的长度 (输出) */
    U8  *data;              /**< 数据缓冲区指针 (输出) */
} SmbusI2CBlockReadData_s;

/**
 * @brief Host Notify 数据结构
 */
typedef struct SmbusHostNotifyData {
    U8  deviceAddr;         /**< 发送通知的设备地址 */
    U16 data;               /**< 通知数据 (低字节在前) */
} SmbusHostNotifyData_s;

/**
 * @brief Alert Response 数据结构
 */
typedef struct SmbusAlertRespData {
    U8  respondingAddr;     /**< 响应Alert的设备地址 (输出) */
} SmbusAlertRespData_s;

/* ======================================================================== */
/*                           函数指针类型定义                                */
/* ======================================================================== */

/**
 * @brief 通用回调函数类型
 * @param devId 设备ID
 * @param event 事件类型
 * @param userData 用户参数
 */
typedef void (*SmbusCallback_t)(DevList_e devId, U32 event, void *userData);

/**
 * @brief ARP仲裁失败处理回调函数类型
 * @param devId SMBus设备ID
 * @param event ARP失败事件
 * @param userData 用户参数
 * @return SmbusArpAction_e 建议采取的动作
 */
typedef SmbusArpAction_e (*SmbusArpFailHandler_t)(DevList_e devId, const SmbusArpFailEvent_s *event, void *userData);

/* ======================================================================== */
/*                         其他核心数据结构                                  */
/* ======================================================================== */

/**
 * @brief SMBus 模式切换参数
 */
typedef struct SmbusSwitchParam {
    SmbusMode_e  targetMode;      /**< 目标模式 */
    U32          flags;           /**< 切换标志 */
    U32          timeout;         /**< 超时时间 */
    union {
        struct {
            U8  slaveAddr;         /**< 从机地址 */
            U8  enableArp;         /**< ARP 使能 */
        } slaveConfig;
        struct {
            U8  addrMode;          /**< 地址模式 (7-bit/10-bit) */
            U32 speed;             /**< 速率 */
        } masterConfig;
    } config;
} SmbusSwitchParam_s;

/* ======================================================================== */
/*                           API 函数声明                                    */
/* ======================================================================== */
typedef struct SmbusUserConfigParam {
    void *base;                    /**< Base configuration pointer */
    U32 busSpeedHz;              /**< Bus speed in Hz (100000, 400000, 1000000) */
    U32 udidWord0;               /**< UDID word 0 */
    U32 irqNo;                   /**< IRQ number */
    U32 irqPrio;                 /**< IRQ priority */
    S32 masterMode;              /**< Master mode configuration (0=disable, 1=enable) */
    S32 addrMode;                /**< Address mode (0=7-bit, 1=10-bit) */
    U8 slaveAddrLow;             /**< Slave address (lower 7 bits for 7-bit mode) */
    S32 interruptMode;           /**< Interrupt mode (0=polling, 1=interrupt) */
    Bool isArpEnable;            /**< ARP enable flag */
} SmbusUserConfigParam_s;
/**
 * @brief  SMBus 总线初始化
 * @param  devId    SMBus设备ID
 * @return 0 成功, 负值 失败
 */
S32 smbusInit(DevList_e devId, SmbusUserConfigParam_s *config);

/**
 * @brief SMBus 总线去初始化
 * @param devId    SMBus设备ID
 * @return 0 成功, 负值 失败
 */
S32 smbusDeInit(DevList_e devId);

/**
 * @brief 注册通用事件回调
 * @param devId SMBus设备ID
 * @param callback 回调函数
 * @param userData 用户数据
 * @return 0 成功, 负值 失败
 */
S32 smbusRegisterCallback(DevList_e devId, SmbusCallback_t callback, void *userData);

/**
 * @brief 注销通用事件回调
 * @param devId SMBus设备ID
 * @param callback 回调函数
 * @return 0 成功, 负值 失败
 */
S32 smbusUnregisterCallback(DevList_e devId, SmbusCallback_t callback);

/**
 * @brief 注册ARP失败回调
 * @param devId    SMBus设备ID
 * @param callback 回调函数
 * @param userData 回调函数参数
 * @return 0 成功, 负值 失败
 */
S32 smbusRegArpFailCallback(DevList_e devId, SmbusArpFailHandler_t callback, void *userData);

/* ======================================================================== */
/*                     ARP 主机端 API 接口                                   */
/* ======================================================================== */

/**
 * @brief 发送 Prepare to ARP 命令
 * @param devId SMBus设备ID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpPrepareToArp(DevList_e devId);

/**
 * @brief 发送 Reset Device 命令 (通用)
 * @param devId SMBus设备ID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpResetDeviceGeneral(DevList_e devId);

/**
 * @brief 发送 Reset Device 命令 (指向特定设备)
 * @param devId SMBus设备ID
 * @param udid 目标设备的UDID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpResetDeviceDirected(DevList_e devId, const SmbusUdid_s *udid);

/**
 * @brief 发送 Get UDID 命令 (指向特定地址)
 * @param devId SMBus设备ID
 * @param address 目标设备地址
 * @param udid 输出参数，存储获取到的UDID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpGetUdidDirected(DevList_e devId, U8 address, SmbusUdid_s *udid);

/**
 * @brief 发送 Get UDID 命令 (通用)
 * @param devId SMBus设备ID
 * @param udid 输出参数，存储获取到的UDID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpGetUdidGeneral(DevList_e devId, SmbusUdid_s *udid);

/**
 * @brief 发送 Assign Address 命令
 * @param devId SMBus设备ID
 * @param udid 目标设备的UDID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpAssignAddress(DevList_e devId, const SmbusUdid_s *udid);

/**
 * @brief 执行完整的设备发现和地址分配流程
 * @param devId SMBus设备ID
 * @return 代表设备数量，负值代表失败
 */
S32 smbusArpEnumDev(DevList_e devId);


/* ======================================================================== */
/*                     ARP 从机端 API 接口                                   */
/* ======================================================================== */

/**
 * @brief 初始化 ARP 从机设备
 * @param devId SMBus设备ID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpSlaveInit(DevList_e devId);

/**
 * @brief 处理 Prepare to ARP 命令 (从机端)
 * @param devId SMBus设备ID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpSlaveHandlerPrep(DevList_e devId);

/**
 * @brief 处理 Reset Device 命令 (从机端)
 * @param devId SMBus设备ID
 * @param directed 是否是指向命令
 * @param udid 如果是指向命令，此为目标UDID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpSlaveHandlerReset(DevList_e devId, Bool directed, const SmbusUdid_s *udid);


/* ======================================================================== */
/*                   Master 模式协议操作 API                                  */
/* ======================================================================== */
/**
 * @brief SMBUS 主机统一协议操作函数
 * 
 * @param devId 设备ID
 * @param param 操作参数指针
 * @return S32 操作结果，成功返回0，失败返回错误码
 */
S32 smbusMasterOperation(DevList_e devId, SmbusParam_s *param);

/* ======================================================================== */
/*                     用户核心函数 API 接口                                        */
/* ======================================================================== */

/**
 * @brief SAR使能核心函数
 * @param devId SMBus设备ID
 * @param sarNum SAR设备编号
 * @return 0 成功, 负值错误码
 */
S32 smbusSarEnable(DevList_e devId, U32 sarNum);

/**
 * @brief SAR禁用核心函数
 * @param devId SMBus设备ID
 * @param sarNum SAR设备编号
 * @return 0 成功, 负值错误码
 */
S32 smbusSarDisable(DevList_e devId, U32 sarNum);

/**
 * @brief ARP使能核心函数
 * @param devId SMBus设备ID
 * @param sarNum SAR设备编号
 * @return 0 成功, 负值错误码
 */
S32 smbusArpEnable(DevList_e devId, U32 sarNum);

/**
 * @brief ARP禁用核心函数
 * @param devId SMBus设备ID
 * @param sarNum SAR设备编号
 * @return 0 成功, 负值错误码
 */
S32 smbusArpDisable(DevList_e devId, U32 sarNum);

/**
 * @brief UDID设置核心函数
 * @param devId SMBus设备ID
 * @param idx UDID索引
 * @param udidInfo UDID 信息
 * @return 0 成功, 负值错误码
 */
S32 smbusArpUdidSet(DevList_e devId, U32 idx, const SmbusUdid_s *udidInfo);

/**
 * @brief UDID获取核心函数
 * @param devId SMBus设备ID
 * @param idx UDID索引
 * @param udidInfo UDID 信息 (输出)
 * @return 0 成功, 负值错误码
 */
S32 smbusArpUdidGet(DevList_e devId, U32 idx, SmbusUdid_s *udidInfo);

/**
 * @brief 地址解析状态获取核心函数
 * @param devId SMBus设备ID
 * @param sarNum SAR设备编号
 * @return 0 成功, 负值错误码
 */
S32 smbusArpAddrResolvedGetStatus(DevList_e devId, U32 sarNum);

/**
 * @brief 地址有效状态获取核心函数
 * @param devId SMBus设备ID
 * @param sarNum SAR设备编号
 * @return 0 成功, 负值错误码
 */
S32 smbusArpAddrValidGetStatus(DevList_e devId, U32 sarNum);

/**
 * @brief 设备地址分配核心函数
 * @param devId SMBus设备ID
 * @param assignAddr 分配地址
 * @return 0 成功, 负值错误码
 */
S32 smbusDevAddrAssign(DevList_e devId, U8 assignAddr);

/**
 * @brief 发送 Get UDID 命令 (通用)
 * @param devId SMBus设备ID
 * @param udid 输出参数，存储获取到的UDID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpGetUdidGeneral(DevList_e devId, SmbusUdid_s *udid);

/**
 * @brief 设备模式切换函数
 * @param devId SMBus设备ID
 * @param param MASTER/SLAVER模式切换需要参数
 * @return 0 成功, 负值错误码
 */
S32 smbusMasterSlaveModeSwitch(DevList_e devId, SmbusSwitchParam_s *param);

/**
 * @brief SMBUS I2C Block Write (Master)
 * @param devId SMBus设备ID
 * @param data 命令参数
 * @return 0 成功, 负值失败
 */
S32 smbusI2CBlockWrite(DevList_e devId, SmbusI2CBlockWriteData_s *data);

/**
 * @brief SMBUS I2C Block Read (Master)
 * @param devId SMBus设备ID
 * @param data I2C block 读 data
 * @return 0 成功, 负值失败
 */
S32 smbusI2CBlockRead(DevList_e devId, SmbusI2CBlockReadData_s *data);

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
S32 smbusI2CRawRead(DevList_e devId, U8 slaveAddr, U8 *buf, U32 len);

/**
 * @brief SMBus I2C reset function for hardware recovery
 * @details This function provides API-level access to the SMBus I2C reset functionality.
 *          It performs a comprehensive reset sequence following the I2C reset pattern.
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
 */
S32 smbusReset(DevList_e devId);

/**
 * @brief Handle host notify protocol
 * @param devId SMBus adapter ID
 * @param data Host notify data
 * @note Protocol: 设备写到 Host Notify Address (0x08)上, Format: [DevAddr][DataLow][DataHigh][PEC]
 * @return 0 成功, 负值失败
 */
S32 smbusHostNotifyCmd(DevList_e devId, SmbusHostNotifyData_s *data);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SMBUS_API_H__ */

