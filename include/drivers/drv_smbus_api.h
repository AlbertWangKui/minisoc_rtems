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
 * 2025/11/17   wangkui         Comprehensive refactoring based on new coding standards.
 * 2025/12/01   wangkui         refactor and simplify the code as for core function
 * 2025/12/15   wangkui         replace slave with target naming
 */
#ifndef __DRV_SMBUS_API_H__
#define __DRV_SMBUS_API_H__

#include <stdbool.h> 
#include <stdint.h>  
#include "common_defines.h"
#include "bsp_device.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/*                           宏定义                                        */
/* ======================================================================== */

#define SMBUS_FLAG_10BIT_ADDR               (1U << 1)  /**< 使用 10-bit 地址 */
#define SMBUS_BLOCK_MAXLEN                  (32)       /** SMBus Block 传输最大长度 */
#define SMBUS_MAX_DATA_LENGTH               (255U)     /** SMBus 最大数据长度 */

/* ======================================================================== */
/*                        统一传输描述符标志位                              */
/* ======================================================================== */

/** 统一传输描述符标志位定义 */
#define SMBUS_FLAG_READ                (1U << 0)    /**< 读操作 */
#define SMBUS_FLAG_WRITE               (1U << 1)    /**< 写操作 */
#define SMBUS_FLAG_CMD_PRESENT         (1U << 2)    /**< 包含命令字 */
#define SMBUS_FLAG_PEC_ENABLE          (1U << 3)    /**< 使能PEC校验 */
#define SMBUS_FLAG_BLOCK_TRANSFER      (1U << 4)    /**< 块传输 */
#define SMBUS_FLAG_PROCESS_CALL        (1U << 5)    /**< 过程调用 */
#define SMBUS_FLAG_QUICK_CMD           (1U << 6)    /**< 快速命令 */
#define SMBUS_FLAG_NO_COMMAND          (1U << 7)    /**< 无命令字 */

#define SMBUS_FEATURE_NONE             (0x00)       /* 基础 I2C 模式 (无 SMBus 特性) */
/* SMBus 特性标志位 */
#define SMBUS_FEATURE_PEC            (1U << 0)  /**< 开启 PEC (Packet Error Checking) */
#define SMBUS_FEATURE_ARP            (1U << 1)  /**< 开启 ARP (Address Resolution Protocol) */
#define SMBUS_FEATURE_ALERT          (1U << 2)  /**< 开启 SMBus Alert */
#define SMBUS_FEATURE_HOST_NOTIFY    (1U << 3)  /**< 开启 Host Notify */
#define SMBUS_FEATURE_QUICK_CMD      (1U << 4)  /**< 开启 Quick Command 支持 */

/* 常用组合 */
#define SMBUS_FEATURE_DEFAULT        (SMBUS_FEATURE_PEC | SMBUS_FEATURE_QUICK_CMD)
#define SMBUS_FEATURE_ARP_FULL       (SMBUS_FEATURE_PEC | SMBUS_FEATURE_ARP | SMBUS_FEATURE_QUICK_CMD)

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
    DW_SMBUS_MODE_TARGET = 0,      /**< 从机模式 */
    DW_SMBUS_MODE_MASTER,        /**< 主机模式 */
    DW_SMBUS_MODE_RESERVED      /**< 保留字段 */
} SmbusMode_e;

typedef enum {
    SMBUS_OK = 0,
    SMBUS_ERR_TIMEOUT,
    SMBUS_ERR_ARBITRATION_LOST,
    SMBUS_ERR_NACK,
    SMBUS_ERR_PEC,
    SMBUS_ERR_RSV,
} SmbusError_e;

/**
 * @brief UDID (Unique Device Identifier) 结构
 * @note 总共 16 字节，符合 SMBus 规范
 */
typedef struct SmbusUdid {
    U8  deviceCapabilities;    /**< [0] 设备能力 */
    U8  versionRevision;       /**< [1] 版本信息 */
    U16 vendorId;              /**< [2-3] 供应商ID */
    U16 deviceId;              /**< [4-5] 设备ID */
    U16 interface;             /**< [6-7] 接口类型 */
    U16 subsystemVendorId;     /**< [8-9] 子系统供应商ID */
    U16 subsystemDeviceId;     /**< [10-11] 子系统ID */
    U32 vendorSpecificId;      /**< [12-15] 供应商特定数据 */
} __attribute__((packed)) SmbusUdid_s;

_Static_assert(sizeof(SmbusUdid_s) == 16, "SmbusUdid_s size mismatch! Must be 16 bytes.");
/**
 * @brief 通用数据传输负载
 * @note 用于 TX_DONE, RX_DONE, TARGET_WRITE_REQ 等涉及数据缓冲区的事件
 */
typedef struct {
    U8  *buffer;      /**< 数据缓冲区指针 */
    U32 len;          /**< 数据长度 */
} SmbusDataPayload_s;

/**
 * @brief 错误信息负载
 * @note 用于 SMBUS_EVT_ERROR, PEC_ERROR 等
 */
typedef struct {
    U32 errorCode;    /**< 错误码 (SMBUS_ERR_...) */
    U32 statusReg;    /**< (可选) 硬件状态寄存器值，用于调试 */
} SmbusErrorPayload_s;

/**
 * @brief ARP 分配负载
 * @note 用于 SMBUS_EVT_ARP_ASSIGNED
 */
typedef struct {
    U8  newAddr;      /**< 分配到的新地址 */
    U8  addrValid;
    U16  oldAddr;
    SmbusUdid_s udid; /**< 关联的设备 UDID */
} SmbusArpPayload_s;

/**
 * @brief Host Notify 负载
 * @note 用于 SMBUS_EVT_HOST_NOTIFY
 */
typedef struct {
    U8  addr;         /**< 发送 Notify 的设备地址 */
    U16 data;         /**< Notify 数据内容 */
} SmbusNotifyPayload_s;

typedef struct {
    U8  lastCommand;      // 最后接收/处理的命令字（0xFF=无命令）
    U8  cmdPresent;       // 标记是否有命令字（0=无，1=有）
    U8  *data;            // 数据缓冲指针
    U32 len;              // 数据长度
    U32 flags;            // 操作标记（读/写/Block等）
} SmbusTargetRequest_s;

/**
 * @brief 统一事件数据联合体
 * @details 使用 Union 节省栈空间，同时提供类型安全的访问入口
 */
typedef union { 
    SmbusDataPayload_s   transfer;    /* 1. 通用数据类 (Tx/Rx/targetReq) */
    SmbusErrorPayload_s  error;       /* 2. 错误类 */
    SmbusArpPayload_s    arp;        /* 3. ARP 类 */
    SmbusNotifyPayload_s notify;     /* 4. Notify 类 */
    SmbusTargetRequest_s  targetReq;    // ← 新增：target请求上下文
    U32                  value;      /* 5. 简单的数值 (如 Stop 检测，或者通用调用) */
} SmbusEventData_u;

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
    SMBUS_ARP_EVENT_ASSIGN,          /**< ARP 地址分配完成 */
    SMBUS_ARP_EVENT_FAIL,            /**< ARP FAIL 事件发生*/
    SMBUS_ARP_EVENT_RESERVED,        /**< 保留字段 */
} SmbusArpEvent_e;

typedef enum SmbusEvent {
    SMBUS_EVENT_TX_DONE = 9,          /**< Master TX 完成 */
    SMBUS_EVENT_RX_DONE,          /**< Master RX 完成 */
    SMBUS_EVENT_STOP_DET,         /**< STOP 条件检测到 */
    SMBUS_EVENT_ERROR,            /**< 通信错误（NACK/Timeout/Arbitration lost 等） */
    SMBUS_EVENT_PEC_ERROR,        /**< PEC 校验失败 */
    SMBUS_EVENT_TARGET_READ_REQ,  /**< 主机发起读请求（target 需应答数据）*/
    SMBUS_EVENT_TARGET_WRITE_REQ, /**< 主机发起写请求（target 接收数据）*/
    SMBUS_EVENT_TARGET_DONE,      /**< target 读写传输结束 */
    SMBUS_EVENT_ALERT,            /**< SMBus ALERT# 信号 */
    SMBUS_EVENT_HOST_NOTIFY,      /**< Host Notify 事件 */
    SMBUS_EVENT_GENERAL_CALL,     /**< General Call 命令 */
    SMBUS_EVENT_QUICK_CMD,        /**< Quick Command 命令 */
    SMBUS_EVENT_SUSPEND,          /**< SMBus Suspend detected */
    SMBUS_EVENT_TX_ABORT,         /**< TX Abort (丢失仲裁等) */
    SMBUS_EVENT_MST_CLK_EXT_TIMEOUT,    /**< Master clock extend timeout */
    SMBUS_EVENT_MST_CLK_LOW_TIMEOUT,    /**< Master clock low timeout */
    SMBUS_EVENT_SLV_CLK_EXT_TIMEOUT,    /**< Slave clock extend timeout */
    SMBUS_EVENT_SLV_CLK_LOW_TIMEOUT,    /**< Slave clock low timeout */
    SMBUS_EVENT_RESERVED,                 /**< 保留字段 */
} SmbusEvent_e;

/**
 * @brief SMBus操作命令类型
 */
typedef enum SmbusCmd {
    /* 硬件控制命令 */
    SMBUS_CMD_HW_ENABLE         = 0x0100,   /**< SMBus硬件使能 */
    SMBUS_CMD_HW_DISABLE,                   /**< SMBus硬件失能 */

    /* SAR(Slave Address Register)相关命令 */
    SMBUS_CMD_SAR_ENABLE        = 0x0200,   /**< SAR使能 */
    SMBUS_CMD_SAR_DISABLE,                  /**< SAR失能 */
    SMBUS_CMD_SAR_SET_ADDR,                 /**< 设置从地址 */
    SMBUS_CMD_SAR_GET_ADDR,                 /**< 获取从地址 */

    /* ARP(Address Resolution Protocol)相关命令 */
    SMBUS_CMD_ARP_ENABLE        = 0x0300,   /**< ARP使能 */
    SMBUS_CMD_ARP_DISABLE,                  /**< ARP失能 */
    SMBUS_CMD_ARP_IS_ADDR_USED,             /**< 检查ARP地址是否被使用 */
    SMBUS_CMD_ARP_SET_UDID,                 /**< 设置ARP UDID */
    SMBUS_CMD_ARP_GET_UDID,                 /**< 获取ARP UDID */
    SMBUS_CMD_ARP_GET_ADDR_VALID,           /**< 获取地址有效状态 */
    SMBUS_CMD_ARP_GET_ADDR_RESOLVED,        /**< 获取地址解析状态 */

    /* Host Notify相关命令 */
    SMBUS_CMD_HOST_NOTIFY       = 0x0400,   /**< Host Notify通知 */
    SMBUS_CMD_HOST_NOTIFY_ENABLE,           /**< 使能Host Notify */
    SMBUS_CMD_HOST_NOTIFY_DISABLE,          /**< 失能Host Notify */

    /* Alert Response相关命令 */
    SMBUS_CMD_ALERT_RESPOND     = 0x0500,   /**< Alert响应 */
    SMBUS_CMD_ALERT_ENABLE,                 /**< 使能Alert */
    SMBUS_CMD_ALERT_DISABLE,                /**< 失能Alert */

    /* I2C总线恢复命令 */
    SMBUS_CMD_BUS_RECOVERY      = 0x0600,   /**< I2C总线恢复 */
} SmbusCmd_e;

/**
 * @brief SAR配置结构体
 */
typedef struct SmbusSarConfig {
    U8 sarId;           /**< SAR ID (0 ~ SMBUS_MAX_SAR_NUM-1) */
    U8 slaveAddr;       /**< 从地址 (7位地址) */
    bool enable;        /**< 使能标志 */
} SmbusSarConfig_s;

/**
 * @brief ARP地址状态结构体
 */
typedef struct SmbusArpAddrStatus {
    U8 addr;            /**< SMBus地址 */
    bool isValid;       /**< 地址是否有效 */
    bool isResolved;    /**< 地址是否已解析 */
    bool isInUse;       /**< 地址是否被使用 */
} SmbusArpAddrStatus_s;

/**
 * @brief ARP配置参数聚合结构体
 * @note 用于 SmbusParam_u 中，避免匿名结构体
 */
typedef struct SmbusArpConfig {
    SmbusUdid_s udid;               /**< ARP UDID (复用已有的 SmbusUdid_s) */
    SmbusArpAddrStatus_s addrStatus;/**< 地址状态 */
    U8 checkAddr;                   /**< 用于检查的地址 */
} SmbusArpConfig_s;

/**
 * @brief Host Notify 数据结构体
 */
typedef struct SmbusHostNotifyData {
    U8 slaveAddr;       /**< 从设备地址 */
    U16 data;           /**< 通知数据 (16位) */
    U32 timestamp;      /**< 时间戳(可选) */
} SmbusHostNotifyData_s;

/**
 * @brief Alert Response 数据结构体
 */
typedef struct SmbusAlertResponseData {
    U8 respondingAddr;                  /**< 响应的设备地址 */
    U8 status;                          /**< 设备状态 */
    U8 data[SMBUS_MAX_DATA_LENGTH];     /**< 附加数据 */
    U16 dataLen;                        /**< 数据长度 */
} SmbusAlertResponseData_s;

/**
 * @brief 总线恢复配置结构体
 */
typedef struct SmbusBusRecoveryConfig {
    U32 sclRecoveryCount;   /**< SCL恢复时钟周期数 */
    U32 timeoutMs;          /**< 恢复超时时间(毫秒) */
    bool forceRecovery;     /**< 强制恢复标志 */
} SmbusBusRecoveryConfig_s;

/**
 * @brief SMBus控制参数联合体
 */
typedef union SmbusParam {
    /* SAR相关参数 */
    SmbusSarConfig_s sarConfig;
    
    /* ARP相关参数 */
    SmbusArpConfig_s arp;
    
    /* Host Notify参数 */
    SmbusHostNotifyData_s hostNotify;
    
    /* Alert Response参数 */
    SmbusAlertResponseData_s alertResponse;
    
    /* 总线恢复参数 */
    SmbusBusRecoveryConfig_s busRecovery;
    
    /* 通用使能/失能标志 (用于简单命令) */
    bool enable;
} SmbusParam_u;

/* ======================================================================== */
/*                        统一传输描述符数据结构                              */
/* ======================================================================== */

/**
 * @brief 统一传输描述符 (核心精简版)
 * @note 替换了原有的多个复杂结构体，提供统一、简洁的传输接口
 */
typedef struct SmbusXfer {
    U8         addr;          /**< 目标设备地址 (7-bit) */
    U8         command;       /**< 命令字 (若无命令设为0xFF) */
    U32        flags;         /**< SMBUS_FLAG_... 组合标志 */
    const U8  *wBuf;          /**< 写缓冲区指针 */
    U32        wLen;          /**< 写数据长度 */
    U8        *rBuf;          /**< 读缓冲区指针 */
    U32        rLen;          /**< 读数据长度 */
    U32       *actualRxLen;   /**< (输出)实际读取的字节数，仅读操作使用 */
    U32        timeout;       /**< 超时时间 (ms) */
} SmbusXfer_s;

/**
 * @brief SMBus ARP Device Node Structure
 */
typedef struct SmbusArpDeviceNode {
    struct SmbusArpDeviceNode *next;   /**< Next device in list */
    SmbusUdid_s udid;                  /**< Device UDID (16 bytes) */
    U8  currentAddress;                /**< Current assigned address */
    U8  flags;                         /**< Device flags */
    U16 reserved;                      /**< Padding/Reserved */
} SmbusArpDeviceNode_s;

/**
 * @brief SMBus ARP Master Context Structure
 */
typedef struct SmbusArpMaster {
    U8                     busId;            /**< Bus ID */
    U8                     nextAddress;      /**< Next address to assign */
    U8                     addressPoolStart; /**< Address pool start */
    U8                     addressPoolEnd;   /**< Address pool end */
    U32                    deviceCount;      /**< Number of discovered devices */
    SmbusArpDeviceNode_s  *deviceList;       /**< Linked list of devices */
} SmbusArpMaster_s;

/* ======================================================================== */
/*                         其他核心数据结构                                   */
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
            U8  targetAddr;         /**< 从机地址 */
            U8  enableArp;         /**< ARP 使能 */
        } targetConfig;
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
#ifdef TEST_SUITS_1
    void *base;                   /**< Base configuration pointer */
    U32 busSpeedHz;              /**< Bus speed in Hz (100000, 400000, 1000000) */
    U32 irqNo;                   /**< IRQ number */
    U32 irqPrio;                 /**< IRQ priority */
    S32 addrMode;                /**< Address mode (0=7-bit, 1=10-bit) */
    U8 targetAddrLow;            /**< target address (lower 7 bits for 7-bit mode) */
    S32 interruptMode;           /**< Interrupt mode (0=polling, 1=interrupt) */
    Bool isArpEnable;            /**< ARP enable flag */
    U32 featureMap;              /**< smbus 使用 SMBUS_FEATURE_* 宏组合*/
#endif
    S32 masterMode;              /**< Master mode configuration (0=disable, 1=enable) */
    U32 udidWord0;               /**< UDID word 0 */
    U32 udidWord1;               /**< UDID word 1 */
    U32 udidWord2;               /**< UDID word 2 */
    U32 udidWord3;               /**< UDID word 3 */
    SmbusUdid_s localUdid;       /**< 本机 UDID (target模式或ARP用) */
    void *userData;              /**< 用户数据，回传给 Callback */
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
 * @brief 统一事件回调函数原型
 * @param devId    设备ID
 * @param event    事件类型 (TX完成, RX数据, ARP成功, ARP失败...)
 * @param data     事件携带的数据 (联合体，根据 event 类型解析)
 * @param userData 注册时传入的用户上下文
 */
typedef void (*SmbusCallback_t)(DevList_e devId, 
                                SmbusEvent_e event, 
                                const SmbusEventData_u *data, 
                                void *userData);
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
 * @brief 发送 Reset Device 命令, NULL 表示广播·
 * @param devId SMBus设备ID
 * @param udid 目标设备的UDID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpResetDevice(DevList_e devId, const SmbusUdid_s *udid);

/**
 * @brief 发送 Get UDID 命令 (通用)
 * @param devId SMBus设备ID
 * @param udid 输出参数，存储获取到的UDID
 * @return 0 成功, 负值 失败
 * @note 此函数通常是非阻塞的，或者配合回调使用
 */
S32 smbusArpGetUdidGeneral(DevList_e devId, SmbusUdid_s *udid);

/**
 * @brief 发送 Assign Address 命令
 * @param devId SMBus设备ID
 * @param udid 目标设备的UDID
 * @return 0 成功, 负值 失败
 */
S32 smbusArpAssignAddress(DevList_e devId, const SmbusUdid_s *udid, U8 addr);

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
S32 ArpDevInstall(DevList_e devId, SmbusArpMaster_s *master, const SmbusUdid_s *udid, U8 address);

/**
 * @brief [target] 设置响应数据
 * @details 在TARGET_READ_REQ回调中调用此函数填充TX缓冲。
 *          支持动态响应不同命令的数据。可选超时自动NACK。
 *
 * @param devId 设备ID
 * @param data  指向要发送的数据指针（可为NULL）
 * @param len   数据长度(0-32字节)，0表示准备NACK
 * @param status 操作状态 (0=发送成功, <0=NACK/错误)
 * @return S32  0成功, <0失败
 *
 * @note 必须在TARGET_READ_REQ回调中调用
 * @note 可选参数status：0=正常响应, -NACK/-TIMEOUT=拒绝请求
 */
S32 smbusTargetSetResponse(DevList_e devId, const U8 *data, U32 len, S32 status);

/* ======================================================================== */
/*                   Master 模式协议操作 API                                 */
/* ======================================================================== */
/**
 * @brief SMBUS 主机统一传输操作函数
 *
 * @param devId 设备ID
 * @param xfer 统一传输描述符指针
 * @return S32 操作结果，成功返回0，失败返回错误码
 */
S32 smbusTransfer(DevList_e devId, SmbusXfer_s *xfer);

/* ======================================================================== */
/*                     用户核心函数 API 接口                                  */
/* ======================================================================== */

/**
 * @brief 设备模式切换函数
 * @param devId SMBus设备ID
 * @param param MASTER/TARGET模式切换需要参数
 * @return 0 成功, 负值错误码
 */
S32 smbusMasterTargetModeSwitch(DevList_e devId, SmbusSwitchParam_s *param);

/**
 * @brief SMBus 控制操作接口
 * @details 提供对SMBus硬件和协议功能的统一控制接口，支持SAR、ARP、Host Notify、Alert和总线恢复等功能
 *
 * @param[in] devId SMBus设备ID，用于指定操作的设备实例
 * @param[in] cmd 控制命令类型，取值为SmbusCmd_e枚举值，包括：
 *              - SMBUS_CMD_HW_ENABLE/DISABLE: 硬件使能/失能
 *              - SMBUS_CMD_SAR_*: SAR相关操作
 *              - SMBUS_CMD_ARP_*: ARP相关操作
 *              - SMBUS_CMD_HOST_NOTIFY_*: Host Notify相关操作
 *              - SMBUS_CMD_ALERT_*: Alert相关操作
 *              - SMBUS_CMD_BUS_RECOVERY: I2C总线恢复
 * @param[in,out] param 控制参数联合体指针，根据cmd类型使用对应的成员：
 *                     - SAR命令: 使用sarConfig成员
 *                     - ARP命令: 使用arp成员
 *                     - Host Notify命令: 使用hostNotify成员
 *                     - Alert命令: 使用alertResponse成员
 *                     - 总线恢复: 使用busRecovery成员
 *                     - 简单使能命令: 使用enable成员
 *
 * @return S32 操作结果，成功返回 0 (EXIT_SUCCESS)，失败返回负值错误码：
 *             - (-EINVAL) 无效参数：参数为空或参数值不符合要求
 *             - (-ENOTSUP) 操作不支持：HAL层未实现control接口或命令不支持
 *             - (-EBUSY) 设备忙：设备已被其他操作占用
 *             - (-ENODEV) 设备未初始化：设备ID无效或未正确初始化
 *             - (-EIO) 通用IO错误：硬件访问失败或其他底层错误
 *             - 其他负值错误码由具体命令的HAL层实现返回
 *
 * @note 此函数为同步阻塞接口，调用后会等待操作完成
 * @note param参数必须根据具体的cmd类型正确设置对应的联合体成员
 * @note 部分命令需要先调用smbusInit初始化设备
 * @warning 调用者必须确保param指针有效，对于不需要参数的简单命令可传入NULL
 * @warning 某些命令可能会影响总线状态，建议在总线空闲时调用
 */
S32 smbusControl(DevList_e devId, SmbusCmd_e cmd, SmbusParam_u *param);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SMBUS_API_H__ */

