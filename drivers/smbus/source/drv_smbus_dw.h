/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_smbus_dw.h
 * @author wangkui (wangkui@starsmicrosystem.com)
 * @date 2025/10/20
 * @brief SMBus protocol core LOGEc header file
 *
 * @par ChangeLog:
 * Date         Author          Description
 * 2025/10/20   wangkui         Initial version
 * 2025/12/01   wangkui         refactor and simplify the code as for core function
 * 2025/12/11   wangkui         correct the issue in AI review
 */

#ifndef __DRV_SMBUS_DW_H__
#define __DRV_SMBUS_DW_H__

#include "osp_types.h"
#include "bsp_sbr.h"
#include <stdbool.h>
#include "../../include/drivers/drv_smbus_api.h"
#include "drv_smbus_dw_i2c.h"

/* RTEMS includes for static inline functions */
#include <rtems.h>

/* System includes for static inline functions */
#include <errno.h>

/* Logging includes for static inline functions */
#include "log_msg.h"

#define TEST_SUITS_1
#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration for SmbusRegMap structure */
typedef struct SmbusRegMap SmbusRegMap_s;

/* Forward declaration for SmbusHalOps structure */
typedef struct SmbusHalOps SmbusHalOps_s;
/* ======================================================================== */
/*                              Constants                                   */
/* ======================================================================== */
#define SMBUS_MAX_DEVICE_NUM          (6)       /**< 最大支持的 SMBus 设备数量 */
#define SMBUS_LOCK_TIMEOUT_MS         (1000)    /**< Lock timeout in milliseconds */
#define SMBUS_TRANSFER_TIMEOUT_MS     (5000)    /**< Transfer timeout in milliseconds */
#define SMBUS_MAX_BLOCK_LEN           (32)      /**< Maximum block transfer length */
#define SMBUS_ALERT_RESPONSE_ADDR     (0x0C)    /**< SMBus alert response address */
#define ARP_ASSIGN_ADDR_CMD_DET_BIT   (7)       /**< Bit position for the ARP Assign Address Command detection */
#define SMBUS_TX_READY_TIMEOUT_US     (10000U)  /**< Transmit ready timeout in microseconds */
#define SMBUS_RX_READY_TIMEOUT_US     (10000U)  /**< Receive ready timeout in microseconds */
#define SMBUS_TRANSACTION_TIMEOUT_US  (100000U) /**< Overall transaction timeout in microseconds */
#define SMBUS_UDID_SIZE               (16U)     /**< Size of the SMBus Unique Device Identifier (UDID) in bytes */
#define SMBUS_MAX_BUFFER_SIZE         (256U)    /**< Maximum buffer size for SMBus data transfer */
#define SMBUS_ARP_TIMEOUT_MS          (5000U)   /**< Timeout for the SMBus Address Resolution Protocol (ARP) in milliseconds */

/* ======================================================================== */
/*                    Unified I2C Transfer Constants                          */
/* ======================================================================== */

/* SMBus Block Transfer Constants */
#define SMBUS_BLOCK_MAX                (32)                      /**< Maximum SMBus block size */
#define SMBUS_STACK_BUF_SIZE           (SMBUS_BLOCK_MAXLEN + 8)  /* 32 + 8 = 40 bytes, safe for headers */
#define SMBUS_PEC_BUF_SIZE             (80)                      /* Safe size for Write history + Read data */
#define SMBUS_PEC_LEN                  (1)                       /**< PEC (Packet Error Code) length */
#define SMBUS_CMD_LEN                  (1)                       /**< Command length */
#define SMBUS_LEN_BYTE                 (1)                       /**< Length byte size */
#define SMBUS_ADDR_LEN                 (1)                       /**< Address byte size */

/* I2C Message Flags */
#define I2C_M_RD                       (0x0001)    /**< I2C read direction flag */
#define I2C_M_TEN                      (0x0010)    /**< Ten-bit chip address */
#define I2C_M_DMA_SAFE                 (0x0200)    /**< DMA safe flag */

/* Maximum Transfer Sizes */
#define SMBUS_MAX_WRITE_BLOCK_SIZE     (SMBUS_CMD_LEN + SMBUS_LEN_BYTE + SMBUS_BLOCK_MAX + SMBUS_PEC_LEN)  /**< Max write block: Cmd + Len + Data + PEC */
#define SMBUS_MAX_READ_BLOCK_SIZE      (SMBUS_LEN_BYTE + SMBUS_BLOCK_MAX + SMBUS_PEC_LEN)                /**< Max read block: Len + Data + PEC */
#define SMBUS_MAX_WORD_XFER_SIZE       (SMBUS_CMD_LEN + 2 + SMBUS_PEC_LEN)                             /**< Max word transfer: Cmd + 2 bytes + PEC */
#define SMBUS_MAX_BYTE_XFER_SIZE       (SMBUS_CMD_LEN + 1 + SMBUS_PEC_LEN)                             /**< Max byte transfer: Cmd + 1 byte + PEC */

/* ======================================================================== */
/*                       SMBus Hardware Timing Constants                      */
/* ======================================================================== */
/* SCL count values for different speed modes (ported from I2C) */
#define SMBUS_HW_CLK_H100             (65)     /**< Standard mode (100kHz) SCL high count */
#define SMBUS_HW_CLK_L100             (65)     /**< Standard mode (100kHz) SCL low count */
#define SMBUS_HW_CLK_H400             (15)      /**< Fast mode (400kHz) SCL high count */
#define SMBUS_HW_CLK_L400             (15)      /**< Fast mode (400kHz) SCL low count */
#define SMBUS_HW_CLK_H1000            (6)      /**< Fast mode plus (1MHz) SCL high count */
#define SMBUS_HW_CLK_L1000            (6)      /**< Fast mode plus (1MHz) SCL low count */
#define SMBUS_HW_CLK_H3400            (30)       /**< High speed (3.4MHz) SCL high count */
#define SMBUS_HW_CLK_L3400            (30)       /**< High speed (3.4MHz) SCL low count */

/* SMBus speed mode frequencies */
#define SMBUS_MAX_STANDARD_MODE_FREQ  (100000U)  /**< Standard mode: 100 kHz */
#define SMBUS_MAX_FAST_MODE_FREQ      (400000U)  /**< Fast mode: 400 kHz */
#define SMBUS_MAX_FAST_MODE_PLUS_FREQ (1000000U) /**< Fast mode plus: 1 MHz */
#define SMBUS_MAX_HIGH_SPEED_MODE_FREQ (3400000U) /**< High speed: 3.4 MHz */

/* ======================================================================== */
/*                           Enumeration Types                              */
/* ======================================================================== */

/**
 * @brief SMBus Command Type Enumeration
 */
typedef enum SmbusCommandType {
    SMBUS_CMD_TYPE_QUICK = 0,              /**< Quick command */
    SMBUS_CMD_TYPE_SEND_BYTE,              /**< Send byte */
    SMBUS_CMD_TYPE_RECEIVE_BYTE,           /**< Receive byte */
    SMBUS_CMD_TYPE_WRITE_BYTE,             /**< Write byte */
    SMBUS_CMD_TYPE_READ_BYTE,              /**< Read byte */
    SMBUS_CMD_TYPE_WRITE_WORD,             /**< Write word */
    SMBUS_CMD_TYPE_READ_WORD,              /**< Read word */
    SMBUS_CMD_TYPE_PROCESS_CALL,           /**< Process call */
    SMBUS_CMD_TYPE_BLOCK_WRITE,            /**< Block write */
    SMBUS_CMD_TYPE_BLOCK_READ,             /**< Block read */
    SMBUS_CMD_TYPE_BLOCK_PROCESS_CALL,     /**< Block process call */
    SMBUS_CMD_TYPE_RESERVED                /**< Reserved */
} SmbusCommandType_e;

/**
 * @brief SMBus Feature Configuration Structure
 * @details This structure replaces the simple isSmbus U8 flag to provide
 *          detailed control over SMBus protocol features including
 *          PEC (Packet Error Code), ARP (Address Resolution Protocol),
 *          and Host Notify support.
 */
typedef struct SmbusFeatureConfig {
    U8 pecEnb           : 1;            /**< PEC (Packet Error Code) enable bit */
    U8 hostNotifyEnb    : 1;            /**< Host Notify support enable bit */
    U8 arpEnb           : 1;            /**< arp enable support bit */
    U8 smbAlertEnb      : 1;            /**< SMBus Alert enable bit */
    U8 quickCmdEnb      : 1;            /**< Quick Command support enable bit */
    U8 reserved         : 4;           /**< Reserved for future expansion */
} SmbusFeatureConfig_s;

enum {
    SMBUS_CMD_PREPARE_FOR_ARP = 0x01,
    SMBUS_CMD_GENERAL_RESET_DEV = 0x02,
    SMBUS_CMD_GENERAL_GET_UDID = 0x03,
    SMBUS_CMD_GENERAL_ASSIGN_ADDR = 0x04,
};
/**
 * @brief SMBus Transfer State Enumeration
 */
typedef enum SmbusTransferState {
    SMBUS_TRANSFER_STATE_IDLE = 0,         /**< Idle state */
    SMBUS_TRANSFER_STATE_ACTIVE,           /**< Transfer active */
    SMBUS_TRANSFER_STATE_COMPLETE,         /**< Transfer complete */
    SMBUS_TRANSFER_STATE_ERROR,            /**< Transfer error */
    SMBUS_TRANSFER_STATE_TIMEOUT,          /**< Transfer timeout */
    SMBUS_TRANSFER_STATE_RESERVED          /**< Reserved */
} SmbusTransferState_e;


#define SMBUS_ARP_DEFAULT_ADDR (0x61)
#define SMBUS_ARP_DYNAMIC_ADDR (0x21)
#define SMBUS_ADDR_GC          (0x00) /* General Call Address */
#define SMBUS_CMD_PREPARE_ARP  (0x01)
#define SMBUS_CMD_RESET_DEVICE (0x02)
#define SMBUS_CMD_ASSIGN_ADDR  (0x04)
#define SMBUS_GC_CMD_RESET     (0x06)
#define SMBUS_CMD_GET_UDID     (0x03)
#define SMBUS_ADDR_HOST_NOTIFY (0x08)
#define SMBUS_UDID_LEN         (17)   // 16 bytes UDID + 1 byte Addr
#define SMBUS_DEFAULT_GC_ACK   (0x01)

#define CRC8_POLY              (0x1070U << 3)
#define	BIT(nr)			       (1UL << (nr))
#define EXTRACT_BYTE(word, byteNum)  ((U8)((word >> (byteNum * 8)) & 0xFF))

/* ======================================================================== */
/*                           Named Constants                                   */
/* ======================================================================== */

/* SMBus Address Constants */
#define SMBUS_MIN_DYNAMIC_ADDRESS      (0x03)        /**< Minimum dynamic address */
#define SMBUS_MIN_VALID_ADDRESS        (0x08)        /**< Minimum valid address (reserved 0x00-0x07)*/
#define SMBUS_MAX_VALID_ADDRESS        (0x77)        /**< Maximum valid address */
#define SMBUS_ARP_CONTROLLER_ADDR      (0x61)        /**< ARP controller address (7-bit) */

/* SMBus Command Codes */
#define SMBUS_CMD_ASSIGN_ADDRESS       (0x04)        /**< Assign Address command code */
#define SMBUS_CMD_PREPARE_TO_ARP       (0x06)        /**< Prepare to ARP command code */
#define SMBUS_CMD_GENERAL_GET_UDID     (0x03)        /**< Get UDID command code */
#define SMBUS_CMD_DIRECTED_RESET       (0x1E)        /**< Directed Reset Device command code */


/* SMBus Protocol Constants */
#define SMBUS_ARP_DEFAULT_SLAVE_ADDR   (0x21)        /**< Default slave address for ARP */
#define SMBUS_ARP_ADDRESS_POOL_START   (0x10)        /**< ARP address pool start */
#define SMBUS_ADDRESS_POOL_START       (0x03)        /**< Address pool start */
#define SMBUS_ADDRESS_POOL_END         (0x77)        /**< Address pool end */
#define SMBUS_MAX_SLAVE_ADDRESS        (0x7F)        /**< Maximum slave address (10-bit) */

/* CRC and Bitmask Constants */
#define SMBUS_CRC8_POLY_HIGHBIT         (0x80)        /**< CRC-8 high bit for detection */
#define SMBUS_ARP_ABORT_MASK            (0x09)        /**< ARP abort source mask */
#define SMBUS_ADDRESS_MASK              (0x7F)        /**< Address mask (7-bit) */
#define SMBUS_TENBIT_ADDRESS_MASK       (0x3FF)       /**< 10-bit address mask */

/* ========================================================================== */
/* DesignWare IC_SMBUS_INTR_MASK Register Bit Definitions           */
/* ========================================================================== */

#define SMBUS_SLV_CLOCK_EXTND_TIMEOUT_BIT (1U << 0)   ///< [0] Slave Clock Extend Timeout 
#define SMBUS_MST_CLOCK_EXTND_TIMEOUT_BIT (1U << 1)   /* [1] Master Clock Extend Timeout */
#define SMBUS_QUICK_CMD_DET_BIT           (1U << 2)   /* [2] Quick Command Detected */
#define SMBUS_HOST_NOTIFY_MST_DET_BIT     (1U << 3)   /* [3] Host Notify Master Detected */
#define SMBUS_ARP_PREPARE_CMD_DET_BIT     (1U << 4)   /* [4] ARP Prepare Command Detected */
#define SMBUS_ARP_RST_CMD_DET_BIT         (1U << 5)   /* [5] ARP Reset Command Detected */
#define SMBUS_GET_UDID_CMD_DET_BIT        (1U << 6)   /* [6] Get UDID Command Detected */
#define SMBUS_ASSIGN_ADDR_CMD_DET_BIT     (1U << 7)   /* [7] Assign Address Command Detected */
#define SMBUS_SLV_RX_PEC_NACK_BIT         (1U << 8)   /* [8] Slave RX PEC NACK */
#define SMBUS_SUSPEND_DET_BIT             (1U << 9)   /* [9] SMBus Suspend Detected */
#define SMBUS_ALERT_DET_BIT               (1U << 10)  /* [10] SMBus Alert Detected */

/* Device Flag Constants */
#define SMBUS_DEVICE_FLAG_ACTIVE       (0x01)        /**< Device active flag */
#define SMBUS_DEVICE_FLAG_NONE         (0x00)        /**< No flags set */

/* Data Transfer Constants */
#define SMBUS_I2C_WRITE_MODE           (0x00)        /**< I2C write mode bit */
#define SMBUS_I2C_READ_MODE            (0x01)        /**< I2C read mode bit */
#define SMBUS_PEC_COMMAND_BIT          (1U << 8)     /**< PEC command enable bit */
#define SMBUS_PEC_ENABLE_BIT           (1U << 9)     /**< PEC enable bit */

/* ARP State Constants */
#define SMBUS_ARP_STATE_INIT           (0)           /**< ARP initialization state */
#define SMBUS_ARP_STATE_READY          (1)           /**< ARP ready state */

/* Device Status Mask Constants */
#define SMBUS_STATUS_ACTIVE_MASK           (0x01)        /**< Device active status mask */
#define SMBUS_STATUS_SLAVE_ENABLED_MASK    (0x02)        /**< Slave enabled status mask */
#define SMBUS_STATUS_WRITE_IN_PROGRESS_MASK (0x04)        /**< Write in progress status mask */
#define SMBUS_STATUS_READ_IN_PROGRESS_MASK  (0x08)        /**< Read in progress status mask */

/* I2C Interrupt Mask Constants - Using BIT() macro */
#define SMBUS_INTR_RX_UNDER        BIT(0)   /**< RX under interrupt */
#define SMBUS_INTR_RX_OVER         BIT(1)   /**< RX over interrupt */
#define SMBUS_INTR_RX_FULL         BIT(2)   /**< RX full interrupt */
#define SMBUS_INTR_TX_OVER         BIT(3)   /**< TX over interrupt */
#define SMBUS_INTR_TX_EMPTY        BIT(4)   /**< TX empty interrupt */
#define SMBUS_INTR_RD_REQ          BIT(5)   /**< Read request interrupt */
#define SMBUS_INTR_TX_ABRT         BIT(6)   /**< TX abort interrupt */
#define SMBUS_INTR_RX_DONE         BIT(7)   /**< RX done interrupt */
#define SMBUS_INTR_ACTIVITY        BIT(8)   /**< Activity interrupt */
#define SMBUS_INTR_STOP_DET        BIT(9)   /**< Stop detection interrupt */
#define SMBUS_INTR_START_DET       BIT(10)  /**< Start detection interrupt */
#define SMBUS_INTR_GEN_CALL        BIT(11)  /**< General call interrupt */
#define SMBUS_INTR_RESTART_DET     BIT(12)  /**< Restart detection interrupt */
#define SMBUS_INTR_WR_REQ          BIT(15)  /**< Write request interrupt (bit 15) */
#define SMBUS_INTR_SLV_ADDR1_TAG   BIT(16)  /**< Slave Address 1 Tag interrupt */
#define SMBUS_INTR_SLV_ADDR2_TAG   BIT(17)  /**< Slave Address 2 Tag interrupt */
#define SMBUS_INTR_SLV_ADDR3_TAG   BIT(18)  /**< Slave Address 3 Tag interrupt */
#define SMBUS_INTR_SLV_ADDR4_TAG   BIT(19)  /**< Slave Address 4 Tag interrupt */

/* All interrupts mask for clearing all interrupts */
#define SMBUS_IC_INTR_ALL                 (0xFFFFFFFF)   /**< All interrupts mask */

#define SMBUS_ARP_INTR_MASK ( \
        SMBUS_ARP_PREPARE_CMD_DET_BIT | \
        SMBUS_ARP_RST_CMD_DET_BIT | \
        SMBUS_ASSIGN_ADDR_CMD_DET_BIT )
/* ===== Grouped Interrupt Masks ===== */
/* CRITICAL: Do NOT enable TX_EMPTY (Bit 4) for Slave initially.
* Only enable RX_FULL, RD_REQ, STOP_DET, TX_ABRT, etc.
* TX_EMPTY should only be enabled inside ISR when RD_REQ is received.
*/
#define SMBUS_SLAVE_INTR_MASK ( \
        SMBUS_INTR_WR_REQ | \
        SMBUS_INTR_RX_FULL | \
        SMBUS_INTR_RD_REQ | \
        SMBUS_INTR_TX_ABRT | \
        SMBUS_INTR_RX_DONE | \
        SMBUS_INTR_STOP_DET )                       /**< Slave mode interrupt mask (with WR_REQ) */


#define SMBUS_SLAVE_INIT_INTR_MASK ( \
        SMBUS_INTR_WR_REQ | \
        SMBUS_INTR_RX_FULL | \
        SMBUS_INTR_RD_REQ | \
        SMBUS_INTR_TX_ABRT | \
        SMBUS_INTR_RX_DONE | \
        SMBUS_INTR_STOP_DET )                       /**< Slave mode initial interrupt mask (TX_EMPTY disabled) */

#define SMBUS_SLAVE_INTERRUPT_CONFIG ( \
        SMBUS_INTR_RX_FULL | \
        SMBUS_INTR_RD_REQ | \
        SMBUS_INTR_TX_ABRT | \
        SMBUS_INTR_RX_DONE | \
        SMBUS_INTR_STOP_DET )                       /**< Slave interrupt configuration mask */

#define SMBUS_SLAVE_INTERRUPT_EXTENDED_CONFIG ( \
        SMBUS_INTR_RX_UNDER | \
        SMBUS_INTR_RX_OVER | \
        SMBUS_INTR_RX_FULL | \
        SMBUS_INTR_TX_OVER | \
        SMBUS_INTR_RD_REQ | \
        SMBUS_INTR_TX_ABRT | \
        SMBUS_INTR_RX_DONE | \
        SMBUS_INTR_ACTIVITY | \
        SMBUS_INTR_STOP_DET | \
        SMBUS_INTR_START_DET | \
        SMBUS_INTR_GEN_CALL | \
        SMBUS_INTR_RESTART_DET )                     /**< Slave extended interrupt configuration mask */

#define SMBUS_MASTER_INTERRUPT_CONFIG ( \
        SMBUS_INTR_RX_FULL | \
        SMBUS_INTR_TX_EMPTY | \
        SMBUS_INTR_TX_ABRT | \
        SMBUS_INTR_STOP_DET )                          /**< Master interrupt configuration mask */

#define SMBUS_ERROR_INTERRUPT_CONFIG ( \
        SMBUS_INTR_RX_UNDER | \
        SMBUS_INTR_RX_OVER | \
        SMBUS_INTR_TX_OVER | \
        SMBUS_INTR_TX_ABRT )                        /**< Error-related interrupt configuration mask */

/* ===== SMBUS Protocol and Clock Extension Interrupt Configurations ===== */
#define SMBUS_BUS_PROTOCOL_INT_MASK ( \
        SMBUS_QUICK_CMD_DET_BIT | \
        SMBUS_HOST_NOTIFY_MST_DET_BIT | \
        SMBUS_SLV_RX_PEC_NACK_BIT )                 /**< SMBus protocol-related interrupt mask */

#define SMBUS_CLOCK_EXT_INT_MASK ( \
        SMBUS_SLV_CLOCK_EXTND_TIMEOUT_BIT | \
        SMBUS_MST_CLOCK_EXTND_TIMEOUT_BIT )                 /**< Clock extension-related interrupt mask */

/* ===== SMBUS Protocol Transfer Flow Based Interrupt Configurations ===== */
#define SMBUS_SLAVE_WRITE_XFER_CONFIG ( \
        SMBUS_INTR_WR_REQ | \
        SMBUS_INTR_RX_FULL | \
        SMBUS_INTR_TX_ABRT | \
        SMBUS_INTR_STOP_DET | \
        SMBUS_INTR_RESTART_DET | \
        SMBUS_INTR_SLV_ADDR1_TAG )   /**< Slave write transfer config (修复添加Slave地址标签) */

#define SMBUS_SLAVE_READ_XFER_CONFIG ( \
        SMBUS_INTR_RD_REQ | \
        SMBUS_INTR_TX_EMPTY | \
        SMBUS_INTR_TX_ABRT | \
        SMBUS_INTR_RX_DONE | \
        SMBUS_INTR_STOP_DET )       /**< Slave read transfer config */

#define SMBUS_SLAVE_OPTIMAL_CONFIG ( \
        SMBUS_SLAVE_WRITE_XFER_CONFIG | \
        SMBUS_SLAVE_READ_XFER_CONFIG )   /**< Combined slave optimal config (与probe模式一致) */

#define SMBUS_MASTER_INTR_MASK ( \
        SMBUS_INTR_TX_EMPTY | \
        SMBUS_INTR_TX_ABRT  | \
        SMBUS_INTR_STOP_DET )                       /**< Master mode interrupt mask (basic) */

/* I2C TX Abort Source Mask Constants */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_MASK (1U << 0)   /**< 7-bit address NACK abort source mask */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_MASK (1U << 1)      /**< Master disable abort source mask */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_10B_ADDR_NOACK_MASK (1U << 2)  /**< 10-bit address NACK abort source mask */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_MASK (1U << 3)     /**< TX data NACK abort source mask */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK_MASK (1U << 4)      /**< General Call NACK abort source mask */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_GCALL_READ_MASK (1U << 5)       /**< General Call read abort source mask */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_HIGH_SPEED_NACK_MASK (1U << 6)  /**< High speed NACK abort source mask */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_BYTE_NACK_MASK (1U << 7)        /**< Byte NACK abort source mask */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_SDA_STUCK_LOW_MASK (1U << 8)    /**< SDA stuck low abort source mask */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_ARB_LOST_MASK (1U << 9)         /**< Arbitration lost abort source mask */

/* SMBus Error Types */
#define SMBUS_ERR_TYPE_NACK_7BIT          (1)  /**< 7-bit address NACK */
#define SMBUS_ERR_TYPE_NACK_10BIT         (2)  /**< 10-bit address NACK */
#define SMBUS_ERR_TYPE_NACK_DATA          (3)  /**< Data NACK */
#define SMBUS_ERR_TYPE_NACK_GCALL         (4)  /**< General Call NACK */
#define SMBUS_ERR_TYPE_ARB_LOST           (5)  /**< Arbitration lost */
#define SMBUS_ERR_TYPE_SDA_STUCK          (6)  /**< SDA stuck low */
#define SMBUS_ERR_TYPE_MASTER_DISABLED    (7)  /**< Master disabled */
#define SMBUS_ERR_TYPE_UNKNOWN            (8)  /**< Unknown abort source */

#define SMBUS_ARP_ERR_NONE                (0)
#define SMBUS_ARP_ERR_BUSY                (-EBUSY)    /* 资源正忙或被锁定 */
#define SMBUS_ARP_ERR_IO                  (-EIO)      /* 通用I/O错误 */
#define SMBUS_ARP_ERR_NACK                (-ENXIO)    /* 地址无应答(NACK) */
#define SMBUS_ARP_ERR_ARB_LOST            (-EAGAIN)   /* 总线仲裁失败 */
#define SMBUS_ARP_ERR_TIMEOUT             (-ETIMEDOUT)/* 操作超时 */
#define SMBUS_ARP_ERR_PROTO               (-EPROTO)   /* 协议错误 */

/* SDA Stuck Recovery Abort and Status Masks */
#define SMBUS_IC_TX_ABRT_SDA_STUCK_AT_LOW_MASK (1U << 17) /**< SDA stuck at low abort mask */
#define SMBUS_IC_STATUS_SDA_STUCK_NOT_RECOVERED_MASK (1U << 11) /**< SDA stuck not recovered status mask */

/* Utility Macros */
#define BIT(nr)                 (1UL << (nr))                    /**< Bit macro */
#define GENMASK(h, l)           (((~0UL) << (l)) & (~0UL >> (31 - (h))))  /**< Generate mask */

/* ===== Parameter Validation Macros ===== */
/**
 * @brief Parameter validation macro with goto exit for error handling
 * @param[in] cond Condition to check (if true, error occurs)
 * @param[in] errcode Error code to return
 * @param[in] fmt Format string for error log
 * @param[in] ... Variable arguments for format string
 */
#define SMBUS_CHECK_PARAM(cond, errcode, fmt, ...)      \
    do {                                                \
        if (cond) {                                     \
            LOGE(fmt, ##__VA_ARGS__);                   \
            ret = (errcode);                            \
            goto exit;                                  \
        }                                               \
    } while (0)

/**
 * @brief Parameter validation macro with direct return (no goto)
 * @param[in] cond Condition to check (if true, error occurs)
 * @param[in] errcode Error code to return
 * @param[in] fmt Format string for error log
 * @param[in] ... Variable arguments for format string
 */
#define SMBUS_CHECK_PARAM_RETURN(cond, errcode, fmt, ...) \
    do {                                                \
        if (cond) {                                     \
            LOGE(fmt, ##__VA_ARGS__);                   \
            return (errcode);                           \
        }                                               \
    } while (0)

/**
 * @brief Parameter validation macro for void functions
 * @param[in] cond Condition to check (if true, error occurs)
 * @param[in] fmt Format string for error log
 * @param[in] ... Variable arguments for format string
 */
#define SMBUS_CHECK_PARAM_VOID(cond, fmt, ...)         \
    do {                                                \
        if (cond) {                                     \
            LOGE(fmt, ##__VA_ARGS__);                   \
            return;                                    \
        }                                               \
    } while (0)

#ifndef UNLIKELY
    #if defined(__GNUC__) || defined(__clang__)
        #define LIKELY(x)   __builtin_expect(!!(x), 1)
        #define UNLIKELY(x) __builtin_expect(!!(x), 0)
    #else
        /* 非 GCC/Clang 编译器，不做优化 */
        #define LIKELY(x)   (x)
        #define UNLIKELY(x) (x)
    #endif
#endif

/* ===== Structure Initialization Macros ===== */
/**
 * @brief Driver data structure initialization macro
 * @details Provides standardized initialization for SmbusDrvData_s basic fields
 */
#define SMBUS_DRV_DATA_INIT {          \
    .devId               = 0,           \
    .retryCount          = 0,           \
    .enablePec           = 0,           \
    .txComplete          = 0,           \
    .rxComplete          = 0,           \
    .errorCode           = 0,           \
    .lastError           = 0,           \
    .errorCount          = 0,           \
    .slaveTransferActive = 0            \
}

/**
 * @brief SMBUS device structure initialization macro
 * @details Provides standardized initialization for SmbusDev_s basic fields
 */
#define SMBUS_DEV_INIT {              \
    .busId               = 0,         \
    .status              = 0,         \
    .cmdErr              = 0,         \
    .abortSource         = 0,         \
    .errorType           = 0,         \
    .enabled             = 0,         \
    .mode                = 0,         \
    .flags               = 0,         \
    .transferTimeout     = 0,         \
    .transferStartTime   = 0          \
}

/**
 * @brief ARP Master structure initialization macro
 * @details Provides standardized initialization for ARP Master context
 */
#define SMBUS_ARP_MASTER_INIT {     \
    .busId               = 0,        \
    .nextAddress         = 0x10,      \
    .addressPoolStart    = 0x10,      \
    .addressPoolEnd      = 0x7E,      \
    .deviceList          = NULL,      \
    .deviceCount         = 0          \
}

/* I2C Status Mask Constants */
#define SMBUS_IC_STATUS_SLAVE_ACTIVITY_MASK   (1U << 5)     /**< Slave activity status mask */
#define SMBUS_IC_STATUS_SLAVE_ACTIVITY_SHIFT  (5)           /**< Slave activity status shift */
#define SMBUS_IC_STATUS_RFNE_MASK             (1U << 3)     /**< RX FIFO not empty status mask */

/* I2C Data Command Mask Constants */
#define SMBUS_IC_DATA_CMD_FIRST_DATA_BYTE_MASK (1U << 10)   /**< First data byte mask */

/**
 * @brief SMBus error codes
 */
#define SMBUS_ERROR_TIMEOUT             (0x01)  /**< Timeout error */
#define SMBUS_ERROR_BUS_ERROR           (0x02)  /**< Bus error */
#define SMBUS_ERROR_ARBITRATION_LOST    (0x03) /**< Arbitration lost error */
#define SMBUS_ERROR_TX_ABORT            (0x04)  /**< TX abort error */

/* Slave buffer management constants */
#define SMBUS_SLAVE_BUF_LEN             (SMBUS_MAX_BUFFER_SIZE)  /**< Slave buffer length */

/* Timeout constants */
#define SMBUS_TX_READY_TIMEOUT_US     (10000U)  /**< Transmit ready timeout in microseconds */
#define SMBUS_RX_READY_TIMEOUT_US     (10000U)  /**< Receive ready timeout in microseconds */
#define SMBUS_TRANSACTION_TIMEOUT_US  (100000U) /**< Overall transaction timeout in microseconds */

/* Address constants */
#define SMBUS_ARP_ADDR                (0x61)    /**< SMBus ARP address */
#define SMBUS_HOST_NOTIFY_ADDR        (0x08)    /**< SMBus host notify address */
#define SMBUS_MIN_VALID_ADDRESS        (0x08)   /**< Minimum valid address (reserved 0x00-0x07) */
#define SMBUS_MAX_VALID_ADDRESS        (0x77)   /**< Maximum valid address */
#define SMBUS_I2C_WRITE_MODE           (0x00)   /**< I2C write mode bit */

/* Command codes */
#define SMBUS_CMD_GENERAL_RESET_DEVICE (0x02)   /**< Reset Device command code */

/* Mode constants */
#define SMBUS_MODE_MASTER              (1)       /**< Master mode */
#define SMBUS_MODE_SLAVE               (0)       /**< Slave mode */

/* Lock timeout */
#define SMBUS_LOCK_TIMEOUT_MS         (1000)    /**< Lock timeout in milliseconds */


/* ARP abort mask */
#define SMBUS_ARP_ABORT_MASK           (0x09)    /**< ARP abort source mask */

/* Default timeout */
#define SMBUS_DEFAULT_TIMEOUT_MS       (5000)    /**< Default timeout in milliseconds */

/* ======================================================================== */
/*                     Additional Timeout Constants                            */
/* ======================================================================== */
#define SMBUS_ADDR_UPDATE_TIMEOUT_CNT     (1000)     /**< Timeout for address update operations */
#define SMBUS_MODE_SWITCH_RETRY_CNT       (10)       /**< Retry count for mode switch operations */
#define SMBUS_FIFO_DEPTH_SHIFT            (8)        /**< Shift amount for FIFO depth calculation */
#define SMBUS_FIFO_DEPTH_ADD              (1)        /**< Add constant for FIFO depth calculation */
#define SMBUS_ENABLE_STATUS_TIMEOUT_CNT   (2000)     /**< Timeout for enable status check */
#define SMBUS_INTERRUPT_CLEAR_ALL         (0x7FFF)   /**< Clear all interrupts mask */
#define SMBUS_SMBUS_INTERRUPT_CLEAR_ALL   (0x7FFF)   /**< Clear all SMBus interrupts mask */
#define SMBUS_DUMMY_DATA_BYTE             (0xFF)     /**< Dummy data byte for invalid operations */
#define SMBUS_INTERNAL_CLOCK_HZ           (12500000) /**< Internal peripheral clock frequency (12.5MHz) */
#define SMBUS_MIN_SDA_HOLD_TIME           (1)        /**< Minimum SDA hold time */
#define SMBUS_DUTY_CYCLE_PERCENT_50       (50)       /**< 50% duty cycle */
#define SMBUS_SPIKE_LEN_OFFSET_HIGH       (7)        /**< High spike length offset */
#define SMBUS_SPIKE_LEN_OFFSET_LOW        (5)        /**< Low spike length offset */

/* ======================================================================== */
/*                     Address Validity Constants                            */
/* ======================================================================== */
#define SMBUS_ADDR_VALID_BIT_MASK         (0x01)     /**< Address valid bit mask */
#define SMBUS_ADDR_SHIFT_BITS             (1)        /**< Address bit shift amount */
#define SMBUS_7BIT_ADDR_MASK              (0x7F)     /**< 7-bit address mask */
#define SMBUS_8BIT_DATA_MASK              (0xFF)     /**< 8-bit data mask */
#define SMBUS_16BIT_DATA_MASK             (0xFFFF)   /**< 16-bit data mask */
#define SMBUS_BYTE_SHIFT_8                (8)        /**< 8-bit shift amount */
#define SMBUS_BYTE_SHIFT_16               (16)       /**< 16-bit shift amount */
#define SMBUS_BYTE_SHIFT_24               (24)       /**< 24-bit shift amount */

/* ======================================================================== */
/*                     Retry and Timeout Constants                           */
/* ======================================================================== */
#define SMBUS_ACTIVE_RETRY_COUNT          (50000)    /**< Retry count for active status check */
#define SMBUS_CHECK_ERROR_RETRY_COUNT     (10)       /**< Retry count for error checking */
#define SMBUS_RECOVERY_TIMEOUT_CNT        (100)      /**< Timeout for recovery operations */
#define SMBUS_ENABLE_DISABLE_TIMEOUT_CNT  (200)      /**< Timeout for enable/disable operations */
#define SMBUS_BUS_CLEAR_TIMEOUT_CNT       (1000)     /**< Timeout for bus clear operations */
#define SMBUS_CLOCK_EXTEND_TIMEOUT_CNT    (1000)     /**< Timeout for clock extend operations */

/* ======================================================================== */
/*                     Command and Data Constants                            */
/* ======================================================================== */
#define SMBUS_ARP_PREPARE_PAYLOAD_BYTE    (0x00)     /**< Prepare ARP payload byte */
#define SMBUS_SLAVE_INVALID_TX_LEN        (0)        /**< Invalid slave TX length */
#define SMBUS_SLAVE_DEFAULT_TX_LEN        (0)        /**< Default slave TX length */
#define SMBUS_SLAVE_INVALID_NEW_ADDR      (0x00)     /**< Invalid new slave address */
#define SMBUS_SLAVE_RESET_OLD_ADDR        (0xFFFF)   /**< Reset old address marker */
#define SMBUS_SLAVE_INVALID_ADDR_TAG      (0xFF)     /**< Invalid address tag */

/* ======================================================================== */
/*                     Status and Flag Constants                             */
/* ======================================================================== */
#define SMBUS_SLAVE_ENABLED               (1)        /**< Slave enabled flag */
#define SMBUS_SLAVE_DISABLED              (0)        /**< Slave disabled flag */
#define SMBUS_QUICK_COMMAND_FLAG          (1)        /**< Quick command flag */
#define SMBUS_MASTER_MODE_FLAG            (1)        /**< Master mode flag */
#define SMBUS_WRITE_OPERATION             (0)        /**< Write operation flag */
#define SMBUS_READ_OPERATION              (1)        /**< Read operation flag */
#define SMBUS_INTERRUPT_POLLING_MODE      (1)        /**< Interrupt polling mode */
#define SMBUS_10BIT_ADDRESS_MODE          (1)        /**< 10-bit address mode flag */

/* ======================================================================== */
/*                     CRC and Checksum Constants                            */
/* ======================================================================== */
#define SMBUS_CRC8_POLYNOMIAL_HIGHBIT     (0x80)     /**< CRC-8 polynomial high bit */
#define SMBUS_CRC8_POLYNOMIAL_VALUE       (0x07)     /**< CRC-8 polynomial value */
#define SMBUS_CRC8_INITIAL_VALUE          (0x00)     /**< CRC-8 initial value */

/* ======================================================================== */
/*                    SMBus Master Configuration Macros                      */
/* ======================================================================== */

/* Control Register Bit Position Macros */
#define SMBUS_IC_CON_MASTER_MODE_EN_BIT       (0)   /**< Master mode enable bit position */
#define SMBUS_IC_CON_SLAVE_DISABLE_BIT        (6)   /**< Slave disable bit position */
#define SMBUS_IC_CON_RESTART_EN_BIT           (5)   /**< Restart enable bit position */
#define SMBUS_IC_CON_10BIT_MASTER_ADDR_BIT    (4)   /**< 10-bit master addressing bit position */
#define SMBUS_IC_CON_10BIT_SLAVE_ADDR_BIT     (3)   /**< 10-bit slave addressing bit position */
#define SMBUS_IC_CON_RX_FIFO_HOLD_BIT         (9)   /**< RX FIFO full hold control bit position */
#define SMBUS_IC_CON_STOP_DET_IFADDRESSED_BIT (7)   /**< STOP detection when addressed bit position */
#define SMBUS_IC_CON_ARP_ENABLE_BIT           (18)  /**< ARP enable bit position */
#define SMBUS_IC_CON_QUICK_CMD_BIT            (17)  /**< SMBus quick command enable bit position */

/* Speed Configuration Values for IC_CON[2:1] - Using bit shift format */
#define SMBUS_SPEED_STANDARD_CFG              (1U << 1)  /**< Standard speed (100 kHz) */
#define SMBUS_SPEED_FAST_CFG                  (2U << 1)  /**< Fast speed (400 kHz) */
#define SMBUS_SPEED_FAST_PLUS_CFG             (3U << 1)  /**< Fast Plus speed (1 MHz) - FIXED! */
#define SMBUS_SPEED_HIGH_CFG                  (3U << 1)  /**< High speed (3.4 MHz) */

/* Speed Configuration Masks */
#define SMBUS_IC_CON_SPEED_MASK               (0x06)      /**< Speed mask for bits [2:1] */
#define SMBUS_IC_CON_SPEED_STD                (1U << 1)   /**< Standard speed configuration */
#define SMBUS_IC_CON_SPEED_FAST               (2U << 1)   /**< Fast speed configuration */
#define SMBUS_IC_CON_SPEED_HIGH               (3U << 1)   /**< High speed configuration */

/* Bus Clear Control Mask */
#define SMBUS_IC_CON_BUS_CLEAR_CTRL_MASK      (1U << 20)  /**< Bus clear control bit mask */

/* ======================================================================== */
/*                    SMBus Enable Register Macros                           */
/* ======================================================================== */

/* IC_ENABLE Register Bit Position Macros */
#define SMBUS_IC_ENABLE_BIT                    (0)   /**< Enable bit position */
#define SMBUS_IC_TX_CMD_BLOCK_BIT              (2)   /**< TX command block bit position */
#define SMBUS_IC_SDA_STUCK_RECOVERY_BIT        (3)   /**< SDA stuck recovery bit position */
#define SMBUS_IC_SMBUS_CLK_RESET_BIT           (16)  /**< SMBus clock line reset bit position */
#define SMBUS_IC_SMBUS_SUSPEND_BIT             (17)  /**< SMBus suspend bit position */
#define SMBUS_IC_SMBUS_ALERT_BIT               (18)  /**< SMBus alert bit position */
#define SMBUS_IC_SAR_ENABLE_BIT                (19)  /**< SAR enable bit position */
#define SMBUS_IC_SAR2_ENABLE_BIT               (20)  /**< SAR2 enable bit position */
#define SMBUS_IC_SAR3_ENABLE_BIT               (21)  /**< SAR3 enable bit position */
#define SMBUS_IC_SAR4_ENABLE_BIT               (22)  /**< SAR4 enable bit position */

/* IC_ENABLE Register Bit Mask Macros */
#define SMBUS_IC_ENABLE_MASK                   (1U << SMBUS_IC_ENABLE_BIT)               /**< Enable mask */
#define SMBUS_IC_TX_CMD_BLOCK_MASK             (1U << SMBUS_IC_TX_CMD_BLOCK_BIT)         /**< TX command block mask */
#define SMBUS_IC_SDA_STUCK_RECOVERY_MASK       (1U << SMBUS_IC_SDA_STUCK_RECOVERY_BIT)   /**< SDA stuck recovery mask */
#define SMBUS_IC_SMBUS_CLK_RESET_MASK          (1U << SMBUS_IC_SMBUS_CLK_RESET_BIT)      /**< SMBus clock line reset mask */
#define SMBUS_IC_SMBUS_SUSPEND_MASK            (1U << SMBUS_IC_SMBUS_SUSPEND_BIT)        /**< SMBus suspend mask */
#define SMBUS_IC_SMBUS_ALERT_MASK              (1U << SMBUS_IC_SMBUS_ALERT_BIT)          /**< SMBus alert mask */
#define SMBUS_IC_SAR_ENABLE_MASK               (1U << SMBUS_IC_SAR_ENABLE_BIT)           /**< SAR enable mask */
#define SMBUS_IC_SAR2_ENABLE_MASK              (1U << SMBUS_IC_SAR2_ENABLE_BIT)          /**< SAR2 enable mask */
#define SMBUS_IC_SAR3_ENABLE_MASK              (1U << SMBUS_IC_SAR3_ENABLE_BIT)          /**< SAR3 enable mask */
#define SMBUS_IC_SAR4_ENABLE_MASK              (1U << SMBUS_IC_SAR4_ENABLE_BIT)          /**< SAR4 enable mask */

/* Enable Status Mask */
#define SMBUS_IC_ENABLE_STATUS_IC_EN          (0x01)      /**< Enable status bit mask */

/**
 * @brief Clear essential SMBus interrupt status bits (performance optimized)
 * @param[in] regBase Pointer to SMBus register base address
 *
 * This macro clears only the most critical interrupt bits that are
 * commonly encountered during normal operation, providing better
 * performance for frequent interrupt handling.
 */
#define SMBUS_CLEAR_ESSENTIAL_INTERRUPTS(regBase) do { \
    (void)(regBase)->icClrIntr;      /* Clear combined interrupt */ \
    (void)(regBase)->icClrTxAbrt;    /* Clear TX abort */ \
    (void)(regBase)->icClrRxDone;    /* Clear RX done */ \
    (void)(regBase)->icClrActivity;  /* Clear activity */ \
} while(0)

/**
 * @brief Clear interrupts based on mask (modular function for both I2C and SMBus)
 * @details Clears specific interrupts based on the provided mask. This function
 *          provides a modular approach to interrupt clearing that can be inherited
 *          by both I2C and SMBus drivers.
 * @param[in] regBase Pointer to SMBus register map
 * @param[in] mask Interrupt mask specifying which interrupts to clear
 * @return void
 *
 * @note If mask equals STARS_I2C_STATUS_INT_ALL, reads ic_clr_intr to clear all interrupts
 * @note Clears specific interrupts based on individual mask bits:
 *       - m_rx_under: Read ic_clr_rx_under
 *       - m_rx_over: Read ic_clr_rx_over
 *       - m_tx_over: Read ic_clr_tx_over
 *       - m_rd_req: Read ic_clr_rd_req
 *       - m_tx_abrt: Read ic_clr_tx_abrt
 *       - m_rx_done: Read ic_clr_rx_done
 *       - m_activity: Read ic_clr_activity
 *       - m_stop_det: Read ic_clr_stop_det
 *       - m_start_det: Read ic_clr_start_det
 *       - m_gen_call: Read ic_clr_gen_call
 *       - m_restart_det: Read ic_clr_restart_det
 * @note For SMBus, also clears ic_clr_smbus_intr with the mask
 * @warning This function should be called with valid register base
 */

/* 中断位到寄存器偏移的映射数组（按位索引） */
static volatile U32* const INTR_CLR_REGS[] = {
    [0]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrRxUnder,    /* RX_UNDER */
    [1]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrRxOver,     /* RX_OVER */
    [3]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrTxOver,     /* TX_OVER */
    [5]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrRdReq,      /* RD_REQ */
    [6]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrTxAbrt,     /* TX_ABRT */
    [7]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrRxDone,     /* RX_DONE */
    [8]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrActivity,   /* ACTIVITY */
    [9]  = (volatile U32*)&((SmbusRegMap_s*)0)->icClrStopDet,    /* STOP_DET */
    [10] = (volatile U32*)&((SmbusRegMap_s*)0)->icClrStartDet,   /* START_DET */
    [11] = (volatile U32*)&((SmbusRegMap_s*)0)->icClrGenCall,    /* GEN_CALL */
    [12] = (volatile U32*)&((SmbusRegMap_s*)0)->icClrRestartDet, /* RESTART_DET */
};

/* Speed Threshold Values (Hz) */
#define SMBUS_SPEED_HIGH_THRESHOLD            (3400000U)  /**< High speed threshold (3.4 MHz) */
#define SMBUS_SPEED_FAST_PLUS_THRESHOLD       (1000000U)  /**< Fast Plus speed threshold (1 MHz) */
#define SMBUS_SPEED_FAST_THRESHOLD            (400000U)   /**< Fast speed threshold (400 kHz) */
#define SMBUS_SPEED_STANDARD_THRESHOLD        (100000U)   /**< Standard speed threshold (100 kHz) */

/* Supported Speed Values (Hz) */
#define SMBUS_SPEED_100KHZ                    (100000U)   /**< Standard speed (100 kHz) */
#define SMBUS_SPEED_400KHZ                    (400000U)   /**< Fast speed (400 kHz) */
#define SMBUS_SPEED_1MHZ                      (1000000U)  /**< Fast Plus speed (1 MHz) */

/* Speed validation limits */
#define SMBUS_SPEED_MIN                        SMBUS_SPEED_100KHZ      /**< Minimum supported speed */
#define SMBUS_SPEED_MAX                        SMBUS_SPEED_1MHZ        /**< Maximum supported speed */

/* I2C Reserved Address Ranges */
#define SMBUS_RESERVED_ADDR_START              (0x78)     /**< Start of reserved address range */
#define SMBUS_RESERVED_ADDR_END                (0x7F)     /**< End of reserved address range */
#define SMBUS_GENERAL_CALL_ADDR                (0x00)     /**< General call address (reserved) */

/* SDA Stuck Recovery Constants */
#define SMBUS_SDA_STUCK_RECOVERY_TIMEOUT_MS    (15)        /**< SDA stuck recovery timeout in milliseconds */
#define SMBUS_SDA_STUCK_CHECK_DELAY_US         (10)        /**< SDA stuck check delay in microseconds */

/* Address mode values */
#define SMBUS_ADDR_MODE_7BIT                   (0)        /**< 7-bit addressing mode */
#define SMBUS_ADDR_MODE_10BIT                  (1)        /**< 10-bit addressing mode */

/* Boolean validation limits */
#define SMBUS_BOOL_MAX                         (1)        /**< Maximum boolean value */

/* Device status mask constants */
#define SMBUS_STATUS_SLAVE_ENABLED_MASK    (0x02)   /**< Slave enabled status mask */
/* ======================================================================== */
/*                           Structure Definitions                          */
/* ======================================================================== */
/* Configure clock rate based on speed setting using enum+lookup table */
typedef enum {
    SMBUS_SPEED_MODE_STANDARD = 0,   /* 100 kHz */
    SMBUS_SPEED_MODE_FAST     = 1,   /* 400 kHz */
    SMBUS_SPEED_MODE_FAST_PLUS = 2,  /* 1 MHz */
    SMBUS_SPEED_MODE_MAX
} SmbusSpeedMode_e;

typedef struct SmbusMsg {
  U16 addr;
  U16 flags;
  U16 len;
  U8 *buf;
}SmbusMsg_s;
/**
 * @brief SMBus device structure (wrapper around adapter)
 */
typedef struct SmbusDev {
    volatile SmbusRegMap_s  *regBase;                               /**< Register base address */
    U32                      busId;                                  /**< Bus channel number */
    U32                      flags;                                 /**< Device flags */
    SmbusMode_e              mode;                                  /**< Operation mode (0=master, 1=slave) */
    U32                      addrMode;                              /**< Address mode (7/10 bit) */
    U32                      slaveAddr;                             /**< Slave address */
    U8                       status;
    U32                      xferStatus;                            /**< Device status */
    S32                      cmdErr;                                /**< Command error code */
    U8                       isQuick;
    U32                      abortSource;                           /**< Abort source register value */
    U32                      errorType;                             /**< Specific error type (NACK, ARB_LOST, etc.) */
    SmbusMsg_s		         msgs[SMBUS_MAX_BLOCK_LEN + 1];         /**< SMBus messages parameters */

    U32                      irq;                /**< Interrupt number */
    U32                      channelNum;         /**< Channel number for compatibility */
    U8                       cmdReg;
    U8                       workMode;           /**< Work mode (0=interrupt, 1=polling) */
    SmbusFeatureConfig_s     smbFeatures;        /**< SMBus feature configuration (PEC, ARP, Host Notify) */
    U8                       restartEnb;         /**< Device restart enabled status */
    U32                      masterCfg;          /**< Master configuration register value */
    U32                      slaveCfg;           /**< Slave configuration register value */
    U32                      txFifoDepth;        /**< TX FIFO depth */
    U32                      rxFifoDepth;        /**< RX FIFO depth */
    void                     *isrHandler;        /**< Interrupt service routine handler */
    OspID                    semaphoreId;        /**< Semaphore ID for synchronization */

    ///< Async transfer state fields (similar to I2C implementation)
    U32                      msgsNum;            /**< Number of messages */
    U32                      msgWriteIdx;        /**< Current write message index */
    U32                      msgReadIdx;         /**< Current read message index */
    S32                      msgErr;             /**< Message error code */
    U32                      rxOutstanding;      /**< Outstanding RX bytes */
    U32                      transferTimeout;    /**< Transfer timeout value */
    U32                      transferStartTime;  /**< Transfer start time for timeout protection */

    U8                       *slaveRxBuf;        /**< Slave RX buffer */
    U8                       *slaveTxBuf;        /**< Slave TX buffer */
    U32                      slaveValidRxLen;    /**< Valid RX buffer length */
    U32                      slaveValidTxLen;    /**< Valid TX buffer length */
    U32                      txIndex;            /**< Current TX buffer index */
    U8                       masterTxBuf[33];    // 当前正在发送的消息缓冲区位置
    U32                      masterTxBufLen;     // 当前消息剩余待发送长度
    U32                      sdaHoldTime;        /**< SDA hold time configuration */
    U32                      clkRate;            /**< Clock rate for timing calculations */

    /* SMBus timing configuration fields (ported from I2C) */
    U16                      ssHcnt;             /**< Standard mode SCL high count */
    U16                      ssLcnt;             /**< Standard mode SCL low count */
    U16                      fsHcnt;             /**< Fast mode SCL high count */
    U16                      fsLcnt;             /**< Fast mode SCL low count */
    U16                      hsHcnt;             /**< High speed SCL high count */
    U16                      hsLcnt;             /**< High speed SCL low count */
    U16                      fpHcnt;             /**< Fast mode plus SCL high count */
    U16                      fpLcnt;             /**< Fast mode plus SCL low count */
    U16                      fsSpklen;           /**< Fast mode spike suppression length */
    U16                      hsSpklen;           /**< High speed spike suppression length */

    /* HAL Operations - pointer to HAL function table */
    SmbusHalOps_s            *halOps;            /**< HAL operations function table */
} SmbusDev_s;

/* SMBus status constants for async operations */
#define SMBUS_STATUS_IDLE               (0x00000000)
#define SMBUS_STATUS_ACTIVE             (0x00000001)  ///< Transfer in progress
#define SMBUS_STATUS_READ_IN_PROGRESS   (0x00000002)  ///< Read operation in progress
#define SMBUS_STATUS_WRITE_IN_PROGRESS  (0x00000004)  ///< Write operation in progress
#define SMBUS_STATUS_MASK               (0x00000007)  ///< Status mask

/* SMBus error codes */
#define SMBUS_ERR_TX_ABRT               (0x00000003)  ///< TX abort error
#define SMBUS_ERR_RX_UNDER              (0x00000005)  ///< RX underflow error
#define SMBUS_ERR_RX_OVER               (0x00000006)  ///< RX overflow error
#define SMBUS_ERR_TX_OVER               (0x00000008)  ///< TX overflow error
#define SMBUS_ERR_ACTIVITY              (0x00000010)  ///< Activity error

/**
 * @brief Host Notify 数据结构
 */
typedef struct SmbusHostNotifyData {
    U8  deviceAddr;         /**< 发送通知的设备地址 */
    U16 data;               /**< 通知数据 (低字节在前) */
} SmbusHostNotifyData_s;

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
 * @brief ARP状态枚举
 */
typedef enum SmbusArpState {
    SMBUS_ARP_STATE_IDLE = 0,              /**< 空闲 */
    SMBUS_ARP_STATE_RESET,                 /**< ARP reset 接收 */
    SMBUS_ARP_STATE_PREPARED,              /**< ARP prepare 接收 */
    SMBUS_ARP_STATE_ASSIGNED,              /**< Address 分配 */
    SMBUS_ARP_STATE_WAITING,               /**< 等待总线 */
    SMBUS_ARP_STATE_COMPETING,             /**< 竞争中 */
    SMBUS_ARP_STATE_WON,                   /**< 仲裁获胜 */
    SMBUS_ARP_STATE_LOST,                  /**< 仲裁失败 */
    SMBUS_ARP_STATE_RETRYING,              /**< 重试中 */
    SMBUS_ARP_STATE_RESERVED               /**< 保留字段 */
} SmbusArpState_e;

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
 * @brief SMBus Driver Data Structure
 */
typedef struct SmbusDrvData {
    U8                     devId;           /**< device number include device list */
    U8                     retryCount;      /**< Current retry count */
    U8                     enablePec;       /**< PEC enable */
    U8                     enableArp;       /**< Reserved for Arp */
    U8                     txComplete;      /**< tx COMPLETE detect flag */
    U8                     slaveTransferActive;
    U8                     *rxBuffer;
    U8                     rxLength;
    U8                     rxComplete;
    U8                     errorCode;
    U8                     lastError;
    SbrI2cSmbusCfg_s       sbrCfg;          /**< SBR configuration */
    bool                   arpEnabled;      /**< ARP enabled flag */
    bool                   arpInitialized;  /**< ARP initialized flag */
    SmbusArpState_e        arpState;        /**< ARP state */
    U16                    assignedAddr;
    void                   *arpContext;     /**< ARP context */
  
    SmbusDev_s             pSmbusDev;      /**< SMBus device context */
    SmbusArpMaster_s       arpMaster;       /**< ARP master context */
    SmbusUdid_s            udid;

    U16                    errorCount;
    U16                    rxBufferSize;
    SmbusCallback_t        callback;         /*common interface */
    void                   *userData;        /**< User data for callback */
} SmbusDrvData_s;

/**
 * @brief SMBus timeout type enumeration
 */
typedef enum SmbusTimeoutType {
    SMBUS_TIMEOUT_TYPE_CLOCK_EXTEND = 0,  /**< Clock extend timeout */
    SMBUS_TIMEOUT_TYPE_CLOCK_LOW    = 1,  /**< Clock low timeout */
} SmbusTimeoutType_e;

typedef struct dw_i2c_dev dwI2cDev_s;
/**
 * @brief Handle ARP Assign Address command finish (slave side)
 * @note Migrated from i2c_designware_slave.c:dwI2cSlvArpAssignAddrFinishHandle()
 */
void smbusSlvArpAssignAddrFinishHandle(SmbusDrvData_s *pDrvData);

/**
 * @brief Calculate CRC-8 for one byte of data
 * @details Calculates CRC-8 checksum for a single byte using the SMBus
 *          polynomial (0x07). This is a low-level function used by higher
 *          level PEC calculation functions.
 * @param[in] crc Current CRC value
 * @param[in] data Data byte to include in CRC calculation
 * @return Updated CRC value
 *
 * @note Uses SMBus CRC-8 polynomial (0x1070)
 * @note Processes 8 bits with polynomial division
 */
static inline U8 smbusCrc8CalcOne(U8 crc, U8 data)
{
    crc ^= data;
    for (U8 i = 0; i < 8; ++i) {
        if (crc & SMBUS_CRC8_POLY_HIGHBIT)
            crc = (U8)(crc << 1) ^ CRC8_POLY;
        else
            crc <<= 1;
    }
    return crc;
}
/**
 * @brief Convert I2C 7-bit address to read/write address byte
 * @details Converts a 7-bit I2C address to an 8-bit address byte with the
 *          appropriate read/write bit set.
 * @param[in] addrIn 7-bit I2C address (0-127)
 * @param[in] isWrite True for write operation, false for read operation
 * @return 8-bit address with R/W bit set
 *
 * @note Left-shifts address by 1 and adds R/W bit
 * @note Write operation: R/W bit = 0, Read operation: R/W bit = 1
 */
static inline const U8 i2cAddrConvert(U8 addrIn, bool isWrite)
{
    return (addrIn << 1) | (isWrite ? 0 : 1);
}


/**
 * @brief Master read data from RX FIFO
 * @details Reads data from RX FIFO and stores in driver data structure
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @return void
 */
void smbusMasterReadData(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase);

/**
 * @brief Master transfer data to TX FIFO
 * @details Transfers data from TX buffer to TX FIFO
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @return void
 */
void smbusMasterTransferData(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase);

/**
 * @brief Trigger slave event callback
 * @details Triggers appropriate SMBus callback for slave events
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] eventType Event type to trigger
 * @param[in] data Event data pointer
 * @param[out] len Data length
 * @return void
 */
void smbusTriggerSlaveEvent(SmbusDrvData_s *pDrvData, U32 eventType, void *data, U32 len);

/**
 * @brief Handle master specific SMBus interrupts
 * @details Processes SMBus-specific interrupts in master mode
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @param[in] smbusIntrStat SMBus interrupt status
 * @return void
 */
void smbusHandleMasterSpecificInterrupts(SmbusDrvData_s *pDrvData,
                                         volatile SmbusRegMap_s *regBase,
                                         U32 smbusIntrStat);

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
                                        U32 smbusIntrStat);

/**
 * @brief Clear and free all discovered devices in ARP master device list
 * @details This function safely deallocates all memory associated with the
 *          device list maintained by the ARP master. It iterates through the
 *          linked list of discovered devices, frees each node, and resets the
 *          ARP master state to its initial condition.
 * @param[in,out] master Pointer to the ARP master structure to be cleaned up
 * @return void
 *
 * @note This function should be called during ARP master deinitialization
 *       or when resetting the ARP discovery process
 * @note All memory allocated for device nodes will be freed
 * @warning After calling this function, all previously discovered devices
 *          will be lost and must be rediscovered
 * @warning The function is not thread-safe and should be called with
 *          appropriate synchronization if accessed from multiple threads
 */
static inline void smbusClearDevs(SmbusArpMaster_s *master)
{
    SmbusArpDeviceNode_s *nodeList = NULL;
    SmbusArpDeviceNode_s *next = NULL;

    SMBUS_CHECK_PARAM_VOID(master == NULL, "master is NULL");

    nodeList = master->deviceList;
    while (nodeList != NULL) {
        next = nodeList->next;
        free(nodeList);
        nodeList = next;
    }

    master->deviceList = NULL;
    master->deviceCount = 0;
    master->nextAddress = master->addressPoolStart;
}

/**
 * @brief Check if address is already used in ARP master device list
 * @param[in] master Pointer to ARP master structure
 * @param[in] address Address to check
 * @return TRUE if address is used, FALSE otherwise
 */
static inline Bool smbusArpIsAddrUsed(SmbusArpMaster_s *master, U8 address)
{
    SmbusArpDeviceNode_s *node = NULL;

    if (master == NULL) {
        return TRUE;
    }

    node = master->deviceList;
    while (node != NULL) {
        if (node->currentAddress == address) {
            return TRUE;
        }
        node = node->next;
    }

    return FALSE;
}

/**
 * @brief 解析 Assign Address 命令的 Payload
 * @note  SMBus ARP "Assign Address" 包结构: 
 * [Command] [ByteCount] [UDID (16 Bytes)] [Assigned Address (1 Byte)] [PEC]
 * 这里的 payload 指针应当指向 UDID 的起始位置。
 * * @param payload    指向接收buffer中 UDID 的第0个字节
 * @param targetUdid [输出] 解析出的目标 UDID 数据 (只填充前16字节)
 * @param newAddr    [输出] 解析出的新 7-bit 地址
 */
static inline void parseAssignPacket(const U8 *payload, SmbusUdid_s *targetUdid, U8 *newAddr)
{
    SMBUS_CHECK_PARAM_VOID(payload == NULL || targetUdid == NULL || newAddr == NULL,
                      "payload, targetUdid or newAddr is NULL");

    /* * 1. 填充 UDID 字段
     * 您的结构体是 packed 的，且前 10 个字节字段(nextAvailAddr 到 subsystemVendorId)
     * 与 SMBus 标准 UDID 布局一致。
     * 剩余的 6 个标准 UDID 字节 (Subsystem Device ID + Vendor Specific ID)
     * 将会被自动拷贝到 targetUdid->bytes[0] 到 targetUdid->bytes[5] 中。
     */
    memcpy(targetUdid, payload, 16);

    /* * 2. 提取分配的新地址
     * 根据 SMBus ARP 协议 (Section 5.6.3):
     * Payload 的第 17 个字节 (index 16) 是 "Assigned Address"。
     * 格式为: [Addr 7-bit] | [Reserved(0)]。即地址左移了1位。
     */
    U8 rawAddrByte = payload[16];

    ///< 右移1位提取 7-bit 实际地址
    *newAddr = (rawAddrByte >> 1) & 0x7F;
}

/**
 * @brief 判断数据包中的 UDID 是否匹配本机 UDID
 * @note  只比较标准 SMBus UDID 定义的 16 个字节。
 * * @param myUdid     本机设备的 UDID 信息
 * @param targetUdid 从 ARP 包中解析出的目标 UDID
 * @return TRUE 匹配, FALSE 不匹配
 */
static inline Bool isMyUdid(const SmbusUdid_s *myUdid, const SmbusUdid_s *targetUdid)
{
    if (myUdid == NULL || targetUdid == NULL) {
        return FALSE;
    }

    /* * 核心逻辑：只比较前 16 个字节。
     * 您的结构体大于 16 字节，后面的 bytes[6-15] 和 deviceAddr 
     * 是本地数据，不属于 ARP 协议由于区分设备的唯一标识符 (UDID) 范畴。
     */
    if (memcmp(myUdid, targetUdid, 16) == 0) {
        return TRUE;
    }

    return FALSE;
}

/* ======================================================================== */
/*                        Static Inline Functions                           */
/* ======================================================================== */

/**
 * @brief Create SMBus semaphore with comprehensive logging and error handling
 * @param pSmbusDev Pointer to SMBus device structure
 * @param context Optional context string for logging (NULL for default)
 * @return 0 on success, -EIO on failure
 *
 * This static inline function encapsulates the common pattern of:
 * - Creating an RTEMS semaphore with fixed name
 * - Comprehensive debug logging
 * - Error handling with semaphore ID cleanup
 * - Consistent return value handling
 *
 * Usage Example:
 * @code
 * S32 ret = smbusCreateSemaphore(&pDrvData->pSmbusDev, "Init");
 * if (ret != 0) {
 *     return ret;  // Error already logged
 * }
 * @endcode
 */
static inline S32 smbusCreateSemaphore(SmbusDev_s *pSmbusDev, const char *context)
{
    pSmbusDev->busId = pSmbusDev->channelNum - (U32)DEVICE_SMBUS0;  // Ensure busId is set from channelNum
    
    rtems_name semName = rtems_build_name('S', 'M', 'B', '0' + pSmbusDev->busId);

    LOGD("%s%s: Creating semaphore with channelNum=%d, semName=0x%08X\n",
         context ? context : "", __func__, pSmbusDev->busId, semName);

    S32 semRet = rtems_semaphore_create(semName, 0,
                                        RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_FIFO, 0,
                                        &(pSmbusDev->semaphoreId));
    LOGD("%s%s: rtems_semaphore_create returned %d\n",
         context ? context : "", __func__, semRet);

    if (semRet != RTEMS_SUCCESSFUL) {
        LOGE("%s%s: Semaphore creation failed, ret=%d, channelNum=%d\n",
             context ? context : "", __func__, semRet, pSmbusDev->channelNum);
        /* Ensure semaphoreId is set to 0 on failure */
        pSmbusDev->semaphoreId = 0;
        return -EIO;
    }

    LOGD("%s%s: Semaphore created successfully, ID=%d, channelNum=%d\n",
         context ? context : "", __func__, pSmbusDev->semaphoreId, pSmbusDev->channelNum);

    return 0;
}

/**
 * @brief Delete SMBus semaphore with comprehensive logging
 * @param pSmbusDev Pointer to SMBus device structure
 * @param context Optional context string for logging (NULL for default)
 * @return 0 on success, -EIO on failure
 *
 * This static inline function encapsulates semaphore deletion with proper
 * logging and error handling for consistent cleanup behavior.
 */
static inline S32 smbusDeleteSemaphore(SmbusDev_s *pSmbusDev, const char *context)
{
    if (pSmbusDev->semaphoreId == 0) {
        /* Already deleted or never created */
        return 0;
    }

    S32 semRet = rtems_semaphore_delete(pSmbusDev->semaphoreId);
    if (semRet == RTEMS_SUCCESSFUL) {
        LOGD("%s%s: Semaphore deleted successfully\n", context ? context : "", __func__);
    } else {
        LOGW("%s%s: Semaphore deletion failed, ret=%d\n", context ? context : "", __func__, semRet);
        return -EIO;
    }

    /* Clear semaphore ID */
    pSmbusDev->semaphoreId = 0;
    return 0;
}

/**
 * @brief Release SMBus semaphore with comprehensive logging
 * @param pSmbusDev Pointer to SMBus device structure
 * @param context Optional context string for logging (NULL for default)
 * @return 0 on success, -EIO on failure
 *
 * This static inline function encapsulates semaphore release with proper
 * logging for debugging and error tracking.
 */
static inline S32 smbusReleaseSemaphore(SmbusDev_s *pSmbusDev, const char *context)
{
    if (pSmbusDev->semaphoreId == 0) {
        LOGE("%s%s: Invalid semaphoreId (0)\n", context ? context : "", __func__);
        return -EIO;
    }

    S32 releaseResult = rtems_semaphore_release(pSmbusDev->semaphoreId);
    LOGD("%s%s: Semaphore release result=%d (semId=%d)\n",
         context ? context : "", __func__, releaseResult, pSmbusDev->semaphoreId);

    if (releaseResult != RTEMS_SUCCESSFUL) {
        LOGE("%s%s: Semaphore release failed! result=%d\n",
             context ? context : "", __func__, releaseResult);
        return -EIO;
    }

    return 0;
}

/**
 * @brief Generic inline helper to execute I2C transfer via HAL
 * @param[in] dev SMBus device handle
 * @param[in] msgs Array of I2C messages
 * @param[in] num Number of messages in array
 * @return Result from i2cTransfer or -ENOTSUP if HAL not available
 * @note This inline function validates HAL availability before calling transfer
 */
static inline S32 smbusExecuteTransfer(SmbusDev_s *dev, SmbusMsg_s *msgs, S32 num)
{
    if (dev->halOps == NULL || dev->halOps->i2cTransfer == NULL) {
        return -ENOTSUP;
    }
    return dev->halOps->i2cTransfer(dev, msgs, num);
}

/**
 * @brief Get SMBus register base address
 * @param devId Device identifier
 * @param pCtrlReg Pointer to register map pointer
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
S32 getSmbusReg(DevList_e devId, SmbusRegMap_s **pCtrlReg);

#ifdef __cplusplus
}
#endif

#endif /* __SMBUS_DW_H__ */
