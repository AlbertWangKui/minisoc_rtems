/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_smbus_dw.h
 * @author wangkui (wangkui@starsmicrosystem.com)
 * @date 2025/10/20
 * @brief SMBus protocol core logic header file
 *
 * @par ChangeLog:
 * Date         Author          Description
 * 2025/10/20   wangkui         Initial version
 */

#ifndef __DRV_SMBUS_DW_H__
#define __DRV_SMBUS_DW_H__

#include "osp_types.h"
#include "bsp_sbr.h"
#include <stdbool.h>
#include "drv_smbus_api.h"

#define TEST_SUITS_1
#ifdef TEST_SUITS_1
    #define I2C_TESTSUITE_SLAVE_ADDR (0x21) /* test slave address 0x21 == 33 */
    #define TEST_SMBUS_DEVICE_ID     (DEVICE_SMBUS0)
    #define TEST_I2C_DEVICE_ID       (DEVICE_SMBUS0) /* I2C bus 0 */
    #define SYS_INT_NUM_SMBUS        (55)
    #define SYS_INT_PRIORITY_SMBUS   (127)
    #define SMBUS_BASE_OFFSET        (0x1000)
    #define SMBUS_BASE_ADDR          (0xBE620000)
#endif
/* Forward declaration for SmbusRegMap structure */
typedef struct SmbusRegMap SmbusRegMap_s;

#ifdef __cplusplus
extern "C" {
#endif

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
/*                       SMBus Hardware Timing Constants                      */
/* ======================================================================== */
/* SCL count values for different speed modes (ported from I2C) */
#define SMBUS_HW_CLK_H100             (1000)     /**< Standard mode (100kHz) SCL high count */
#define SMBUS_HW_CLK_L100             (1000)     /**< Standard mode (100kHz) SCL low count */
#define SMBUS_HW_CLK_H400             (250)      /**< Fast mode (400kHz) SCL high count */
#define SMBUS_HW_CLK_L400             (250)      /**< Fast mode (400kHz) SCL low count */
#define SMBUS_HW_CLK_H1000            (100)      /**< Fast mode plus (1MHz) SCL high count */
#define SMBUS_HW_CLK_L1000            (100)      /**< Fast mode plus (1MHz) SCL low count */
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
#define CRC8_POLY              (0x1070U << 3)

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

/* SAR Register Constants */
#define SMBUS_SAR1_INTR_BIT            (1U << 4)     /**< SAR1 interrupt bit */
#define SMBUS_SAR2_INTR_BIT            (1U << 5)     /**< SAR2 interrupt bit */
#define SMBUS_ARP_INTR_BIT             (1U << 6)     /**< ARP interrupt bit */
#define SMBUS_GET_UDID_INTR_BIT        (1U << 7)     /**< Get UDID interrupt bit */
#define SMBUS_ASSIGN_ADDR_INTR_BIT     (1U << 8)     /**< Assign Address interrupt bit */
#define SMBUS_HOST_NOTIFY_INTR_BIT     (1U << 9)     /**< Host notify interrupt bit */
#define SMBUS_ALERT_INTR_BIT           (1U << 10)    /**< Alert interrupt bit */
#define SMBUS_SUSPEND_INTR_BIT         (1U << 10)    /**< Suspend interrupt bit */

/* Timeout Interrupt Bits */
#define SMBUS_MST_CLK_EXT_TIMEOUT_BIT  (1U << 0)    /**< Master clock extend timeout */
#define SMBUS_MST_CLK_LOW_TIMEOUT_BIT  (1U << 1)    /**< Master clock low timeout */
#define SMBUS_SLV_CLK_EXT_TIMEOUT_BIT  (1U << 2)    /**< Slave clock extend timeout */
#define SMBUS_SLV_CLK_LOW_TIMEOUT_BIT  (1U << 3)    /**< Slave clock low timeout */

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

/* I2C Interrupt Mask Constants */
#define SMBUS_IC_INTR_RX_UNDER_MASK       (1 << 0)      /**< RX under interrupt mask - 0x001 */
#define SMBUS_IC_INTR_RX_OVER_MASK        (1 << 1)      /**< RX over interrupt mask - 0x002 */
#define SMBUS_IC_INTR_RX_FULL_MASK        (1 << 2)      /**< RX full interrupt mask - 0x004 */
#define SMBUS_IC_INTR_TX_OVER_MASK        (1 << 3)      /**< TX over interrupt mask - 0x008 */
#define SMBUS_IC_INTR_TX_EMPTY_MASK       (1 << 4)      /**< TX empty interrupt mask - 0x010 */
#define SMBUS_IC_INTR_RD_REQ_MASK         (1 << 5)      /**< Read request interrupt mask - 0x020 */
#define SMBUS_IC_INTR_TX_ABRT_MASK        (1 << 6)      /**< TX abort interrupt mask - 0x040 */
#define SMBUS_IC_INTR_RX_DONE_MASK        (1 << 7)      /**< RX done interrupt mask - 0x080 */
#define SMBUS_IC_INTR_ACTIVITY_MASK       (1 << 8)      /**< Activity interrupt mask - 0x100 */
#define SMBUS_IC_INTR_STOP_DET_MASK       (1 << 9)      /**< Stop detection interrupt mask - 0x200 */
#define SMBUS_IC_INTR_START_DET_MASK      (1 << 10)     /**< Start detection interrupt mask - 0x400 */
#define SMBUS_IC_INTR_GEN_CALL_MASK       (1 << 11)     /**< General call interrupt mask - 0x800 */
#define SMBUS_IC_INTR_RESTART_DET_MASK    (1 << 12)     /**< Restart detection interrupt mask - 0x1000 */

/* All interrupts mask for clearing all interrupts */
#define SMBUS_IC_INTR_ALL                 (0xFFFFFFFF)   /**< All interrupts mask */

/* I2C TX Abort Source Mask Constants */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_MASK (1U << 1)  /**< Master disable abort source mask */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_MASK (1U << 0)  /**< 7-bit address NACK abort source mask */
#define SMBUS_IC_TX_ABRT_SOURCE_ABRT_ARB_LOST_MASK (1U << 9)     /**< Arbitration lost abort source mask */

/* SDA Stuck Recovery Abort and Status Masks */
#define SMBUS_IC_TX_ABRT_SDA_STUCK_AT_LOW_MASK (1U << 17) /**< SDA stuck at low abort mask */
#define SMBUS_IC_STATUS_SDA_STUCK_NOT_RECOVERED_MASK (1U << 11) /**< SDA stuck not recovered status mask */

/* Utility Macros */
#define BIT(nr)                 (1UL << (nr))                    /**< Bit macro */
#define GENMASK(h, l)           (((~0UL) << (l)) & (~0UL >> (31 - (h))))  /**< Generate mask */

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

/* ARP state constants */
#define SMBUS_ARP_STATE_INIT           (0)       /**< ARP initialization state */
#define SMBUS_ARP_STATE_READY          (1)       /**< ARP ready state */

/* ======================================================================== */
/*                    SMBus Master Configuration Macros                      */
/* ======================================================================== */

/* Control Register Bit Position Macros */
#define SMBUS_IC_CON_MASTER_MODE_EN_BIT       (0)   /**< Master mode enable bit position */
#define SMBUS_IC_CON_SLAVE_DISABLE_BIT        (6)   /**< Slave disable bit position */
#define SMBUS_IC_CON_RESTART_EN_BIT           (5)   /**< Restart enable bit position */
#define SMBUS_IC_CON_10BIT_MASTER_ADDR_BIT    (4)   /**< 10-bit master addressing bit position */
#define SMBUS_IC_CON_10BIT_SLAVE_ADDR_BIT     (3)   /**< 10-bit slave addressing bit position */
#define SMBUS_IC_CON_RX_FIFO_HOLD_BIT         (7)   /**< RX FIFO full hold control bit position */
#define SMBUS_IC_CON_STOP_DET_IFADDRESSED_BIT (9)   /**< STOP detection when addressed bit position */
#define SMBUS_IC_CON_ARP_ENABLE_BIT           (14)  /**< ARP enable bit position */
#define SMBUS_IC_CON_QUICK_CMD_BIT            (15)  /**< SMBus quick command enable bit position */

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

/* Enable Status Mask */
#define SMBUS_IC_ENABLE_STATUS_IC_EN          (0x01)      /**< Enable status bit mask */

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
/**
 * @brief SMBus ARP Device Node Structure
 */
typedef struct SmbusArpDeviceNode {
    U8  currentAddress;                    /**< Current assigned address */
    U8  flags;                             /**< Device flags */
    U16 reserved;                          /**< Reserved for alignment */
    SmbusUdid_s udid;                      /**< Device UDID */
    struct SmbusArpDeviceNode *next;       /**< Next device in list */
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
    void                  *hwContext;        /**< Hardware context */
} SmbusArpMaster_s;

/**
 * @brief SMBus ARP notify callback function type
 */
typedef void (*SmbusArpEventCallback)(SmbusArpEvent_e event, const SmbusArpDev_s *dev, void *userData);

/**
 * @brief SMBus ARP Callback Node Structure
 */
typedef struct SmbusArpNotifyCb {
    SmbusArpEventCallback callbacks[6];        /**< Callback function */
    U8                    callbackCount;       /**< Callback count */
    void                  *userData[6];           /**< Callback paramter */
} SmbusArpNotifyCb_s;

typedef struct SmbusMsg {
  U16 addr;
  U16 flags;
  U16 len;
  U8 *buf;
}SmbusMsg_t;
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
    U32                      status;                                /**< Device status */
    S32                      cmdErr;                                /**< Command error code */
    U32                      abortSource;                           /**< Abort source register value */
    SmbusMsg_t		         msgs[SMBUS_MAX_BLOCK_LEN + 1];         /**< SMBus messages parameters */

    U32                      irq;                /**< Interrupt number */
    U32                      channelNum;         /**< Channel number for compatibility */
    U8                       cmdReg;
    U8                       workMode;           /**< Work mode (0=interrupt, 1=polling) */
    U8                       isSmbus;            /**< SMBus mode flag */
    U8                       enabled;            /**< Device enabled status */
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
} SmbusDev_s;

/* SMBus status constants for async operations */
#define SMBUS_STATUS_ACTIVE          0x00000001  ///< Transfer in progress
#define SMBUS_STATUS_READ_IN_PROGRESS 0x00000002  ///< Read operation in progress
#define SMBUS_STATUS_WRITE_IN_PROGRESS 0x00000004  ///< Write operation in progress
#define SMBUS_STATUS_MASK            0x00000007  ///< Status mask

/* SMBus error codes */
#define SMBUS_ERR_TX_ABRT             0x00000001  ///< TX abort error
#define SMBUS_ERR_RX_UNDER            0x00000002  ///< RX underflow error
#define SMBUS_ERR_RX_OVER             0x00000004  ///< RX overflow error
#define SMBUS_ERR_TX_OVER             0x00000008  ///< TX overflow error
#define SMBUS_ERR_ACTIVITY            0x00000010  ///< Activity error

/* Callback function type definitions */
typedef struct SmbusTxDoneEventData {
    U8 *val;
} SmbusTxDoneEventData_s;

typedef struct SmbusRxDoneEventData {
    U8 *val;
    U8 len;
} SmbusRxDoneEventData_s;

typedef struct SmbusErrorEventData {
    U8 *val;
    U8 errorCode;
} SmbusErrorEventData_s;

typedef struct SmbusSlaveReadReqEventData {
    U8 *data;
    U32 *len;
} SmbusSlaveReadReqEventData_s;

typedef struct SmbusSlaveWriteReqEventData {
    U8 *data;
    U32 len;
} SmbusSlaveWriteReqEventData_s;

typedef struct SmbusSlaveStopEventData {
    U8 *val;
} SmbusSlaveStopEventData_s;

typedef struct SmbusPecErrorEventData {
    U8 *val;
} SmbusPecErrorEventData_s;

typedef struct SmbusArpDiscoverEventData {
    U8 *val;
} SmbusArpDiscoverEventData_s;

typedef struct SmbusGeneralCallEventData {
    U8 *val;
} SmbusGeneralCallEventData_s;

typedef union SmbusEventData {
    SmbusTxDoneEventData_s          txDone;
    SmbusRxDoneEventData_s          rxDone;
    SmbusErrorEventData_s           error;
    SmbusSlaveReadReqEventData_s    slaveReadReq;
    SmbusSlaveWriteReqEventData_s   slaveWriteReq;
    SmbusSlaveStopEventData_s       slaveStop;
    SmbusPecErrorEventData_s        pecError;
    SmbusArpDiscoverEventData_s     arpDiscover;
    SmbusGeneralCallEventData_s     generalCall;
} SmbusEventData_u;

typedef struct SmbusCallbackInfo {
    SmbusCallback_t  cb;            /**< common evnet callbck */
    SmbusEventData_u *userData;     /**< User data */
} SmbusCallbackInfo_s;


/**
 * @brief SMBus Driver Data Structure
 */
typedef struct SmbusDrvData {
    U8                     devId;           /**< device number include device list */
    U8                     retryCount;      /**< Current retry count */
    U8                     enablePec;       /**< PEC enable */
    U8                     reserved1;       /**< Reserved for alignment */
    U8                     txComplete;      /**< tx COMPLETE detect flag */
    U8                     slaveTransferActive;
    U8                     *rxBuffer;
    U8                     rxLength;
    U8                     rxComplete;
    U8                     errorCode;
    U8                     lastError;
    SbrI2cSmbusCfg_s       sbrCfg;          /**< SBR configuration */
    SmbusArpConfig_s       arpConfig;       /**< ARP configuration */
    bool                   arpEnabled;      /**< ARP enabled flag */
    bool                   arpInitialized;  /**< ARP initialized flag */
    SmbusArpState_e        arpState;        /**< ARP state */
    SmbusArpNotifyCb_s     arpNotifyEvent;  /**< ARP  notify event*/
    SmbusArpFailHandler_t  arpFailHandler;  /**< ARP failure handler */
    void                   *arpFailParam;   /**< ARP failure callback parameter */
    void                   *arpContext;     /**< ARP context */
    U32                    slaveValidRxLen; /**< Slave valid RX length */
    U32                    slaveTxIndex;   /**< Slave TX buffer index */
    SmbusDev_s             pSmbusDev;      /**< SMBus device context */
    SmbusArpMaster_s       arpMaster;       /**< ARP master context */
    SmbusUdid_s            udid;

    U16                    errorCount;
    U16                    rxBufferSize;
    SmbusCallbackInfo_s    callback;         /*common interface */
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
 * @brief Handle master timeout recovery based on flow diagram
 * @param pDrvData Pointer to driver data structure
 * @param timeoutType Type of timeout that occurred
 */
void smbusHandleMasterTimeoutRecovery(SmbusDrvData_s *pDrvData, SmbusTimeoutType_e timeoutType);

/**
 * @brief Handle slave timeout recovery based on flow diagram
 * @param pDrvData Pointer to driver data structure
 * @param timeoutType Type of timeout that occurred
 */
void smbusHandleSlaveTimeoutRecovery(SmbusDrvData_s *pDrvData, SmbusTimeoutType_e timeoutType);

/* ======================================================================== */
/*                    Core Protocol - PEC and Utility Functions               */
/* ======================================================================== */

/**
 * @brief Calculate CRC-8 for one byte of data
 * @details Calculates CRC-8 checksum for a single byte using the SMBus
 *          polynomial (0x07). This is a low-level function used by higher
 *          level PEC calculation functions.
 * @param[in] crc Current CRC value
 * @param[in] data Data byte to include in CRC calculation
 * @return Updated CRC value
 *
 * @note Uses SMBus CRC-8 polynomial (0x07)
 * @note Processes 8 bits with polynomial division
 * [CORE] Core protocol layer - CRC calculation utilities
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
 * @brief Calculate SMBus PEC (Packet Error Code) for buffer
 * @details Calculates SMBus PEC (CRC-8) for a buffer of data. PEC is used
 *          to verify data integrity in SMBus communications.
 * @param[in] pec Initial PEC value (typically 0)
 * @param[in] buf Pointer to data buffer
 * @param[in] len Length of data buffer
 * @return Calculated PEC value
 *
 * @note Uses CRC-8 with polynomial 0x07
 * @note Processes entire buffer sequentially
 * [CORE] Core protocol layer - PEC calculation utilities
 */
U8 smbusCalcPEC(U8 pec, const U8 *buf, U32 len);

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
 * [CORE] Core protocol layer - address conversion utilities
 */
const U8 i2cAddrConvert(U8 addrIn, bool isWrite);

/**
 * @brief Construct PEC for complete SMBus packet
 * @details Constructs PEC for a complete SMBus packet including device
 *          address, read/write bit, and data payload. This function
 *          calculates the PEC that would be transmitted at the end of
 *          an SMBus transaction.
 * @param[in] addr7bitIn 7-bit SMBus device address
 * @param[in] isWrite True for write operation, false for read operation
 * @param[in] pData Pointer to data payload buffer
 * @param[in] count Number of data bytes in payload
 * @return Calculated PEC value for the complete packet
 *
 * @note Includes device address and R/W bit in PEC calculation
 * @note Validates input parameters for safety
 * @note Returns 0xFF for invalid parameters
 */
U8 smbusPecPktConstruct(U8 addr7bitIn, bool isWrite, U8 *pData, U32 count);

/* ======================================================================== */
/*                    ISR Helper Function Declarations                           */
/* ======================================================================== */

/**
 * @brief Read and clear interrupt bits (modular function)
 * @details Reads interrupt status register and clears the interrupts
 * @param[in] regBase Register base address
 * @return Raw interrupt status value
 *
 * @note This function combines read and clear operations
 * [CORE] Core protocol layer - interrupt management
 */
U32 smbusReadClearIntrBits(volatile SmbusRegMap_s *regBase);
U32 smbusReadClearIntrBitsMasked(SmbusDrvData_s *pDrvData, U32 mask);

/**
 * @brief Master read data from RX FIFO
 * @details Reads data from RX FIFO and stores in driver data structure
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @return void
 *
 * [CORE] Core protocol layer - master data handling
 */
void smbusMasterReadData(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase);

/**
 * @brief Master transfer data to TX FIFO
 * @details Transfers data from TX buffer to TX FIFO
 * @param[in] pDrvData Pointer to driver data structure
 * @param[in] regBase Register base address
 * @return void
 *
 * [CORE] Core protocol layer - master data handling
 */
void smbusMasterTransferData(SmbusDrvData_s *pDrvData, volatile SmbusRegMap_s *regBase);

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
void smbusTriggerSlaveEvent(SmbusDrvData_s *pDrvData, U32 eventType, void *data, U32 len);


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
                                         U32 smbusIntrStat);

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
void smbusClearDevs(SmbusArpMaster_s *master);
/**
 * @brief Compare two UDIDs for equality
 * @param[in] udid1 First UDID
 * @param[in] udid2 Second UDID
 * @return 1 if equal, 0 if different
 */
Bool smbusUdidCompare(const SmbusUdid_s *udid1, const SmbusUdid_s *udid2);

/**
 * @brief Check if address is already used in ARP master device list
 * @param[in] master Pointer to ARP master structure
 * @param[in] address Address to check
 * @return TRUE if address is used, FALSE otherwise
 */
Bool smbusArpIsAddrUsed(SmbusArpMaster_s *master, U8 address);

/**
 * @brief Install ARP device in master device list
 * @details Adds a new device to the ARP master's device list with the specified
 *          UDID and address. This function performs parameter validation and
 *          conflict checks before adding the device.
 * @param[in,out] master Pointer to the ARP master structure
 * @param[in] udid Pointer to the device UDID structure
 * @param[in] address Address to assign to the device
 * @return 0 on success, negative error code on failure:
 *         -1: Invalid parameters
 *         -2: Address out of range or reserved
 *         -3: Device already exists
 *         -4: Address conflict with existing device
 *         -ENOMEM: Memory allocation failure
 *
 * @note Validates address range and pool boundaries
 * @note Checks for address conflicts with existing devices
 * @note Updates master device count and next available address
 * @warning This function is not thread-safe
 * @warning Caller must ensure master pointer is valid
 *
 */
S32 ArpDevInstall(SmbusArpMaster_s *master, const SmbusUdid_s *udid, U8 address);

/**
 * @brief Get SMBus register base address
 * @param devId Device identifier
 * @param pCtrlReg Pointer to register map pointer
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
S32 getSmbusReg(DevList_e devId, SmbusRegMap_s **pCtrlReg);

S32 smbusDrvLockAndCheck(DevList_e devId);

#ifdef __cplusplus
}
#endif

#endif /* __SMBUS_DW_H__ */
