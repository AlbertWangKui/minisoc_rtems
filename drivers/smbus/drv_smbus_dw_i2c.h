/**
 * Copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_smbus_dw_i2c.h
 * @author wangkui (wangkui@starsmicrosystem.com)
 * @date 2025/10/20
 * @brief SMBus protocol core Logec header file
 *
 * @par ChangeLog:
 * Date         Author          Description
 * 2025/10/20   wangkui         Initial version
 * 2025/12/01   wangkui         refactor and simplify the code as for core function
 * 2025/12/11   wangkui         correct the issue in AI review
 * 2025/12/15   wangkui         replace slave with target naming
 */

#ifndef __DRV_SMBUS_DW_I2C_H__
#define __DRV_SMBUS_DW_I2C_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "bsp_config.h"
#include "bsp_drv_id.h"
#include "bsp_api.h"
#include "bsp_device.h"

#include "log_msg.h"
#include "udelay.h"
#include "osp_interrupt.h"
#include "drv_smbus_api.h"

/* Message direction flag */
#define SMBUS_M_RD              0x0001  /**< Read data, from target to master*/

/* Special message flags for SMBus Block operations */
#define SMBUS_M_RECV_LEN        0x0400  /**< Length will be first received byte */
                                        /* Used for Block Read to indicate the first
                                         * byte received is the block length */

/* PEC (Packet Error Code) support */
#define SMBUS_M_PEC             0x0008  /**< Enable PEC for this message */
                                        /* SMBus Packet Error Checking */

/* Other common I2C flags you may need */
#define SMBUS_M_TEN             0x0010  /**< 10-bit address mode */
#define SMBUS_M_IGNORE_NAK      0x1000  /**< Treat NACK as ACK */
#define SMBUS_M_REV_DIR_ADDR    0x2000  /**< Toggle R/W bit */
#define SMBUS_M_NOSTART         0x4000  /**< No (re)start before this message */
#define SMBUS_M_STOP            0x8000  /**< Send STOP after this message */

/* Local constant definitions to avoid circular include */
#define SMBUS_TX_READY_TIMEOUT_US     (10000U)
#define SMBUS_RX_READY_TIMEOUT_US     (10000U)
#define SMBUS_TRANSACTION_TIMEOUT_US  (100000U)
#define SMBUS_ARP_ADDR                (0x61)
#define SMBUS_HOST_NOTIFY_ADDR        (0x08)
#define SMBUS_TARGET_DEFAULT_ADDR     (0x61)  /**< Default SMBus target address */
#define SMBUS_ARP_ABORT_MASK          (0x09)
#define SMBUS_MODE_TARGET             (0)
#define SMBUS_MODE_MASTER             (1)
#define SMBUS_DEFAULT_TIMEOUT_MS      (5000)
#define SMBUS_IC_ENABLE_ENABLE_MASK   (1 << 0)
#define SMBUS_IC_ENABLE_ABORT_MASK    (1 << 1)
#define SMBUS_IC_ENABLE_RECOVERY_MASK (1 << 3)  /* 对应 sdaStuckRecoveryEnable */

#define SMBUS_IC_STATUS_ACTIVITY_MASK (1 << 0)
#define SMBUS_INTR_TX_ABRT_MASK       (1 << 6)

/* SMBus speed range definitions */
#define SMBUS_SS_MIN_SPEED             (8000)    /**< Min frequency during standard speed */
#define SMBUS_SS_MAX_SPEED             (100000)  /**< Max frequency during standard speed */
#define SMBUS_FS_MIN_SPEED             (100000)  /**< Min frequency during fast speed */
#define SMBUS_FS_MAX_SPEED             (1000000) /**< Max frequency during fast speed */
#define SMBUS_HS_MIN_SPEED             (1000000) /**< Min frequency during high speed */
#define SMBUS_HS_MAX_SPEED             (3400000) /**< Max frequency during high speed */
/* Mode switch timing constants */
#define SMBUS_MODE_SWITCH_STD_SPEED_DELAY_US     (100U)     /**< 100us delay for standard speed */
#define SMBUS_MODE_SWITCH_FAST_SPEED_DELAY_US    (25U)      /**< 25us delay for fast speed */
#define SMBUS_MODE_SWITCH_HIGH_SPEED_DELAY_US    (10U)      /**< 10us delay for high speed */
#define SMBUS_MODE_SWITCH_STD_SPEED_TIMEOUT_CNT  (10000U)   /**< Timeout count for standard speed (1s) */
#define SMBUS_MODE_SWITCH_FAST_SPEED_TIMEOUT_CNT (40000U)   /**< Timeout count for fast speed (1s) */
#define SMBUS_MODE_SWITCH_HIGH_SPEED_TIMEOUT_CNT (100000U)  /**< Timeout count for high speed (1s) */

/* Default spike suppression limits for different speed modes */
#define SMBUS_SS_DEFAULT_SPKLEN        (11)  /**< Default spike suppression limit during standard speed */
#define SMBUS_FS_DEFAULT_SPKLEN        (2)   /**< Default spike suppression limit during fast speed */
#define SMBUS_HS_DEFAULT_SPKLEN        (1)   /**< Default spike suppression limit during high speed */

/* IC_DATA_CMD Register Bit Definitions */
#define SMBUS_IC_DATA_CMD_READ_CMD    (0x1 << 8)  /**< Bit 8: Read command (1=read, 0=write) */
#define SMBUS_IC_DATA_CMD_STOP        (0x1 << 9)  /**< Bit 9: Stop condition */
#define SMBUS_IC_DATA_CMD_RESTART     (0x1 << 10) /**< Bit 10: Restart condition */

/* IC_CON register bit definitions */
#define SMBUS_IC_CON_RESTART_EN     (1U << 5)   /* Restart enable */
#define SMBUS_IC_CON_MASTER         (1U << 0)   /* Master mode */

/* Macro to initialize all SMBus timing counters to zero */
#define SMBUS_INIT_TIMING_COUNTERS(dev) do { \
    (dev)->ssHcnt = 0; \
    (dev)->ssLcnt = 0; \
    (dev)->fsHcnt = 0; \
    (dev)->fsLcnt = 0; \
    (dev)->hsHcnt = 0; \
    (dev)->hsLcnt = 0; \
    (dev)->fpHcnt = 0; \
    (dev)->fpLcnt = 0; \
    (dev)->fsSpklen = 0; \
    (dev)->hsSpklen = 0; \
} while(0)

/* Forward declarations */
typedef struct SmbusDev SmbusDev_s;
typedef struct SmbusMsg SmbusMsg_s;
typedef struct SmbusDrvData SmbusDrvData_s;


#ifdef __cplusplus
extern "C" {
#endif

#define SMBUS_MAX_DEVICE_NUM    (6)   /**< 最大支持的 SMBus 设备数量 */
/* ======================================================================== */
/*                      Register Bitfield Unions                            */
/* ======================================================================== */

/**
 * @brief IC_CON - Control Register Bitfield
 */
typedef union SmbusIcConReg {
    struct {
        /** [0] Master mode enable */
        U32 masterMode             : 1;
        /** [2:1] Speed control: 1=Standard (100k), 2=Fast/Fast+ (<=1M), 3=High (3.4M) */
        U32 speed                  : 2;
        /** [3] 10-bit target addressing mode enable */
        U32 ic10bitaddrtarget       : 1;
        /** [4] 10-bit master addressing mode enable */
        U32 ic10bitaddrMaster      : 1;
        /** [5] RESTART condition enable in master mode */
        U32 icRestartEn            : 1;
        /** [6] target mode disable */
        U32 ictargetDisable         : 1;
        /** [7] STOP detection interrupt only when addressed in target mode */
        U32 stopDetIfaddressed     : 1;
        /** [8] TX_EMPTY interrupt control */
        U32 txEmptyCtrl            : 1;
        /** [9] RX FIFO full hold bus control */
        U32 rxFifoFullHldCtrl      : 1;
        /** [10] STOP detection interrupt only when master active */
        U32 stopDetIfMasterActive  : 1;
        /** [11] Bus clear feature control */
        U32 busClearCtrl           : 1;
        /** [15:12] Reserved */
        U32 reserved_12_15         : 4;
        /** [16] Optional SAR (target Address Register) control */
        U32 optionalSarCtrl        : 1;
        /** [17] SMBus target Quick command enable */
        U32 smbusSlvgetQuickEn      : 1;
        /** [18] SMBus Address Resolution Protocol (ARP) enable */
        U32 smbusArpEn             : 1;
        /** [19] SMBus Persistent target address enable (for default address) */
        U32 smbusPersistentSlvAddrEn: 1;
        /** [20] SMBus Persistent target address 2 (IC_SAR2) enable */
        U32 smbusPersistentSlvAddr2En: 1;
        /** [21] SMBus Persistent target address 3 (IC_SAR3) enable */
        U32 smbusPersistentSlvAddr3En: 1;
        /** [22] SMBus Persistent target address 4 (IC_SAR4) enable */
        U32 smbusPersistentSlvAddr4En: 1;
        /** [23] SMBus ARP enable for target Address 2 (IC_SAR2) */
        U32 icSar2SmbusArpEn       : 1;
        /** [24] SMBus ARP enable for target Address 3 (IC_SAR3) */
        U32 icSar3SmbusArpEn       : 1;
        /** [25] SMBus ARP enable for target Address 4 (IC_SAR4) */
        U32 icSar4SmbusArpEn       : 1;
        /** [31:26] Reserved */
        U32 reserved_26_31         : 6;
    } fields;
    U32 value;
} SmbusIcConReg_u;

/**
 * @brief IC_TAR - Target Address Register Bitfield
 */
typedef union SmbusIcTarReg {
    struct {
        U32 icTar                : 10;  /**< [9:0] Target target address */
        U32 gcOrStart            : 1;   /**< [10] General call or START byte */
        U32 special              : 1;   /**< [11] Special command enable */
        U32 ic10bitaddrMaster    : 1;   /**< [12] Master 10-bit addressing */
        U32 deviceId             : 1;   /**< [13] Device-ID transfer */
        U32 reserved1            : 2;   /**< [15:14] Reserved */
        U32 smbusQuickCmd        : 1;   /**< [16] SMBus quick command */
        U32 reserved2            : 15;  /**< [31:17] Reserved */
    } fields;
    U32 value;
} SmbusIcTarReg_u;

/**
 * @brief IC_SAR - target Address Register Bitfield
 * Based on SMBus specification, IC_SAR register only contains the target address
 * SAR_ENABLE is located in IC_ENABLE register bit 19, not in IC_SAR register
 */
typedef union SmbusIcSarReg {
    struct {
        U32 icSar                : 10;  /**< [9:0] target address */
        U32 reserved1            : 22;  /**< [31:10] Reserved */
    } fields;
    U32 value;
} SmbusIcSarReg_u;

/**
 * @brief IC_DATA_CMD - Data Buffer and Command Register Bitfield
 */
typedef union SmbusIcDataCmdReg {
    struct {
        U32 dat                  : 8;   /**< [7:0] 数据 */
        U32 cmd                  : 1;   /**< [8] 命令 (1=read, 0=write) */
        U32 stop                 : 1;   /**< [9] STOP 位 */
        U32 restart              : 1;   /**< [10] RESTART 位 */
        U32 rsvd                 : 21;  /**< [31:11] 保留 */
    } fields;
    U32 value;
} SmbusIcDataCmdReg_u;

/**
 * @brief IC_INTR_STAT - Interrupt Status Register Bitfield
 */
typedef union SmbusIcIntrStatReg {
    struct {
        U32 rxUnder             : 1;   /**< [0] RX under - Receiver Under */
        U32 rxOver              : 1;   /**< [1] RX over - Receiver Over */
        U32 rxFull              : 1;   /**< [2] RX full - Receive Full */
        U32 txOver              : 1;   /**< [3] TX over - Interrupt Transmit Over */
        U32 txEmpty             : 1;   /**< [4] TX empty - Interrupt Transmit Empty */
        U32 rdReq               : 1;   /**< [5] Read request - Interrupt Read Request */
        U32 txAbrt              : 1;   /**< [6] TX abort - Interrupt TX Abort */
        U32 rxDone              : 1;   /**< [7] RX done - Interrupt RX Done */
        U32 activity            : 1;   /**< [8] Activity - Interrupt R_activity */
        U32 stopDet             : 1;   /**< [9] Stop detection - Interrupt Stop Detect */
        U32 startDet            : 1;   /**< [10] Start detection - Interrupt Start Detect */
        U32 genCall             : 1;   /**< [11] General call - Interrupt General Call */
        U32 restartDet          : 1;   /**< [12] Restart detection - Interrupt ReStart Detect */
        U32 mstOnHold           : 1;   /**< [13] Master on hold - Interrupt Master On Hold */
        U32 sclStuckAtLow       : 1;   /**< [14] SCL stuck at low - Interrupt R_SCL_STUCK_AT_LOW */
        U32 WrReq               : 1;    /**< [15] WR request */
        U32 targetrAddr1Tag      : 1;    /**< [16] target Address 1 Tag */
        U32 targetrAdd21Tag      : 1;    /**< [17] target Address 2 Tag */
        U32 targetrAddr3Tag      : 1;    /**< [18] target Address 3 Tag */
        U32 targetrAddr4Tag      : 1;    /**< [19] target Address 4 Tag */
        U32 reserved            : 12;   /**< [31:20] Reserved bits */
    } fields;
    U32 value;
} SmbusIcIntrStatReg_u;

typedef union IcIntrMask {
  struct {
    uint32_t rxUnder             : 1;  /**< [0] M_RX_UNDER */
    uint32_t rxOver              : 1;  /**< [1] M_RX_OVER */
    uint32_t rxFull              : 1;  /**< [2] M_RX_FULL */
    uint32_t txOver              : 1;  /**< [3] M_TX_OVER */
    uint32_t txEmpty             : 1;  /**< [4] M_TX_EMPTY */
    uint32_t rdReq               : 1;  /**< [5] M_RD_REQ */
    uint32_t txAbrt              : 1;  /**< [6] M_TX_ABRT */
    uint32_t rxDone              : 1;  /**< [7] M_RX_DONE */
    uint32_t activity            : 1;  /**< [8] M_ACTIVITY */
    uint32_t stopDet             : 1;  /**< [9] M_STOP_DET */
    uint32_t startDet            : 1;  /**< [10] M_START_DET */
    uint32_t genCall             : 1;  /**< [11] M_GEN_CALL */
    uint32_t restartDet          : 1;  /**< [12] M_RESTART_DET */
    uint32_t masterOnHold        : 1;  /**< [13] M_MASTER_ON_HOLD */
    uint32_t sclStuckAtLow       : 1;  /**< [14] M_SCL_STUCK_AT_LOW */
    uint32_t wrReq               : 1;  /**< [15] M_WR_REQ */
    uint32_t slvAddr1Tag         : 1;  /**< [16] M_SLV_ADDR1_TAG */
    uint32_t slvAddr2Tag         : 1;  /**< [17] M_SLV_ADDR2_TAG */
    uint32_t slvAddr3Tag         : 1;  /**< [18] M_SLV_ADDR3_TAG */
    uint32_t slvAddr4Tag         : 1;  /**< [19] M_SLV_ADDR4_TAG */
    uint32_t reserved            : 12; /**< [31:20] RSVD_IC_INTR_STAT */
  } fields;
  uint32_t value;
} SmbusIcIntrMask_u;

/**
 * @brief IC_RAW_INTR_STAT - Raw Interrupt Status Register Bitfield
 * @details This register provides the raw interrupt status before masking.
 *          It has the same bitfield structure as IC_INTR_STAT but shows
 *          the unmasked interrupt status directly from the hardware.
 */
typedef union SmbusIcRawIntrStatReg {
    struct {
        U32 rxUnder             : 1;   /**< [0] RX under - Receiver Under */
        U32 rxOver              : 1;   /**< [1] RX over - Receiver Over */
        U32 rxFull              : 1;   /**< [2] RX full - Receive Full */
        U32 txOver              : 1;   /**< [3] TX over - Interrupt Transmit Over */
        U32 txEmpty             : 1;   /**< [4] TX empty - Interrupt Transmit Empty */
        U32 rdReq               : 1;   /**< [5] Read request - Interrupt Read Request */
        U32 txAbrt              : 1;   /**< [6] TX abort - Interrupt TX Abort */
        U32 rxDone              : 1;   /**< [7] RX done - Interrupt RX Done */
        U32 activity            : 1;   /**< [8] Activity - Interrupt R_activity */
        U32 stopDet             : 1;   /**< [9] Stop detection - Interrupt Stop Detect */
        U32 startDet            : 1;   /**< [10] Start detection - Interrupt Start Detect */
        U32 genCall             : 1;   /**< [11] General call - Interrupt General Call */
        U32 restartDet          : 1;   /**< [12] Restart detection - Interrupt ReStart Detect */
        U32 mstOnHold           : 1;   /**< [13] Master on hold - Interrupt Master On Hold */
        U32 sclStuckAtLow       : 1;   /**< [14] SCL stuck at low - Interrupt R_SCL_STUCK_AT_LOW */
        U32 reserved            : 17;  /**< [31:15] Reserved bits */
    } fields;
    U32 value;
} SmbusIcRawIntrStatReg_u;

/**
 * @brief IC_ENABLE - Enable Register Bitfield
 */
typedef union SmbusIcEnableReg {
    struct {
        U32 enable               : 1;   /**< [0] I2C/SMBus controller enable */
        U32 abort                : 1;   /**< [1] Transfer abort */
        U32 txCmdBlock           : 1;   /**< [2] TX command block */
        U32 sdaStuckRecoveryEnable : 1; /**< [3] SDA stuck recovery enable */
        U32 reserved1            : 12;  /**< [15:4] Reserved */
        U32 smbusClkReset        : 1;   /**< [16] SMBus clock line reset */
        U32 smbusSuspendEn       : 1;   /**< [17] SMBus suspend enable */
        U32 smbusAlertEn         : 1;   /**< [18] SMBus alert enable */
        U32 icSarEn              : 1;   /**< [19] Controls whether the DW_apb_i2c SAR is enabled */
        U32 icSar2En             : 1;   /**< [20] Controls whether the DW_apb_i2c SAR2 is enabled */
        U32 icSar3En             : 1;   /**< [21] Controls whether the DW_apb_i2c SAR3 is enabled */
        U32 icSar4En             : 1;   /**< [22] Controls whether the DW_apb_i2c SAR4 is enabled */
        U32 reserved2            : 9;   /**< [31:23] Reserved */
    } fields;
    U32 value;
} SmbusIcEnableReg_u;

/**
 * @brief IC_STATUS - Status Register Bitfield
 */
typedef union SmbusIcStatusReg {
    struct {
        U32 activity             : 1;    /**< [0] ACTIVITY */
        U32 tfnf                 : 1;    /**< [1] TFNF - TX FIFO not full */
        U32 tfe                  : 1;    /**< [2] TFE - TX FIFO empty */
        U32 rfne                 : 1;    /**< [3] RFNE - RX FIFO not empty */
        U32 rff                  : 1;    /**< [4] RFF - RX FIFO full */
        U32 mstActivity          : 1;    /**< [5] MST_ACTIVITY */
        U32 slvActivity          : 1;    /**< [6] SLV_ACTIVITY */
        U32 mstHoldTxFifoEmpty   : 1;    /**< [7] MST_HOLD_TX_FIFO_EMPTY */
        U32 mstHoldRxFifoFull    : 1;    /**< [8] MST_HOLD_RX_FIFO_FULL */
        U32 slvHoldTxFifoEmpty   : 1;    /**< [9] SLV_HOLD_TX_FIFO_EMPTY */
        U32 slvHoldRxFifoFull    : 1;    /**< [10] SLV_HOLD_RX_FIFO_FULL */
        U32 sdaStuckNotRecovered : 1;    /**< [11] SDA_STUCK_NOT_RECOVERED */
        U32 slvIsoSarDataClkStretch : 1; /**< [12] SLV_ISO_SAR_DATA_CLK_STRETCH */
        U32 reserved1            : 3;    /**< [15:13] Reserved */
        U32 smbusQuickCmdBit     : 1;    /**< [16] SMBUS_QUICK_CMD_BIT */
        U32 smbusSlvgetAddrValid  : 1;   /**< [17] SMBUS_SLVGET_ADDR_VALID */
        U32 smbusSlvgetAddrResolved : 1; /**< [18] SMBUS_SLVGET_ADDR_RESOLVED */
        U32 smbusAlertStatus     : 1;    /**< [19] SMBUS_ALERT_STATUS */
        U32 smbusSuspendStatus   : 1;    /**< [20] SMBUS_SUSPEND_STATUS */
        U32 reserved2            : 11;   /**< [31:21] Reserved */
    } fields;
    U32 value;
} SmbusIcStatusReg_u;

/**
 * @brief IC_TX_ABRT_SOURCE - TX Abort Source Register Bitfield
 */
typedef union SmbusIcTxAbrtSourceReg {
    struct {
        U32 abrt7bAddrNoack      : 1;   /**< [0] 7-bit addr no ACK */
        U32 abrt10addr1Noack     : 1;   /**< [1] 10-bit addr byte1 no ACK */
        U32 abrt10addr2Noack     : 1;   /**< [2] 10-bit addr byte2 no ACK */
        U32 abrtTxdataNoack      : 1;   /**< [3] TX data no ACK */
        U32 abrtGcallNoack       : 1;   /**< [4] General call no ACK */
        U32 abrtGcallRead        : 1;   /**< [5] General call read */
        U32 abrtHsAckdet         : 1;   /**< [6] HS mode ACK detected */
        U32 abrtSbyteAckdet      : 1;   /**< [7] Start byte ACK detected */
        U32 abrtHsNorstrt        : 1;   /**< [8] HS mode no restart */
        U32 abrtSbyteNorstrt     : 1;   /**< [9] Start byte no restart */
        U32 abrt10bRdNorstrt     : 1;   /**< [10] 10-bit read no restart */
        U32 abrtMasterDis        : 1;   /**< [11] Master operation disabled */
        U32 arbLost              : 1;   /**< [12] Arbitration lost */
        U32 abrtSlvflushTxfifo   : 1;   /**< [13] target flush TX FIFO */
        U32 abrtSlvArblost       : 1;   /**< [14] target arbitration lost */
        U32 abrtSlvrdIntx        : 1;   /**< [15] target read in TX mode */
        U32 abrtUserAbrt         : 1;   /**< [16] User initiated abort */
        U32 abrtSdaStuckAtLow    : 1;   /**< [17] SDA stuck at low */
        U32 reserved             : 14;  /**< [31:18] Reserved */
    } fields;
    U32 value;
} SmbusIcTxAbrtSourceReg_u;

typedef union IcSmbusIntrMaskReg {
    struct {
        U32 slvClockExtndTimeout : 1; /**< [0] M_SLV_CLOCK_EXTND_TIMEOUT */
        U32 mstClockExtndTimeout : 1; /**< [1] M_MST_CLOCK_EXTND_TIMEOUT */
        U32 quickCmdDet          : 1; /**< [2] M_QUICK_CMD_DET */
        U32 hostNotifyMstDet     : 1; /**< [3] M_HOST_NOTIFY_MST_DET */
        U32 arpPrepareCmdDet     : 1; /**< [4] M_ARP_PREPARE_CMD_DET */
        U32 arpRstCmdDet         : 1; /**< [5] M_ARP_RST_CMD_DET */
        U32 arpGetUdidCmdDet     : 1; /**< [6] M_ARP_GET_UDID_CMD_DET */
        U32 arpAssignAddrCmdDet  : 1; /**< [7] M_ARP_ASSGN_ADDR_CMD_DET */
        U32 slvRxPecNack         : 1; /**< [8] M_SLV_RX_PEC_NACK */
        U32 smbusSuspendDet      : 1; /**< [9] M_SMBUS_SUSPEND_DET */
        U32 smbusAlertDet        : 1; /**< [10] M_SMBUS_ALERT_DET */
        U32 rsvdIcSmbusIntrMask  : 21;/**< [31:11] RSVD_IC_SMBUS_INTR_MASK */
    } fields;
    U32 value;
} IcSmbusIntrMask_u;

/**
 * @brief IC_SMBUS_INTR_STAT - SMBus Interrupt Status Register Bitfield
 */
typedef union SmbusIntrStatReg {
    struct {
        U32 slvClockExtndTimeout      : 1; /**< [0] R_SLV_CLOCK_EXTND_TIMEOUT */
        U32 mstClockExtndTimeout      : 1; /**< [1] R_MST_CLOCK_EXTND_TIMEOUT */
        U32 quickCmdDet               : 1; /**< [2] R_QUICK_CMD_DET */
        U32 hostNotifyMstDet          : 1; /**< [3] R_HOST_NOTIFY_MST_DET */
        U32 arpPrepare                : 1; /**< [4] R_ARP_PREPARE_CMD_DET */
        U32 arpRst                    : 1; /**< [5] R_ARP_RST_CMD_DET */
        U32 arpGetUdid                : 1; /**< [6] R_ARP_GET_UDID_CMD_DET */
        U32 arpAssignAddr             : 1; /**< [7] R_ARP_ASSIGN_ADDR_CMD_DET */
        U32 slvRxPecNack              : 1; /**< [8] R_SLV_RX_PEC_NACK */
        U32 smbusAlert                : 1; /**< [9] R_SMBUS_SUSPEND_DET  */
        U32 smbusSuspend              : 1; /**< [10] R_SMBUS_ALERT_DET*/
        U32 rsvoIcSmbusIntrStat       : 21; /**< [31:11] RSVO_IC_SMBUS_INTR_STAT */
    } fields;
    U32 value;
} SmbusIntrStatReg_u;

/**
 * @brief IC_SMBUS_UDID_WORD - SMBus UDID Word Register Bitfield
 */
typedef union SmbusUdidWordReg {
    struct {
        U32 udidData             : 32;  /**< [31:0] UDID 4-byte data */
    } fields;
    U32 value;
} SmbusUdidWordReg_u;

/* ======================================================================== */
/*                      Complete Register Map Structure                     */
/* ======================================================================== */

/**
 * @brief Complete SMBus/I2C Controller Register Map
 * @note Address offsets are implicit based on struct member order
 */
typedef struct SmbusRegMap {
    SmbusIcConReg_u              icCon;                 /**< 0x00: Control register */
    SmbusIcTarReg_u              icTar;                 /**< 0x04: Target address register */
    SmbusIcSarReg_u              icSar;                 /**< 0x08: target address register */
    U32                          reserved1;             /**< 0x0C: Reserved */
    SmbusIcDataCmdReg_u          icDataCmd;             /**< 0x10: Data buffer and command */
    U32                          icSsSclHcnt;           /**< 0x14: Std speed SCL high count */
    U32                          icSsSclLcnt;           /**< 0x18: Std speed SCL low count */
    U32                          icFsSclHcnt;           /**< 0x1C: Fast speed SCL high count */
    U32                          icFsSclLcnt;           /**< 0x20: Fast speed SCL low count */
    U32                          icHsSclHcnt;           /**< 0x24: High speed SCL high count */
    U32                          icHsSclLcnt;           /**< 0x28: High speed SCL low count */
    SmbusIcIntrStatReg_u         icIntrStat;            /**< 0x2C: Interrupt status */
    SmbusIcIntrMask_u            icIntrMask;            /**< 0x30: Interrupt mask */
    SmbusIcRawIntrStatReg_u      icRawIntrStat;         /**< 0x34: Raw interrupt status */
    U32                          icRxTl;                /**< 0x38: RX FIFO threshold */
    U32                          icTxTl;                /**< 0x3C: TX FIFO threshold */
    U32                          icClrIntr;             /**< 0x40: Clear combined interrupt */
    U32                          icClrRxUnder;          /**< 0x44: Clear RX underflow */
    U32                          icClrRxOver;           /**< 0x48: Clear RX overflow */
    U32                          icClrTxOver;           /**< 0x4C: Clear TX overflow */
    U32                          icClrRdReq;            /**< 0x50: Clear read request */
    U32                          icClrTxAbrt;           /**< 0x54: Clear TX abort */
    U32                          icClrRxDone;           /**< 0x58: Clear RX done */
    U32                          icClrActivity;         /**< 0x5C: Clear activity */
    U32                          icClrStopDet;          /**< 0x60: Clear STOP detection */
    U32                          icClrStartDet;         /**< 0x64: Clear START detection */
    U32                          icClrGenCall;          /**< 0x68: Clear general call */
    SmbusIcEnableReg_u           icEnable;              /**< 0x6C: Enable register */
    SmbusIcStatusReg_u           icStatus;              /**< 0x70: Status register */
    U32                          icTxflr;               /**< 0x74: TX FIFO level */
    U32                          icRxflr;               /**< 0x78: RX FIFO level */
    U32                          icSdaHold;             /**< 0x7C: SDA hold time */
    SmbusIcTxAbrtSourceReg_u     icTxAbrtSource;        /**< 0x80: TX abort source */
    U32                          icSlvDataNackOnly;     /**< 0x84: Generate target Data NACK */
    U32                          icDmaCr;               /**< 0x88: DMA Control Register */
    U32                          icDmaTdlr;             /**< 0x8C: DMA Transmit Data Level */
    U32                          icDmaRdlr;             /**< 0x90: DMA Receive Data Level */
    U32                          icSdaSetup;            /**< 0x94: I2C SDA Setup Register */

    U32                          icAckGeneralCall;      /**< 0x98: ACK general call */
    U32                          icEnableStatus;        /**< 0x9C: Enable status */
    U32                          icFsSpklen;            /**< 0xA0: Fast speed spike length */
    U32                          icHsSpklen;            /**< 0xA4: High speed spike length */
    U32                          icClrRestartDet;       /**< 0xA8: Clear RESTART detection */
    U32                          icSclStuckTimeout;     /**< 0xAC: SCL stuck at low timeout */
    U32                          icSdaStuckTimeout;     /**< 0xB0: SDA stuck at low timeout */
    U32                          icClrSclStuckDet;      /**< 0xB4: Clear SCL Stuck at Low Detect */
    U32                          icDeviceId;            /**< 0xB8: I2C Device ID */
    U32                          icSmbusClkLowSext;     /**< 0xBC: SMBus slaver Clock Extend Timeout */
    U32                          icSmbusClkLowMext;     /**< 0xC0: SMBus Master Clock Extend Timeout */
    U32                          icSmbusThighMaxIdleCount; /**< 0xC4: SMBus Master THigh MAX Bus-idle count */
    SmbusIntrStatReg_u           icSmbusIntrStat;       /**< 0xC8: SMBus interrupt status */
    IcSmbusIntrMask_u            icSmbusIntrMask;       /**< 0xCC: SMBus interrupt mask */
    SmbusIntrStatReg_u           icSmbusRawIntrStat;    /**< 0xD0: SMBus raw interrupt status */
    U32                          icClrSmbusIntr;        /**< 0xD4: Clear SMBus interrupt */
    U32                          icOptionalSar;         /**< 0xD8: Optional SAR */
    SmbusUdidWordReg_u           icSmbusUdidWord0;      /**< 0xDC: UDID word 0 */
    SmbusUdidWordReg_u           icSmbusUdidWord1;      /**< 0xE0: UDID word 1 */
    SmbusUdidWordReg_u           icSmbusUdidWord2;      /**< 0xE4: UDID word 2 */
    SmbusUdidWordReg_u           icSmbusUdidWord3;      /**< 0xE8: UDID word 3 */
    U32                          reserved4;             /**< 0xEC: Reserved (Still undefined in docs provided) */
    U32                          icRegTimeoutRst;       /**< 0xF0: Register timeout counter reset value */
    U32                          icCompParam1;          /**< 0xF4: Component parameter 1 */
    U32                          icCompVersion;         /**< 0xF8: Component version */
    U32                          icCompType;            /**< 0xFC: Component type */
    SmbusIcSarReg_u              icSar2;                /**< 0x100: slaver address 2 */
    SmbusIcSarReg_u              icSar3;                /**< 0x104: slaver address 3 */
    SmbusIcSarReg_u              icSar4;                /**< 0x108: slaver address 4 */
    U32                          reserved5[4];          /**< 0x10C-0x118: Reserved */
    U32                          icClrWrReq;            /**< 0x11C: Clear WR_REQ interrupt */
    U32                          icClrSlvAddrTag;       /**< 0x120: Clear SLV_ADDR_TAG interrupt */
    SmbusUdidWordReg_u           icSmbus2UdidWord0;     /**< 0x124: SAR2 UDID word 0 */
    SmbusUdidWordReg_u           icSmbus2UdidWord1;     /**< 0x128: SAR2 UDID word 1 */
    SmbusUdidWordReg_u           icSmbus2UdidWord2;     /**< 0x12C: SAR2 UDID word 2 */
    SmbusUdidWordReg_u           icSmbus2UdidWord3;     /**< 0x130: SAR2 UDID word 3 */
    SmbusUdidWordReg_u           icSmbus3UdidWord0;     /**< 0x134: SAR3 UDID word 0 */
    SmbusUdidWordReg_u           icSmbus3UdidWord1;     /**< 0x138: SAR3 UDID word 1 */
    SmbusUdidWordReg_u           icSmbus3UdidWord2;     /**< 0x13C: SAR3 UDID word 2 */
    SmbusUdidWordReg_u           icSmbus3UdidWord3;     /**< 0x140: SAR3 UDID word 3 */
    SmbusUdidWordReg_u           icSmbus4UdidWord0;     /**< 0x144: SAR4 UDID word 0 */
    SmbusUdidWordReg_u           icSmbus4UdidWord1;     /**< 0x148: SAR4 UDID word 1 */
    SmbusUdidWordReg_u           icSmbus4UdidWord2;     /**< 0x14C: SAR4 UDID word 2 */
    SmbusUdidWordReg_u           icSmbus4UdidWord3;     /**< 0x150: SAR4 UDID word 3 */
} SmbusRegMap_s;

/* ======================================================================== */
/*                      HAL Function Pointer Structure                      */
/* ======================================================================== */

/**
 * @brief HAL operations function pointers
 */
typedef struct SmbusHalOps {
    S32 (*checkTxReady)(volatile SmbusRegMap_s *regBase);
    S32 (*checkRxReady)(volatile SmbusRegMap_s *regBase);
    void (*enable)(SmbusDev_s *dev);
    void (*disable)(SmbusDev_s *dev);
    S32 (*modeSwitchCore)(SmbusDev_s *dev, SmbusMode_e targetMode);
    S32 (*i2cTransfer)(SmbusDev_s *dev, SmbusMsg_s *msgs, S32 num);  /**< Generic I2C transfer with message array */
    void (*smbusDwRead)(SmbusDrvData_s *pDrvData, void *buf, U32 len);  ///< block read RX full used
    void (*smbusDwXferMsg)(SmbusDev_s *dev);    ///< block write TX empty used
    void (*configureTarget)(SmbusDev_s *dev);        ///< Configure SMBus device for target mode
} SmbusHalOps_s;

/**
 * @brief Generic I2C transfer function
 * @param[in] dev SMBus device handle
 * @param[in] msgs Array of I2C messages
 * @param[in] num Number of messages in array
 * @return EXIT_SUCCESS on success, negative error code on failure
 */
S32 smbusI2cTransfer(SmbusDev_s *dev, SmbusMsg_s *msgs, S32 num);

/**
 * @brief Get HAL function pointers for external use
 * @return Pointer to HAL operations structure
 */
SmbusHalOps_s* smbusGetHalOps(void);

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
 */
S32 smbusProbeMaster(SmbusDev_s *dev);

/**
 * @brief SMBus target probe function compatible with I2C initialization
 * @details Configures SMBus controller in target mode, calculates timing,
 *          detects FIFO size, and enables interrupts. This function is
 *          compatible with I2C initialization interface and can be called
 *          from SMBus device initialization to enable I2C functionality.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Compatible with I2C's i2c_dw_probe_target interface
 * @note Configures target mode timing and FIFO parameters
 * @note Sets up interrupt handling for target operations
 * @note Allocates target buffer memory
 * @note Enables controller after configuration
 * @warning This function should only be called in target mode
 */
S32 smbusProbeTarget(SmbusDev_s *dev);

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
 */
S32 smbusUnprobeMaster(SmbusDev_s *dev);

/**
 * @brief SMBus target unprobe function for cleanup
 * @details Disables SMBus controller, clears configuration, and removes
 *          interrupt handlers for target mode. This function performs
 *          cleanup operations when shutting down target mode.
 * @param[in] dev Pointer to SMBus device structure
 * @return EXIT_SUCCESS on success, negative error code on failure
 *
 * @note Compatible with I2C's i2c_dw_unprobe_TARGET interface
 * @note Disables controller and clears target configuration
 * @note Removes interrupt handlers and frees target buffers
 * @note Performs graceful shutdown of target operations
 * @warning This function should only be called in target mode
 */
S32 smbusUnprobetarget(SmbusDev_s *dev);

#ifdef __cpluscplus
}
#endif

#endif   ///< DRV_SMBUS_DW_HAL

