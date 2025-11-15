/*
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_qspi_ctrl.h
 * @author yuxy3@starsmicrosystem.com
 * @date 2025/06/04
 * @brief  qspictrl header file
 * @note
 */

#ifndef __DRV_QSPI_CTRL_H__
#define __DRV_QSPI_CTRL_H__

#include "common_defines.h"
#include "osp_mutex.h"
#include "osp_semaphore.h"
#include "osp_common.h"
#include "osp_attr.h"
#include "osp_interrupt.h"
#include "bsp_device.h"
#include "drv_qspi_ctrl_api.h"

#define QSPICTRL_INT_ENABLE
#define QSPICTRL_MAX_CLK (25000000)
#define QSPICTRL_MAX_SEGMENT_LEN (32)
#define QSPICTRL_MAX_TRANSACATION (2)
#define WAIT_FOR_QSPICTRL_TIMEOUT (20)  ///< Unit is ms
#define QSPICTRL_MAX_REG_DATA_SEGNUM  8

#define SYS_QSPICTRL_BASE_ADRS (0xBE605000)
#define SYS_INT_NUM_QSPICTRL       81
#define SYS_INT_PRIORITY_TIMESYNC  127

#define SUPPORT_QSPICTRL_SLAVE_TABLE \
    { \
        { \
            .name = "cpld_0", \
            .slaveId = 0, \
            .slaveCfg = { \
                .cs = 0, \
                .clk = 25000000, \
                .readDummyCnt = 12, \
            }, \
        }, \
    }

typedef struct Qspictrl Qspictrl_s;
typedef struct QspictrlSlave QspictrlSlave_s;
typedef struct QspictrlSlaveMgr QspictrlSlaveMgr_s;

typedef enum QspictrlCs {
    QSPICTRL_CS0 = 0,
    QSPICTRL_CS1,
    QSPICTRL_CS2,
    QSPICTRL_CS3,
    QSPICTRL_CS_MAX
} QspictrlCs_u;

typedef struct QspictrlSlaveConfig {
    QspictrlCs_u cs; ///< cs0~cs3
    U32 clk;
    U32 readDummyCnt;
} QspictrlSlaveConfig_s;

typedef struct QspictrlSlaveHcfg {
    char name[32]; ///<flash chip name
    U32 slaveId;
    QspictrlSlaveConfig_s slaveCfg;
} QspictrlSlaveHcfg_s;

/* 0x00: cmd reg, cmd + data_len */
typedef union QspictrlCmdReg {
    struct {
        U32 wrdata_len : 6; ///< [5:0]: RW, write_data or read_data length (bytes)
        U32 : 1; ///< [6]: Reserved
        U32 wrcmd : 1; ///< [7]: RW, 0:write_cmd; 1:read_cmd
        U32 : 24; ///< [31:8]: Reserved
    } fields;
    U32 dword;
} QspictrlCmdReg_u;

/*  0x04: address reg, object address for reading or writing */
typedef union QspictrlAddressReg {
    struct {
        U32 addr : 16; ///< [15:0]: RW, the address of register in slave device
        U32 : 16; ///< [16]: Reserved
    } fields;
    U32 dword;
} QspictrlAddressReg_u;

/*
 * 0x08 ~ 0x24: rw, wdata reg
 * 0x2c ~ 0x48: ro, rdata reg
 */
typedef union QspictrlRwdataReg {
    struct {
        U8 rwdata[4];
    } fields;
    U32 dword;
} QspictrlRwdataReg_u;

/* 0x28: start reg */
typedef union QspictrlStartReg {
    struct {
        U32 start : 1; ///< [0]: RW, update signal of write cmd frame to tx_fifo
        U32 : 31; ///< [31:1]: Reserved
    } fields;
    U32 dword;
} QspictrlStartReg_u;

/* 0x4c: update reg */
typedef union QspictrlUpdateReg {
    struct {
        U32 update : 1; ///< [0]: RW, update signal of receive data
        U32 : 31; ///< [31:1]: Reserved
    } fields;
    U32 dword;
} QspictrlUpdateReg_u;

/* 0x50: div reg */
typedef union QspictrlDivReg {
    struct {
        U32 div : 8; ///< [7:0]: RW, the divide value of sclock divided by clk_i
        U32 : 24; ///< [31:8]: Reserved
    } fields;
    U32 dword;
} QspictrlDivReg_u;

/* 0x54: dummy counter reg */
typedef union QspictrlDummyCntReg {
    struct {
        U32 dummy_cnt : 8; ///< [7:0]: RW, the number of sclock cycle in dummy phase
        U32 : 24; ///< [31:8]: Reserved
    } fields;
    U32 dword;
} QspictrlDummyCntReg_u;

/* 0x58: int raw reg */
typedef union QspictrlIntRawReg {
    struct {
        U32 rx_overflow : 1; ///< [0]: RC, rx_fifo is overflow
        U32 tx_overflow : 1; ///< [1]: RC, tx_fifo is overflow
        U32 crc_error : 1; ///< [2]: RC, received data crc check is failed
        U32 rx_valid : 1; ///< [3]: RO, rx_fifo is not empty
        U32 tx_fifo_empty : 1; ///< [4]: RO, tx_fifo is empty
        U32 : 27; ///< [31:5]: Reserved
    } fields;
    U32 dword;
} QspictrlIntRawReg_u;

/* 0x5c: fifo status reg */
typedef union QspictrlFifoStatusReg {
    struct {
        U32 rx_fifo_empty : 1; ///< [0]: RO, rx_fifo is empty
        U32 rx_fifo_full : 1; ///< [1]: RO, rx_fifo is full
        U32 rx_fifo_level : 2; ///< [3:2]: RO, rx fifo level
        U32 rx_fifo_size : 2; ///< [5:4]: RO, rx fifo is 2?
        U32 tx_fifo_empty : 1; ///< [6]: RO, tx_fifo is empty
        U32 tx_fifo_full : 1; ///< [7]: RO, tx_fifo is full
        U32 tx_fifo_level : 2; ///< [9:8]: RO, tx fifo level
        U32 tx_fifo_size : 2; ///< [11:10]: RO, tx fifo is 2?
        U32 : 20; ///< [31:12]: Reserved
    } fields;
    U32 dword;
} QspictrlFifoStatusReg_u;

/*  0x60: slave select reg */
typedef union QspictrlSlaveSelReg {
    struct {
        U32 slave_sel : 2; ///< [1:0]: RW, slave device sel: 0:ss0, 1: ss1, 2:ss2, 3: ss3
        U32 : 30; ///< [31:2]: Reserved
    } fields;
    U32 dword;
} QspictrlSlaveSelReg_u;

/* 0x64: interrupt control reg */
typedef union QspictrlIntCtrlReg {
    struct {
        U32 rx_overflow_mask : 1; ///< [0]: RW, mask rx overflow interrupt
        U32 tx_overflow_mask : 1; ///< [1]: RW, mask tx overflow interrupt
        U32 crc_error_mask : 1; ///< [2]: RW, mask crc error interrupt
        U32 rx_valid_mask : 1; ///< [3]: RW, mask rx valid interrupt
        U32 txfifo_empty_mask : 1; ///< [4]: RW, mask tx_empty interrupt
        U32 : 26; ///< [30:5]: Reserved
        ///< ps: taohb:excel and doc conflict
        U32 int_mode : 1; ///< [31]: RW, 1:interrupt mode; 0: polling mode
    } fields;
    U32 dword;
} QspictrlIntCtrlReg_u;

/*  0x68: rdata_id */
typedef union QspictrlRdataIdReg {
    struct {
        U32 rdata_addr : 16; ///< [15:0]: RO, the addr of data in rdata_reg
        U32 rdata_size : 6; ///< [21:16]: RO, the size of valid data in rdata_reg
        U32 : 9; ///< [30:22]: Reserved
        U32 rdata_crc : 1; ///< [31]: RO, 0:crc ok; 1: crc fail
    } fields;
    U32 dword;
} QspictrlRdataIdReg_u;

typedef union QspictrlSlaveResetReg {
    struct {
        U32 slave0_rst : 1; ///< [0]: RW, write 1/0 to release/reset slave0
        U32 slave1_rst : 1; ///< [1]: RW, write 1/0 to release/reset slave1
        U32 slave2_rst : 1; ///< [2]: RW, write 1/0 to release/reset slave2
        U32 slave3_rst : 1; ///< [3]: RW, write 1/0 to release/reset slave3
        U32 : 28;           ///< [31:4]: Reserved
    } fields;
    U32 dword;
} QspictrlSlaveResetReg_u;

typedef volatile struct QspictrlRegs {
    ///< 0x00: cmd reg, cmd + data_len
    QspictrlCmdReg_u cmd;
    ///< 0x04: address reg, object address for reading or writing
    QspictrlAddressReg_u addr;
    ///< 0x08 ~ 0x24: rw, wdata reg
    QspictrlRwdataReg_u wdata[8];
    ///< 0x28: start reg
    QspictrlStartReg_u start;
    ///< 0x2c ~ 0x48: ro, rdata reg
    QspictrlRwdataReg_u rdata[8];
    ///< 0x4c: update reg
    QspictrlUpdateReg_u update;
    ///< 0x50: div reg
    QspictrlDivReg_u div;
    ///< 0x54: dummy counter reg
    QspictrlDummyCntReg_u dummy_cnt;
    ///< 0x58: int raw reg
    QspictrlIntRawReg_u int_raw_status;
    ///< 0x5c: fifo status reg
    QspictrlFifoStatusReg_u fifo_status;
    ///< 0x60: slave select reg
    QspictrlSlaveSelReg_u slave_sel;
    ///< 0x64: interrupt control reg
    QspictrlIntCtrlReg_u int_ctrl;
    ///< 0x68: rdata_id
    QspictrlRdataIdReg_u rdata_id;
    ///< 0x6C: slave_reset
    QspictrlSlaveResetReg_u slave_rst;
} QspictrlRegs_vs;

typedef struct QspictrlHcfg {
    U32 mapAddr; ///< map address
    U32 interruptNum; ///< Interrupt number for Qspictrl_s
    U32 interruptPriority; ///< Interrupt priority for Qspictrl_s
} QspictrlHcfg_s;

typedef struct QspictrlSlave {
    QspictrlSlaveHcfg_s hcfg;
    Qspictrl_s *drv;
    bool initOk; ///< Initialize OK flag
    S32 (*init)(QspictrlSlave_s *dev, QspictrlSlaveHcfg_s *hcfg);
} QspictrlSlave_s;

typedef struct QspictrlSlaveMgr {
    U32 slaveNum;
    QspictrlSlave_s *slaveTbl[QSPICTRL_CS_MAX];
    bool initOk; ///< initialize OK flag
} QspictrlSlaveMgr_s;

typedef struct Qspictrl {
    QspictrlRegs_vs *regs;
    bool initOk; ///< Initialize OK flag
    QspictrlHcfg_s hcfg;
    QspictrlSlave_s *curSlave;
    OspRecursiveMutex_t slaveMutex; ///< for different slave select hander over
    OspID semIdInterrupt;
    S32 (*init)(Qspictrl_s *t, QspictrlHcfg_s *hcfg);
} Qspictrl_s;

#endif
