/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_i2c_api.h
 * @author
 * @date 2025/06/06
 * @brief none
 * @version v1.0
 */

#ifndef __DRV_I2C_API_H__
#define __DRV_I2C_API_H__

#include "common_defines.h"
#include "bsp_device.h"

#define I2C_M_TEN 0x0010
#define I2C_M_RD 0x0001
#define I2C_M_RECV_LEN 0x0400
#define I2C_SMBUS_BLOCK_MAX 32

#define I2C_M_STOP 0x8000
#define I2C_M_NOSTART 0x4000
#define I2C_M_REV_DIR_ADDR 0x2000
#define I2C_M_IGNORE_NAK 0x1000
#define I2C_M_NO_RD_ACK 0x0800

typedef void (*ArpAssignAddrFinishCb)(U32 addrId, U32 slvAddr, void *ptr);

/* emu for device working i2c_mode */
typedef enum DwI2cMode
{
    DEV_SLAVE_MODE = 0, /* Indicate that the device working as slave */
    DEV_MASTER_MODE = 1, /* Indicate that the device working as master */
    DEV_INVALID_MODE = 255 /* Indicate that the device not init */
} DwI2cMode_t;

/* emu for device working method */
typedef enum DevMethod
{
    DEV_POLL_METHOD = 0,        /* Indicate that the device running in poll method */
    DEV_INTERRUPT_METHOD = 1    /* Indicate that the device running in interrupt method */
} DevMethod_t;

/* I2C Bus possible speed */
typedef enum I2cSpeedMode
{
    I2C_SPEED_STANDARD = 0, /* Bidirectional, Standard-i2c_mode (Sm), with a bit rate up to 100 kbit/s */
    I2C_SPEED_FAST     = 1, /* Bidirectional, Fast-i2c_mode (Fm), with a bit rate up to 400 kbit/s */
    I2C_SPEED_FASTPLUS = 2, /* Bidirectional, Fast-i2c_mode Plus (Fm+), with a bit rate up to 1 Mbit/s */
    I2C_SPEED_HIGH     = 3, /* Bidirectional, High-speed i2c_mode (Hs-i2c_mode), with a bit rate up to 3.4 Mbit/s */
    I2C_SPEED_ULTRA    = 4, /* Bidirectional, Ultra-fast i2c_mode (Uf-i2c_mode), with a bit rate up to 5 Mbit/s */
    I2C_SPEED_INVALID  = 255
} I2cSpeedMode_t;

struct i2c_msg {
  /**
   * @brief The slave address.
   *
   * In case the @ref I2C_M_TEN flag is set, then this is a 10-bit address,
   * otherwise it is a 7-bit address.
   */
  U16 addr;

  /**
   * @brief The message flags.
   *
   * Valid flags are
   * - @ref I2C_M_TEN,
   * - @ref I2C_M_RD,
   * - @ref I2C_M_STOP,
   * - @ref I2C_M_NOSTART,
   * - @ref I2C_M_REV_DIR_ADDR,
   * - @ref I2C_M_IGNORE_NAK,
   * - @ref I2C_M_NO_RD_ACK, and
   * - @ref I2C_M_RECV_LEN.
   */
  U16 flags;

  /**
   * @brief The message data length in bytes.
   */
  U16 len;

  /**
   * @brief Pointer to the message data.
   */
  U8 *buf;
};


#define IC_CON_SPEED_MASK       (0x6)
#define IC_CON_SPEED_STANDARD   (0x2)
#define IC_CON_SPEED_FAST       (0x4)
#define IC_CON_SPEED_HIGH       (0x6)

#define IC_DELAY_25US                               (25)
#define IC_DELAY_100US                              (100)
#define IC_DELAY_25US_COUNT                         (40000)
#define IC_DELAY_100US_COUNT                        (10000)
#define IC_1US_TO_1000NANO                          (1000)

///< IC_ENABLE_STATUS Bits
#define IC_ENABLE_STATUS_IC_EN          (1 << 0)
#define IC_ENABLE_STATUS_SLV_DIS        (1 << 1)
#define IC_ENABLE_STATUS_SLV_RX_LOST    (1 << 2)

///< Enable DW I2C
#define DW_I2C_ENABLE          (1)
///< Disable DW I2C
#define DW_I2C_DISABLE         (0)

///< Working i2c_mode of IC_CON
#define IC_CON_MST_SLV_MODE_MASK    (0x41)
#define IC_CON_ENA_MASTER_MODE      (0x41)
#define IC_CON_ENA_SLAVE_MODE       (0)
#define STOP_DET_IFADDRESSED        (1 << 7)
#define STOP_DET_IF_MASTER_ACTIVE   (1 << 10)

#define IC_CON_BUS_CLEAR_FEATURE_CTRL_MASK    (1 << 11)

/* I2C Addressing Mode */
typedef enum i2caddressi2cmode
{
    I2C_7BIT_ADDRESS  = 0,  /* Use 7bit address i2c_mode */
    I2C_10BIT_ADDRESS = 1   /* Use 10bit address i2c_mode */
} I2cAddressMode_t;

///< arp udid info
typedef struct DwSmbusArpUdid
{
    U32 sar_udid0;
    U32 sar_udid1;
    U32 sar_udid2;
    U32 sar_udid3;
} DwSmbusArpUdid_t;

///< 由于部分寄存器是读清，所以只记录非读清部分寄存器
typedef struct I2cDebugInfo {
    U32 ic_con;
    U32 ic_tar;
    U32 ic_sar;
//    U32 ic_hs_mar;
    U32 ic_ss_scl_hcnt;
    U32 ic_ss_scl_lcnt;
    U32 ic_fs_scl_hcnt;
    U32 ic_fs_scl_lcnt;
    U32 ic_hs_scl_hcnt;
    U32 ic_hs_scl_lcnt;
    U32 ic_intr_stat;
    U32 ic_intr_mask;
    U32 ic_raw_intr_stat;
    U32 ic_rx_tl;
    U32 ic_tx_tl;
    U32 ic_enable;
    U32 ic_status;
    U32 ic_txflr;
    U32 ic_rxflr;
    U32 ic_sda_hold;
    U32 ic_tx_abrt_source;
//    U32 ic_slv_data_nack_only;
//    U32 ic_sda_setup;
    U32 ic_enable_status;
    U32 ic_fs_spklen;
    U32 ic_hs_spklen;
//    U32 ic_scl_stuck_at_low_timeout;
//    U32 ic_sda_stuck_at_low_timeout;
//    U32 ic_device_id;
    U32 ic_comp_param_1;
} I2cDebugInfo_s;


typedef void (*DevCallBack) (void *ptr);

/**
 * @brief init i2c-channel.
 * @param [in] i2cNum I2C设备编号，从 0开始，最大编号为实际设备数-1
 * @param [in] i2cMode  设备作为master（DEV_MASTER_MODE）还是slave（DEV_SLAVE_MODE）
 * @param [in] method： 设备是以轮询还是中断方式工作
 * @param [in] speed：设备工作速度
 * @return 0表示初始化成功，其他为错误
 * @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
int i2cInit(DevList_e i2cNum, DwI2cMode_t i2cMode, DevMethod_t method,
        I2cSpeedMode_t speed);

/**
 *  @brief read i2c device
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @param [in] addr:  i2c协议地址
 *  @param [in] data： 接收数据的缓冲区
 *  @param [in] len： 读取的长度
 *  @return 作为master时，0表示读取成功，其他为错误；作为slave时，返回值为实际读取数据长度
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
int i2cRead(DevList_e i2cNum, int addr, void *data, U32 len);

/**
 *  @brief write i2c device
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @param [in] addr:  i2c协议地址
 *  @param [in] data：  待写数据的缓冲区
 *  @param [in] len：待写的长度
 *  @return 0表示写入成功，其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
int i2cWrite(DevList_e i2cNum, int addr, void *data, U32 len);

/**
 *  @brief 连续（连续读、连续写或者连续读写）I2C操作
 *  @param [in] i2cDevId:  设备ID，虚拟设备ID；
 *  @param [in] msgs:  连续读写操作的节点信息，包含slave协议地址、源数据地址和大小
 *  @param [in] num  节点数
 *  @return 0表示成功，其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
int i2cTransfer(int i2cDevId, struct i2c_msg msgs[], int num);

/**
 *  @brief  reset i2c device
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @return none
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
void i2cReset(DevList_e i2cNum);

/**
 *  @brief  free i2c device
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @return none
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
void i2cDeInit(DevList_e i2cNum);

/**
 *  @brief  install app callback
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @param [in] callBackRd：master读取完成回调
 *  @param [in] paramRd：业务自定义参数
 *  @param [in] callBackWr：master写入完成回调
 *  @param [in] 业务自定义参数
 *  @return none
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
void i2cIntrReg(DevList_e i2cNum, DevCallBack callBackRd, void *paramRd,
        DevCallBack callBackWr, void *paramWr);


/**
 *  @brief  check scl state
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @return 0： scl正常;1： scl死锁;小于0：函数执行错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
int i2cSclStuckCheck(DevList_e i2cNum);

/**
 *  @brief  install callback for arp done
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @param [in] arpAddrFinishCb 分配地址业务回调函数
 *  @param [in] paramArp 回调函数参数，业务自定义
 *  @return 0，执行成功；其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 i2cRegArpAssignAddrFinishCb(DevList_e i2cId,ArpAssignAddrFinishCb arpAddrFinishCb, void* paramArp);

/**
 *  @brief  set udid
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @param [in] sarNum slave address num
 *  @param [out] udid slave udid to be set
 *  @return 0，执行成功；其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 i2cSmbusArpUdidSet(DevList_e i2cNum,U8 sarNum,DwSmbusArpUdid_t *udid);

/**
 *  @brief  get udid
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @param [in] sarNum slave address num
 *  @param [out] udid return slave udid
 *  @return 0，执行成功；其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 i2cSmbusArpUdidGet(DevList_e i2cNum,U8 sarNum,DwSmbusArpUdid_t *udid);

/**
 *  @brief  assign slave address
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @param [in] assign_addr slave address
 *  @return 0，执行成功；其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 i2cSmbusDevAddrAssign(DevList_e i2cNum,U8 assign_addr);

/**
 *  @brief  enable arp
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @param [in] sarNum  slave address num
 *  @return 0，执行成功；其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 i2cSmbusArpEnable(DevList_e i2cNum,U8 sarNum);

/**
 *  @brief  disable arp
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @param [in] sarNum  slave address num
 *  @return 0，执行成功；其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 i2cSmbusArpDisable(DevList_e i2cNum,U8 sarNum);

/**
 *  @brief  enable slave address
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @param [in] sarNum  slave address num
 *  @return 0，执行成功；其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 i2cSmbusSarEnable(DevList_e i2cNum,U8 sarNum);

/**
 *  @brief  disable slave address
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @param [in] sarNum  slave address num
 *  @return 0，执行成功；其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 i2cSmbusSarDisable(DevList_e i2cNum,U8 sarNum);

/**
 *  @brief  get ARP address status(reosoveld or not)
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @return 0，执行成功；其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 i2cSmbusArpAddrResolvedGetStatus(DevList_e i2cNum, U32 sarNum);

/**
 *  @brief  get ARP address status(valid or not)
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @return 0，执行成功；其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 i2cSmbusArpAddrValidGetStatus(DevList_e i2cNum, U32 sarNum);

/**
 *  @brief  host cmd notify
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @return 0，执行成功；其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 i2cSmbusMasterArpNotifyCmd(DevList_e i2cNum);

/**
 *  @brief  switch master or slave mode
 *  @param [in] i2cNum:  设备ID，虚拟设备ID；
 *  @param [in] i2cMode  work mode
 *  @return 0，执行成功；其他为错误
 *  @warning 非阻塞；不可重入；OS启动后；不可用于中断上下文；可以用于线程上下文
 */
S32 i2cSwitchMode(DevList_e i2cNum, DwI2cMode_t i2cMode);

#define I2C_TRANSFER_TIMEOUT_MIN_NUM    (20)
#define I2C_TRANSFER_TIMEOUT_DEF_NUM    (1000)
#define INVALID_SDA_HOLD_TIME           (0x0)

#define DEV_SYS_CMDBSE              (0x00000000)
#define DEV_SET_SYSCMD(cmd)         (DEV_SYS_CMDBSE|(cmd))
/* Define I2C control commands for common usage */
#define DEV_SET_I2C_SYSCMD(cmd)     DEV_SET_SYSCMD((cmd))
/* Define I2C control commands for slave usage */
#define DEV_SET_I2C_SLV_SYSCMD(cmd) DEV_SET_SYSCMD(0x00008000|(cmd))

///< I2C_CMD_GET_STATUS 兼容业务API，实际是获取设备是否使能的状态
#define I2C_CMD_GET_STATUS          DEV_SET_I2C_SYSCMD(0)

#define I2C_CMD_SET_TXINT           DEV_SET_I2C_SYSCMD(7)
#define I2C_CMD_SET_RXINT           DEV_SET_I2C_SYSCMD(8)
#define I2C_CMD_RESET               DEV_SET_I2C_SYSCMD(11)
#define I2C_CMD_FLUSH_TX            DEV_SET_I2C_SYSCMD(12)
#define I2C_CMD_FLUSH_RX            DEV_SET_I2C_SYSCMD(13)
#define I2C_CMD_ENA_DEV             DEV_SET_I2C_SYSCMD(14)
#define I2C_CMD_DIS_DEV             DEV_SET_I2C_SYSCMD(15)

/*
 * Set \ref dev_i2c_info::addr_i2c_mode "i2c addressing i2c_mode".
 * - Param type : uint32_t
 * - Param usage : i2c addressing i2c_mode, possible values can be found \ref I2C_ADDRESS_MODE "here"
 * - Return value explanation :
 */
#define I2C_CMD_SET_ADDR_MODE           DEV_SET_I2C_SYSCMD(1)
/*
 * Set \ref dev_i2c_info::transfer_timeout "i2c master transfer timeout(ms)".
 * - Param type : uint32_t
 * - Param usage : i2c master transfer timeout(ms), default 1000ms,min:20ms;时间计算有10ms偏差
 * - Return value explanation :
 */
#define I2C_CMD_SET_TRANSFER_TIMEOUT    DEV_SET_I2C_SYSCMD(2)
/* ++++ Slave only commands for I2C Device ++++ */
/*
 * Set slave address when working as slave i2c device
 * - Param type : uint32_t
 * - Param usage : slave address value
 * - Return value explanation :
 */
#define I2C_CMD_SLV_SET_SLV_ADDR        DEV_SET_I2C_SLV_SYSCMD(21)
/*
 * Set \ref dev_i2c_info::sda_hold_time "i2c sda hold time".
 * - Param type : uint32_t
 * - Param usage : i2c sda hold time(in units of ic_clk period). Bit0~bit15 for IC_SDA_TX_HOLD, bit16~bit23 for IC_SDA_RX_HOLD.
 * - Return value explanation :
 */
#define I2C_CMD_SET_SDA_HOLD_TIME       DEV_SET_I2C_SYSCMD(3)

/*
 * - Param type : DwI2cMode_t
 * - Param usage : channel working mode
 * - Return value explanation :
 */
#define I2C_CMD_GET_CHANNEL_MODE       DEV_SET_I2C_SYSCMD(4)

int i2cControl(DevList_e i2cDevId, U32 ctrlCmd, void *param);

/*******************************************************************************
@ Brief: regArbitrationFailedCb
@ Param [i2cNum]: i2c enum ID
@ param callBackArb,callback handler
@ param paramArb,callback parameter
@ Return: none
@ Note: none
*******************************************************************************/
void i2cRegMstArbitrationFailedCb(DevList_e i2cDevId,DevCallBack callBackArb, void* paramArb);
#endif
