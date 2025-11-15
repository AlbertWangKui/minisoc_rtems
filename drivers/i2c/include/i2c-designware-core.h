/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file i2c-designware-core.h
 * @author 
 * @date 2025/06/06
 * @brief none
 * @version v1.0
 */
#ifndef __I2C_DESIGNWARE_CORE_H__
#define __I2C_DESIGNWARE_CORE_H__
#include <rtems.h>
#include <rtems/bspIo.h>
#include <bsp.h>

#define DW_I2C_CLK        (200000000)

#define min(a, b) ((a) > (b) ? (b) : (a))
#define	BIT(nr)			(1UL << (nr))
#define	BITS_PER_LONG		32
#define	GENMASK(hi, lo)		(((~0UL) << (lo)) & (~0UL >> (BITS_PER_LONG - 1 - (hi))))

/*
 ** Divide positive or negative dividend by positive or negative divisor
 ** and round to closest integer. Result is undefined for negative
 ** divisors if the dividend variable type is unsigned and for negative
 ** dividends if the divisor variable type is unsigned.
 **/
#define DIV_ROUND_CLOSEST(x, divisor)(          \
        {                           \
        typeof(x) __x = x;              \
        typeof(divisor) __d = divisor;          \
        (((typeof(x))-1) > 0 ||             \
         ((typeof(divisor))-1) > 0 ||           \
         (((__x) > 0) == ((__d) > 0))) ?        \
         (((__x) + ((__d) / 2)) / (__d)) :   \
         (((__x) - ((__d) / 2)) / (__d));    \
         }                           \
         )

/* I2C Frequency Modes */
#define I2C_MAX_STANDARD_MODE_FREQ  100000
#define I2C_MAX_FAST_MODE_FREQ      400000
#define I2C_MAX_FAST_MODE_PLUS_FREQ 1000000
#define I2C_MAX_TURBO_MODE_FREQ     1400000
#define I2C_MAX_HIGH_SPEED_MODE_FREQ    3400000
#define I2C_MAX_ULTRA_FAST_MODE_FREQ    5000000

///< this test code is tianlong_es
#define HW_I2C_CLK_H100     (1000)
#define HW_I2C_CLK_L100     (1000)
#define HW_I2C_CLK_H400     (250)
#define HW_I2C_CLK_L400     (250)
#define HW_I2C_CLK_H1000    (100)
#define HW_I2C_CLK_L1000    (100)
#define HW_I2C_CLK_H3400    (30)
#define HW_I2C_CLK_L3400    (30)

#define CHECK_ERR_TIMEOUT_COUNT (0x5ff)

#define I2C_CLIENT_PEC      0x04    /* Use Packet Error Checking */

#define DW_IC_CON_MASTER			BIT(0)
#define DW_IC_CON_SPEED_STD			(1 << 1)
#define DW_IC_CON_SPEED_FAST			(2 << 1)
#define DW_IC_CON_SPEED_HIGH			(3 << 1)
#define DW_IC_CON_SPEED_MASK			GENMASK(2, 1)
#define DW_IC_CON_10BITADDR_SLAVE		BIT(3)
#define DW_IC_CON_10BITADDR_MASTER		BIT(4)
#define DW_IC_CON_RESTART_EN			BIT(5)
#define DW_IC_CON_SLAVE_DISABLE			BIT(6)
#define DW_IC_CON_STOP_DET_IFADDRESSED		BIT(7)
#define DW_IC_CON_TX_EMPTY_CTRL			BIT(8)
#define DW_IC_CON_RX_FIFO_FULL_HLD_CTRL		BIT(9)
#define DW_IC_CON_STOP_DET_IF_MASTER_ACTIVE BIT(10)
#define DW_IC_CON_BUS_CLEAR_CTRL            BIT(11)

#define DW_IC_DATA_CMD_DAT			        GENMASK(7, 0)
#define DW_IC_DATA_CMD_FIRST_DATA_BYTE      BIT(11)

/*
 * Registers offset
 */
#define DW_IC_CON		0x00
#define DW_IC_TAR		0x04
#define DW_IC_SAR		0x08
#define DW_IC_DATA_CMD		0x10
#define DW_IC_SS_SCL_HCNT	0x14
#define DW_IC_SS_SCL_LCNT	0x18
#define DW_IC_FS_SCL_HCNT	0x1c
#define DW_IC_FS_SCL_LCNT	0x20
#define DW_IC_HS_SCL_HCNT	0x24
#define DW_IC_HS_SCL_LCNT	0x28
#define DW_IC_INTR_STAT		0x2c
#define DW_IC_INTR_MASK		0x30
#define DW_IC_RAW_INTR_STAT	0x34
#define DW_IC_RX_TL		0x38
#define DW_IC_TX_TL		0x3c
#define DW_IC_CLR_INTR		0x40
#define DW_IC_CLR_RX_UNDER	0x44
#define DW_IC_CLR_RX_OVER	0x48
#define DW_IC_CLR_TX_OVER	0x4c
#define DW_IC_CLR_RD_REQ	0x50
#define DW_IC_CLR_TX_ABRT	0x54
#define DW_IC_CLR_RX_DONE	0x58
#define DW_IC_CLR_ACTIVITY	0x5c
#define DW_IC_CLR_STOP_DET	0x60
#define DW_IC_CLR_START_DET	0x64
#define DW_IC_CLR_GEN_CALL	0x68
#define DW_IC_ENABLE		0x6c
#define DW_IC_STATUS		0x70
#define DW_IC_TXFLR		0x74
#define DW_IC_RXFLR		0x78
#define DW_IC_SDA_HOLD		0x7c
#define DW_IC_TX_ABRT_SOURCE	0x80
#define DW_IC_ACK_GENERAL_CALL	0x98
#define DW_IC_ENABLE_STATUS	0x9c
#define DW_IC_FS_SPKLEN		0xa0
#define DW_IC_HS_SPKLEN		0xa4
#define DW_IC_CLR_RESTART_DET	0xa8
#define DW_IC_SCL_STUCK_AT_LOW_TIMEOUT 0xac
#define DW_IC_SDA_STUCK_AT_LOW_TIMEOUT 0xb0
#define DW_IC_SMBUS_INTR_STAT   0xc8
#define DW_IC_SMBUS_INTR_MASK	0xcc

#define DW_IC_CLR_SMBUS_INTR   0xd4
#define DW_IC_SMBUS_UDID_WORD0 0xdc
#define DW_IC_SMBUS_UDID_WORD1 0xe0
#define DW_IC_SMBUS_UDID_WORD2 0xe4
#define DW_IC_SMBUS_UDID_WORD3 0xe8

#define DW_IC_SAR2             0x100

#define DW_IC_SMBUS2_UDID_WORD0 0x124
#define DW_IC_SMBUS2_UDID_WORD1 0x128
#define DW_IC_SMBUS2_UDID_WORD2 0x12c
#define DW_IC_SMBUS2_UDID_WORD3 0x130

#define DW_IC_SMBUS3_UDID_WORD0 0x134
#define DW_IC_SMBUS3_UDID_WORD1 0x138
#define DW_IC_SMBUS3_UDID_WORD2 0x13c
#define DW_IC_SMBUS3_UDID_WORD3 0x140

#define DW_IC_SMBUS4_UDID_WORD0 0x144
#define DW_IC_SMBUS4_UDID_WORD1 0x148
#define DW_IC_SMBUS4_UDID_WORD2 0x14c
#define DW_IC_SMBUS4_UDID_WORD3 0x150

#define DW_IC_COMP_PARAM_1	0xf4
#define DW_IC_COMP_VERSION	0xf8
#define DW_IC_SDA_HOLD_MIN_VERS	0x3131312A /* "111*" == v1.11* */
#define DW_IC_COMP_TYPE		0xfc
#define DW_IC_COMP_TYPE_VALUE	0x44570140 /* "DW" + 0x0140 */ 

#define DW_IC_INTR_RX_UNDER	BIT(0)
#define DW_IC_INTR_RX_OVER	BIT(1)
#define DW_IC_INTR_RX_FULL	BIT(2)
#define DW_IC_INTR_TX_OVER	BIT(3)
#define DW_IC_INTR_TX_EMPTY	BIT(4)
#define DW_IC_INTR_RD_REQ	BIT(5)
#define DW_IC_INTR_TX_ABRT	BIT(6)
#define DW_IC_INTR_RX_DONE	BIT(7)
#define DW_IC_INTR_ACTIVITY	BIT(8)
#define DW_IC_INTR_STOP_DET	BIT(9)
#define DW_IC_INTR_START_DET	BIT(10)
#define DW_IC_INTR_GEN_CALL	BIT(11)
#define DW_IC_INTR_RESTART_DET	BIT(12)
#define DW_IC_INTR_MST_ON_HOLD  BIT(13)
#define DW_IC_INTR_SCL_STUCK_AT_LOW  BIT(14)

#define DW_IC_INTR_DEFAULT_MASK		(DW_IC_INTR_RX_FULL | \
        DW_IC_INTR_TX_ABRT | \
        DW_IC_INTR_STOP_DET)
#define DW_IC_INTR_MASTER_MASK		(DW_IC_INTR_DEFAULT_MASK | \
        DW_IC_INTR_TX_EMPTY)
#define DW_IC_INTR_SLAVE_MASK		(DW_IC_INTR_DEFAULT_MASK | \
        DW_IC_INTR_RX_UNDER | \
        DW_IC_INTR_RD_REQ  | \
        DW_IC_INTR_RESTART_DET)

#define DW_IC_ENABLE_ENABLE                 BIT(0)
#define DW_IC_ENABLE_ABORT                  BIT(1)
#define DW_IC_ENABLE_SDA_STUCK_RECOVERY     BIT(3)

#define DW_IC_STATUS_ACTIVITY		BIT(0)
#define DW_IC_STATUS_TFE		BIT(2)
#define DW_IC_STATUS_RFNE		BIT(3)
#define DW_IC_STATUS_MASTER_ACTIVITY	BIT(5)
#define DW_IC_STATUS_SLAVE_ACTIVITY	BIT(6)
#define DW_IC_STATUS_MASTER_HOLD_TX_FIFO_EMPTY	BIT(7)
#define DW_IC_STATUS_SDA_STUCK_NOT_RECOVERED BIT(11)

#define DW_IC_SDA_HOLD_RX_SHIFT		16
#define DW_IC_SDA_HOLD_RX_MASK		GENMASK(23, 16)

#define DW_IC_ERR_TX_ABRT	0x1

#define DW_IC_TAR_10BITADDR_MASTER	BIT(12)

#define DW_IC_COMP_PARAM_1_SPEED_MODE_HIGH	(BIT(2) | BIT(3))
#define DW_IC_COMP_PARAM_1_SPEED_MODE_MASK	GENMASK(3, 2)

/*
 * Sofware status flags
 */
#define STATUS_ACTIVE               BIT(0)        
#define STATUS_WRITE_IN_PROGRESS    BIT(1)        
#define STATUS_READ_IN_PROGRESS     BIT(2)
#define STATUS_MASK                 GENMASK(2, 0) 

#define IC_INTR_STAT_STOP_DET       (1 << 9)
#define IC_INTR_STAT_RESTART_DET    (1 << 12)
#define IC_INTR_STAT_TX_EMPTY       (1 << 4)
#define IC_INTR_STAT_RX_FULL        (1 << 2)
#define IC_INTR_STAT_WR_REQ         (1 << 15)
#define IC_INTR_STAT_RD_REQ         (1 << 5)
#define IC_INTR_STAT_TX_ABRT        (1 << 6)

#define IC_TX_ABRT_SLV_ARBLOST      (1 << 14)

#define IC_INT_COMMON_ENABLE        (IC_INTR_STAT_STOP_DET | IC_INTR_STAT_RESTART_DET)
#define IC_INT_MST_TX_ENABLE        (IC_INT_COMMON_ENABLE | IC_INTR_STAT_TX_EMPTY)
#define IC_INT_MST_RX_ENABLE        (IC_INT_COMMON_ENABLE|IC_INTR_STAT_RX_FULL)

#define IC_INT_SLV_TX_ENABLE        (IC_INT_COMMON_ENABLE | IC_INTR_STAT_RD_REQ)
#ifdef __DW_SMBUS__
#define IC_INT_SLV_RX_ENABLE        (IC_INT_COMMON_ENABLE | IC_INTR_STAT_WR_REQ)
#else
#define IC_INT_SLV_RX_ENABLE        (IC_INT_COMMON_ENABLE | IC_INTR_STAT_RX_FULL)
#endif

/*
 * operation modes
 */
#define DW_IC_MASTER		0
#define DW_IC_SLAVE		1

#define DEV_DISABLED                (0) ///< Bit 0 for device enabled state, disabled 
#define DEV_ENABLED                 (1<<0)  ///< Bit 0 for device enabled state, enabled 

/*
 * Hardware abort codes from the DW_IC_TX_ABRT_SOURCE register
 *
 * Only expected abort codes are listed here
 * refer to the datasheet for the full list
 */
#define ABRT_7B_ADDR_NOACK	0
#define ABRT_10ADDR1_NOACK	1
#define ABRT_10ADDR2_NOACK	2
#define ABRT_TXDATA_NOACK	3
#define ABRT_GCALL_NOACK	4
#define ABRT_GCALL_READ		5
#define ABRT_SBYTE_ACKDET	7
#define ABRT_SBYTE_NORSTRT	9
#define ABRT_10B_RD_NORSTRT	10
#define ABRT_MASTER_DIS		11
#define ARB_LOST		12
#define ABRT_SLAVE_FLUSH_TXFIFO	13
#define ABRT_SLAVE_ARBLOST	14
#define ABRT_SLAVE_RD_INTX	15
#define ABRT_SDA_STUCK_AT_LOW	17

#define DW_IC_TX_ABRT_7B_ADDR_NOACK		BIT(ABRT_7B_ADDR_NOACK)
#define DW_IC_TX_ABRT_10ADDR1_NOACK		BIT(ABRT_10ADDR1_NOACK)
#define DW_IC_TX_ABRT_10ADDR2_NOACK		BIT(ABRT_10ADDR2_NOACK)
#define DW_IC_TX_ABRT_TXDATA_NOACK		BIT(ABRT_TXDATA_NOACK)
#define DW_IC_TX_ABRT_GCALL_NOACK		BIT(ABRT_GCALL_NOACK)
#define DW_IC_TX_ABRT_GCALL_READ		BIT(ABRT_GCALL_READ)
#define DW_IC_TX_ABRT_SBYTE_ACKDET		BIT(ABRT_SBYTE_ACKDET)
#define DW_IC_TX_ABRT_SBYTE_NORSTRT		BIT(ABRT_SBYTE_NORSTRT)
#define DW_IC_TX_ABRT_10B_RD_NORSTRT		BIT(ABRT_10B_RD_NORSTRT)
#define DW_IC_TX_ABRT_MASTER_DIS		BIT(ABRT_MASTER_DIS)
#define DW_IC_TX_ARB_LOST		    	BIT(ARB_LOST)
#define DW_IC_TX_ABRT_SDA_STUCK_AT_LOW  BIT(ABRT_SDA_STUCK_AT_LOW)
#define DW_IC_RX_ABRT_SLAVE_RD_INTX		BIT(ABRT_SLAVE_RD_INTX)
#define DW_IC_RX_ABRT_SLAVE_ARBLOST		BIT(ABRT_SLAVE_ARBLOST)
#define DW_IC_RX_ABRT_SLAVE_FLUSH_TXFIFO	BIT(ABRT_SLAVE_FLUSH_TXFIFO)

#define DW_IC_TX_ABRT_NOACK		(DW_IC_TX_ABRT_7B_ADDR_NOACK | \
        DW_IC_TX_ABRT_10ADDR1_NOACK | \
        DW_IC_TX_ABRT_10ADDR2_NOACK | \
        DW_IC_TX_ABRT_TXDATA_NOACK | \
        DW_IC_TX_ABRT_GCALL_NOACK)

#define IC_SMBUS_INTR_STAT_ARP_ASSGN_ADDR_CMD_DET   (1 << 7)
#define IC_SMBUS_ARP_ASSGIN_ADDR_CMD_BIT            (1 << 7)
#define IC_SMBUS_ARP_GET_UDID_CMD_BIT               (1 << 6)
#define IC_SMBUS_ARP_RESET_CMD_BIT                  (1 << 5)
#define IC_SMBUS_ARP_PREPARE_CMD_BIT                (1 << 4)

#define IC_SMBUS_MST_NOTIFY_ADDR             (0x8)
#define IC_SMBUS_MST_NOTIFY_DEVID            (0xc2)
#define IC_SMBUS_MST_NOTIFY_DATA0            (0x0)
#define IC_SMBUS_MST_NOTIFY_DATA1            (0x0)

#define IC_STATUS_SMBUS_SLAVE_ADDR_VALID            (1 << 17)
#define IC_STATUS_SMBUS_SLAVE_ADDR_RESOLVED         (1 << 18)
#define IC_STATUS_SMBUS_SLAVE_ADDR2_VALID           (1 << 21)
#define IC_STATUS_SMBUS_SLAVE_ADDR2_RESOLVED        (1 << 22)

#define ACCESS_INTR_MASK	BIT(0)
#define ACCESS_NO_IRQ_SUSPEND	BIT(1)
#define ARBITRATION_SEMAPHORE	BIT(2)

#define MODEL_MSCC_OCELOT	BIT(8)
#define MODEL_BAIKAL_BT1	BIT(9)
#define MODEL_AMD_NAVI_GPU	BIT(10)
#define MODEL_MASK		GENMASK(11, 8)

/*
 * Enable UCSI interrupt by writing 0xd at register
 * offset 0x474 specified in hardware specification.
 */
#define AMD_UCSI_INTR_REG	0x474
#define AMD_UCSI_INTR_EN	0xd

#define IC_CON_SMBUS_ARP_EN               (1 << 18)
#define IC_SAR_ENABLE                     (1<<19)
#define IC_CON_SAR2_SMBUS_ARP_EN          (1 << 23)

#define IC_SAR_NUM                                   (1)
#define IC_SAR2_NUM                                  (2)
#define IC_SAR_NUM_COUNT                             (2)
#define IC_UDID_NUM_COUNT                            (4)

#define I2C_NAME_LEN 16

typedef void (*user_callback)(void *);

/**
 ** struct i2c_timings - I2C timing information
 ** @bus_freq_hz: the bus frequency in Hz
 **/

enum i2c_slave_event {
    I2C_SLAVE_READ_REQUESTED,
    I2C_SLAVE_WRITE_REQUESTED,
    I2C_SLAVE_READ_PROCESSED,
    I2C_SLAVE_WRITE_RECEIVED,
    I2C_SLAVE_STOP,
};

enum {
    SMBUS_CMD_PREPARE_4_ARP = 0x01,
    SMBUS_CMD_GENERAL_RESET_DEVICE = 0x02,
    SMBUS_CMD_GENERAL_GET_UDID = 0x03,
    SMBUS_CMD_GENERAL_ASSIGN_ADDR = 0x04,
};

typedef struct {
    ArpAssignAddrFinishCb arpAddrAssignedCB;
    void *param;
}arpAddrAssignedCbInfos_s;
struct dw_i2c_dev {
    U32		*base;
    U32 i2c_channel_num;
    U32 is_smbus;
    char i2c_name[I2C_NAME_LEN];
    rtems_id semaphore_id;
    int			cmd_err;
    struct i2c_msg		*msgs;
    int			msgs_num;
    int			msg_write_idx;
    U32			tx_buf_len;
    U8			*tx_buf;
    int			msg_read_idx;
    U32			rx_buf_len;
    U8			*rx_buf;
    int			msg_err;
    unsigned int		status;
    U32			abort_source;
    U32			irq;
    U32			flags;
    U32			master_cfg;
    U32			slave_cfg;
    unsigned int		tx_fifo_depth;
    unsigned int		rx_fifo_depth;
    int			rx_outstanding;
    struct i2c_timings  timings;
    U32			sda_hold_time;
    U16         fs_spklen;
    U16         hs_spklen;
    U16			ss_hcnt;
    U16			ss_lcnt;
    U16			fs_hcnt;
    U16			fs_lcnt;
    U16			fp_hcnt;
    U16			fp_lcnt;
    U16			hs_hcnt;
    U16			hs_lcnt;
    int			mode;
    unsigned int addr_mode; ///< user config addr mode 7bit/10bit
    unsigned int addr; ///< slave addr
    unsigned int work_mode; ///< 驱动工作模式：0为中断模式（默认），1为轮询模式
    unsigned int transfer_timeout; ///< master 模式传输等待超时时间,单位ms
    user_callback callback_func;
    void *callback_param;
    user_callback callback_read_func;
    void *callback_read_param;
    user_callback callback_write_func;
    void *callback_write_param;
    DevCallBack arbCb;
    void *arbCbParam;
#define SLAVE_BUF_LEN   (1024)
    U32 slave_valid_rx_len;
    U32 slave_valid_usr_tx_len;
    U32 slave_cur_tx_index;
    U8 *slave_rx_buf;
    U8 *slave_tx_buf;
    arpAddrAssignedCbInfos_s arpAssignedCB;
    U32 enabled;
};

U32 i2c_dw_scl_hcnt(U32 ic_clk, U32 tSYMBOL, U32 tf, int cond, int offset);
U32 i2c_dw_scl_lcnt(U32 ic_clk, U32 tLOW, U32 tf, int offset);
int i2c_dw_calc_sda_hold(struct dw_i2c_dev *dev);
int i2c_dw_wait_bus_not_busy(struct dw_i2c_dev *dev);
int i2c_dw_handle_tx_abort(struct dw_i2c_dev *dev);
int i2c_dw_calc_fifo_size(struct dw_i2c_dev *dev);
void i2c_dw_disable(struct dw_i2c_dev *dev);
void i2c_dw_disable_int(struct dw_i2c_dev *dev);

static inline int dw_reg_read(struct dw_i2c_dev *dev, U32 reg, U32 *val)
{
    *val = *(volatile U32 *)((uintptr_t)(dev->base) + reg);
#if defined(__ARM_ARCH_8A)
    __asm__ __volatile__("dmb oshld" : : : "memory");
#else
    __asm__ __volatile__("dsb" : : : "memory");
#endif
    return 0;
}

static inline int dw_reg_write(struct dw_i2c_dev *dev, U32 reg, U32 val)
{
#if defined(__ARM_ARCH_8A)
    __asm__ __volatile__("dmb oshst" : : : "memory");
#else
    __asm__ __volatile__("dsb st" : : : "memory");
#endif
    *(volatile U32 *)((uintptr_t)(dev->base) + reg) = val;
    return 0;
}

static inline void __i2c_dw_enable(struct dw_i2c_dev *dev)
{
    U32 tmp = 0;

    dw_reg_read(dev,DW_IC_ENABLE,&tmp);
    tmp |= (1);
    dw_reg_write(dev,DW_IC_ENABLE,tmp);
    dev->status |= STATUS_ACTIVE;
}

static inline void __i2c_dw_disable_nowait(struct dw_i2c_dev *dev)
{
    U32 tmp = 0;

    dw_reg_read(dev, DW_IC_ENABLE, &tmp);
    tmp &= ~(0x1);
    dw_reg_write(dev, DW_IC_ENABLE, tmp);
    dev->status &= ~STATUS_ACTIVE;
}

void __i2c_dw_disable(struct dw_i2c_dev *dev);

void i2c_dw_configure_master(struct dw_i2c_dev *dev);
int i2c_dw_probe_master(struct dw_i2c_dev *dev);
int i2c_dw_unprobe_master(struct dw_i2c_dev *dev);
int i2c_dw_xfer(struct dw_i2c_dev *dev, struct i2c_msg msgs[], int num);

int i2c_dw_set_slave_addr(struct dw_i2c_dev *dev);
void i2c_dw_configure_slave(struct dw_i2c_dev *dev);
int i2c_dw_probe_slave(struct dw_i2c_dev *dev);
int i2c_dw_unprobe_slave(struct dw_i2c_dev *dev);
void i2c_dw_lock_slave(ISR_lock_Context *lock_contex);
void i2c_dw_unlock_slave(ISR_lock_Context *lock_contex);
unsigned long i2c_dw_clk_rate(struct dw_i2c_dev *dev);
static inline int i2c_dw_probe(struct dw_i2c_dev *dev)
{
    switch (dev->mode) {
        case DW_IC_SLAVE:
            return i2c_dw_probe_slave(dev);
        case DW_IC_MASTER:
            return i2c_dw_probe_master(dev);
        default:
            LOGE("Wrong operation mode: %d\n", dev->mode);
            return -1;
    }
}

#endif
