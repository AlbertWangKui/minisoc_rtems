/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file i2c-designware-master.c
 * @author 
 * @date 2025/06/06
 * @brief none
 * @version v1.0
 */

#include <stdio.h>
#include <rtems.h>
#include <rtems/counter.h>
#include <rtems/bspIo.h>
#include <rtems/irq-extension.h>
#include <bsp.h>
#include <errno.h>
#include "common_defines.h"
#include "bsp_device.h"
#include "udelay.h"
#include "log_msg.h"
#include "drv_i2c_api.h"
#include "../include/i2c-designware-core.h"

#define SMBUS_ARP_DEFAULT_ADDR (0x61)
#define CRC8_POLY (0x1070U << 3)

static void i2c_dw_configure_fifo_master(struct dw_i2c_dev *dev)
{
    /* Configure Tx/Rx FIFO threshold levels */
    dw_reg_write(dev, DW_IC_TX_TL, dev->tx_fifo_depth / 2);
    dw_reg_write(dev, DW_IC_RX_TL, 0);

    ///<  master_cfg需要根据用户参数生成配置，参考i2c_dw_configure_master
    /* Configure the I2C master */
    dw_reg_write(dev, DW_IC_CON, dev->master_cfg);
}

static int i2c_dw_calc_sclcnt_master(struct dw_i2c_dev *dev)
{
    dev->ss_lcnt = 0;
    dev->ss_hcnt = 0;
    dev->fs_lcnt = 0;
    dev->fs_hcnt = 0;
    dev->hs_lcnt = 0;
    dev->hs_hcnt = 0;
    switch(dev->timings.bus_freq_hz) {
        case I2C_MAX_STANDARD_MODE_FREQ:
            dev->ss_lcnt = HW_I2C_CLK_L100;
            dev->ss_hcnt = HW_I2C_CLK_H100;
            break;
        case I2C_MAX_FAST_MODE_FREQ:
            dev->fs_lcnt = HW_I2C_CLK_L400;
            dev->fs_hcnt = HW_I2C_CLK_H400;
            break;
        case I2C_MAX_HIGH_SPEED_MODE_FREQ:
            dev->hs_lcnt = HW_I2C_CLK_L3400;
            dev->hs_hcnt = HW_I2C_CLK_H3400;
            dev->fs_lcnt = HW_I2C_CLK_L400;
            dev->fs_hcnt = HW_I2C_CLK_H400;
            break;
        case I2C_MAX_FAST_MODE_PLUS_FREQ:
            dev->fs_lcnt = HW_I2C_CLK_L1000;
            dev->fs_hcnt = HW_I2C_CLK_H1000;
            break;
        default:
            break;
    }

    return 0;
}

static int i2c_dw_calc_timings_master(struct dw_i2c_dev *dev)
{
    ///<  当前所有的时序先按照linux来计算，调试时和硬件对齐
    ///<  1、计算滤波
    ///<  配置滤波计数器。linux使用的为默认值。请少卿在调试exp i2c时，发现要配置为0x14。所以调试hba是要咨询硬件具体值
#if 0
    dev->fs_spklen = 0x14;
    dev->hs_spklen = 0x14;
#endif
    ///<  2、计算scl high & low cnt，需要参考硬件时钟配置接口 TODO
    (void)i2c_dw_calc_sclcnt_master(dev);
    ///<  3、计算sda hold time(参考exp暂时使用默认值)
    ///<  linux在i2c_dw_acpi_configure解析hold time，然后再后续流程中配置到IC_SDA_HOLD寄存器
    ///<  少青提供的demo代码使用的是默认值，所以此处暂时使用默认值，待后续调试时咨询hba硬件
    ///<  sda hold参数改由用户配置
#if 0
    (void)i2c_dw_calc_sda_hold(dev);
#endif
    return 0;
}

/**
 * i2c_dw_init_master() - Initialize the designware I2C master hardware
 * @dev: device private data
 *
 * This functions configures and enables the I2C master.
 * This function is called during I2C init function, and in case of timeout at
 * run time.
 */
static int i2c_dw_init_master(struct dw_i2c_dev *dev)
{
    /* Disable the adapter */
    __i2c_dw_disable(dev);

    ///<  20220921:和硬件白炳成对齐，当前硬件是配置scl时序。spllen和sda hold time都是默认值
#if 0
    dw_reg_write(dev, DW_IC_FS_SPKLEN, dev->fs_spklen);
    dw_reg_write(dev, DW_IC_HS_SPKLEN, dev->hs_spklen);
#endif

#if 1
    /* Write standard speed timing parameters */
    if (dev->ss_hcnt && dev->ss_lcnt) {
        dw_reg_write(dev, DW_IC_SS_SCL_HCNT, dev->ss_hcnt);
        dw_reg_write(dev, DW_IC_SS_SCL_LCNT, dev->ss_lcnt);
    }
    /* Write fast mode/fast mode plus timing parameters */
    if (dev->fs_hcnt && dev->fs_lcnt) {
        dw_reg_write(dev, DW_IC_FS_SCL_HCNT, dev->fs_hcnt);
        dw_reg_write(dev, DW_IC_FS_SCL_LCNT, dev->fs_lcnt);
    }
    /* Write high speed timing parameters if supported */
    if (dev->hs_hcnt && dev->hs_lcnt) {
        dw_reg_write(dev, DW_IC_HS_SCL_HCNT, dev->hs_hcnt);
        dw_reg_write(dev, DW_IC_HS_SCL_LCNT, dev->hs_lcnt);
    }
#else
    /* Write standard speed timing parameters */
    dw_reg_write(dev, DW_IC_SS_SCL_HCNT, dev->ss_hcnt);
    dw_reg_write(dev, DW_IC_SS_SCL_LCNT, dev->ss_lcnt);

    /* Write fast mode/fast mode plus timing parameters */
    dw_reg_write(dev, DW_IC_FS_SCL_HCNT, dev->fs_hcnt);
    dw_reg_write(dev, DW_IC_FS_SCL_LCNT, dev->fs_lcnt);

    /* Write high speed timing parameters if supported */
    if (dev->hs_hcnt && dev->hs_lcnt) {
        dw_reg_write(dev, DW_IC_HS_SCL_HCNT, dev->hs_hcnt);
        dw_reg_write(dev, DW_IC_HS_SCL_LCNT, dev->hs_lcnt);
    }
#endif

    /* Write SDA hold time if supported */
    if (dev->sda_hold_time) {
        dw_reg_write(dev, DW_IC_SDA_HOLD, dev->sda_hold_time);
    }

    i2c_dw_configure_fifo_master(dev);

    ///<  linux未配置置IC_HS_MADDR(0xc)寄存器，exp demo有配置。
    ///<  20220921查看协议手册5.3.2并和硬件白炳成确认：该寄存器默认值为000，高速模式多master场景需要配置
    ///<  当前了解到业务还没有高速模式下多master场景，所以暂时不配置该寄存器

    ///<  master配置完成后，保持disable状态。主动读写时打开，操作完成后又关闭控制器
    return 0;
}

static void i2c_dw_xfer_init(struct dw_i2c_dev *dev)
{
    struct i2c_msg *msgs = dev->msgs;
    U32 ic_con = 0, ic_tar = 0;
    U32 dummy;

    /* Disable the adapter */
    __i2c_dw_disable(dev);

    /* If the slave address is ten bit address, enable 10BITADDR */
    dw_reg_read(dev, DW_IC_CON, &ic_con);
    if (msgs[dev->msg_write_idx].flags & I2C_M_TEN) {
        ic_con |= DW_IC_CON_10BITADDR_MASTER;
        /*
         ** If I2C_DYNAMIC_TAR_UPDATE is set, the 10-bit addressing
         ** mode has to be enabled via bit 12 of IC_TAR register.
         ** We set it always as I2C_DYNAMIC_TAR_UPDATE can't be
         ** detected from registers.
         **/
        ic_tar = DW_IC_TAR_10BITADDR_MASTER;
    } else {
        ic_con &= ~DW_IC_CON_10BITADDR_MASTER;
    }

    dw_reg_write(dev, DW_IC_CON, ic_con);

    /*
     ** Set the slave (target) address and enable 10-bit addressing mode
     ** if applicable.
     **/
    dw_reg_write(dev, DW_IC_TAR, msgs[dev->msg_write_idx].addr | ic_tar);

    /* Enforce disabled interrupts (due to HW issues) */
    dw_reg_write(dev, DW_IC_INTR_MASK, 0);

    /* Enable the adapter */
    __i2c_dw_enable(dev);

    /* Dummy read to avoid the register getting stuck on Bay Trail */
    dw_reg_read(dev, DW_IC_ENABLE_STATUS, &dummy);

    /* Clear and enable interrupts */
    dw_reg_read(dev, DW_IC_CLR_INTR, &dummy);
    dw_reg_write(dev, DW_IC_INTR_MASK, DW_IC_INTR_MASTER_MASK);
}

/*
 * Initiate (and continue) low level master read/write transaction.
 * This function is only called from i2c_dw_isr, and pumping i2c_msg
 * messages into the tx buffer.  Even if the size of i2c_msg data is
 * longer than the size of the tx buffer, it handles everything.
 */
static void i2c_dw_xfer_msg(struct dw_i2c_dev *dev)
{
    struct i2c_msg *msgs = dev->msgs;
    U32 intr_mask;
    int tx_limit, rx_limit;
    U32 addr = msgs[dev->msg_write_idx].addr;
    U32 buf_len = dev->tx_buf_len;
    U8 *buf = dev->tx_buf;
    bool need_restart = false;
    unsigned int flr;

    intr_mask = DW_IC_INTR_MASTER_MASK;

    for (; dev->msg_write_idx < dev->msgs_num; dev->msg_write_idx++) {
        U32 flags = msgs[dev->msg_write_idx].flags;

        LOGT("msg rd id:%d, msg num:%d, msg flags:%hu, dev status:%u.\n", 
                dev->msg_read_idx, dev->msgs_num, msgs[dev->msg_read_idx].flags, dev->status);
        /*
         * If target address has changed, we need to
         * reprogram the target address in the I2C
         * adapter when we are done with this transfer.
         */
        if (msgs[dev->msg_write_idx].addr != addr) {
            LOGE("Invalid target address(origin:%u, new:%u).\n", addr, msgs[dev->msg_write_idx].addr);
            dev->msg_err = -EINVAL;
            break;
        }

        if (!(dev->status & STATUS_WRITE_IN_PROGRESS)) {
            /* new i2c_msg */
            buf = msgs[dev->msg_write_idx].buf;
            buf_len = msgs[dev->msg_write_idx].len;

            /* If both IC_EMPTYFIFO_HOLD_MASTER_EN and
             * IC_RESTART_EN are set, we must manually
             * set restart bit between messages.
             */
            if ((dev->master_cfg & DW_IC_CON_RESTART_EN) &&
                    (dev->msg_write_idx > 0)) {
                need_restart = true;
            }
        }

        dw_reg_read(dev, DW_IC_TXFLR, &flr);
        tx_limit = dev->tx_fifo_depth - flr;

        dw_reg_read(dev, DW_IC_RXFLR, &flr);
        rx_limit = dev->rx_fifo_depth - flr;

        while (buf_len > 0 && tx_limit > 0 && rx_limit > 0) {
            U32 cmd = 0;

            /*
             * If IC_EMPTYFIFO_HOLD_MASTER_EN is set we must
             * manually set the stop bit. However, it cannot be
             * detected from the registers so we set it always
             * when writing/reading the last byte.
             */

            /*
             * i2c-core always sets the buffer length of
             * I2C_FUNC_SMBUS_BLOCK_DATA to 1. The length will
             * be adjusted when receiving the first byte.
             * Thus we can't stop the transaction here.
             */

			 if (((dev->msg_write_idx == dev->msgs_num - 1)||(flags&I2C_M_STOP)) &&
                    buf_len == 1 && !(flags & I2C_M_RECV_LEN)) {
                cmd |= BIT(9);
            }

            if (need_restart) {
                cmd |= BIT(10);
                need_restart = false;
            }

            LOGT("tx limit:%d, rx limit:%d, buf len:%u, flags:%hu, cmd:%u, rx outstanding:%d.\n", 
                    tx_limit, rx_limit, buf_len, flags, cmd, dev->rx_outstanding);
            if (msgs[dev->msg_write_idx].flags & I2C_M_RD) {

                /* Avoid rx buffer overrun */
                if (dev->rx_outstanding >= dev->rx_fifo_depth) {
                    break;
                }

                dw_reg_write(dev, DW_IC_DATA_CMD, cmd | 0x100);
                rx_limit--;
                dev->rx_outstanding++;
            } else {
                dw_reg_write(dev, DW_IC_DATA_CMD, cmd | *buf++);
            }
            tx_limit--; buf_len--;
            mdelay(5);
        }

        dev->tx_buf = buf;
        dev->tx_buf_len = buf_len;

        /*
         * Because we don't know the buffer length in the
         * I2C_FUNC_SMBUS_BLOCK_DATA case, we can't stop the
         * transaction here. Also disable the TX_EMPTY IRQ
         * while waiting for the data length byte to avoid the
         * bogus interrupts flood.
         */
        if (flags & I2C_M_RECV_LEN) {
            dev->status |= STATUS_WRITE_IN_PROGRESS;
            intr_mask &= ~DW_IC_INTR_TX_EMPTY;
            break;
        }
        else if (buf_len > 0) {
            /* more bytes to be written */
            dev->status |= STATUS_WRITE_IN_PROGRESS;
            break;
        } else
            dev->status &= ~STATUS_WRITE_IN_PROGRESS;
    }

    /*
     * If i2c_msg index search is completed, we don't need TX_EMPTY
     * interrupt any more.
     */
    if (dev->msg_write_idx == dev->msgs_num) {
        intr_mask &= ~DW_IC_INTR_TX_EMPTY;
    }

    if (dev->msg_err) {
        intr_mask = 0;
    }

    dw_reg_write(dev,  DW_IC_INTR_MASK, intr_mask);
}

static U8 i2c_dw_recv_len(struct dw_i2c_dev *dev, U8 len)
{
    struct i2c_msg *msgs = dev->msgs;
    U32 flags = msgs[dev->msg_read_idx].flags;
    unsigned int intr_mask;

    /*
     * Adjust the buffer length and mask the flag
     * after receiving the first byte.
     */
    len += (flags & I2C_CLIENT_PEC) ? 2 : 1;
    dev->tx_buf_len = len - min(len, dev->rx_outstanding);
    msgs[dev->msg_read_idx].len = len;
    msgs[dev->msg_read_idx].flags &= ~I2C_M_RECV_LEN;

    /*
     * Received buffer length, re-enable TX_EMPTY interrupt
     * to resume the SMBUS transaction.
     */
    dw_reg_read(dev, DW_IC_INTR_MASK, &intr_mask);
    intr_mask |= DW_IC_INTR_TX_EMPTY;
    dw_reg_write(dev, DW_IC_INTR_MASK, intr_mask);

    return len;
}

static void i2c_dw_read(struct dw_i2c_dev *dev)
{
    struct i2c_msg *msgs = dev->msgs;
    unsigned int rx_valid;

    for (; dev->msg_read_idx < dev->msgs_num; dev->msg_read_idx++) {
        U32 len, tmp;
        U8 *buf;

        LOGT("msg rd id:%d, msg num:%d, msg flags:%hu, dev status:%u.\n", 
                dev->msg_read_idx, dev->msgs_num, msgs[dev->msg_read_idx].flags, dev->status);
        if (!(msgs[dev->msg_read_idx].flags & I2C_M_RD))
            continue;

        if (!(dev->status & STATUS_READ_IN_PROGRESS)) {
            len = msgs[dev->msg_read_idx].len;
            buf = msgs[dev->msg_read_idx].buf;
        } else {
            len = dev->rx_buf_len;
            buf = dev->rx_buf;
        }

        dw_reg_read(dev, DW_IC_RXFLR, &rx_valid);

        for (; len > 0 && rx_valid > 0; len--, rx_valid--) {
            U32 flags = msgs[dev->msg_read_idx].flags;

            LOGT("len:%u, rx valid:%u, flags:%hu.\n", len, rx_valid, flags);
            dw_reg_read(dev, DW_IC_DATA_CMD, &tmp);
            tmp &= DW_IC_DATA_CMD_DAT;
            /* Ensure length byte is a valid value */
            if (flags & I2C_M_RECV_LEN) {
                /*
                 * if IC_EMPTYFIFO_HOLD_MASTER_EN is set, which cannot be
                 * detected from the registers, the controller can be
                 * disabled if the STOP bit is set. But it is only set
                 * after receiving block data response length in
                 * I2C_FUNC_SMBUS_BLOCK_DATA case. That needs to read
                 * another byte with STOP bit set when the block data
                 * response length is invalid to complete the transaction.
                 */
                if (!tmp || tmp > I2C_SMBUS_BLOCK_MAX)
                    tmp = 1;

                len = i2c_dw_recv_len(dev, tmp);
            }
            *buf++ = tmp;
            dev->rx_outstanding--;
        }

        if (len > 0) {
            dev->status |= STATUS_READ_IN_PROGRESS;
            dev->rx_buf_len = len;
            dev->rx_buf = buf;
            return;
        } else
            dev->status &= ~STATUS_READ_IN_PROGRESS;
    }
}

/*
 * This function waits for the controller to be idle before disabling I2C
 * When the controller is not in the IDLE state, the MST_ACTIVITY bit
 * (IC_STATUS[5]) is set.
 *
 * Values:
 * 0x1 (ACTIVE): Controller not idle
 * 0x0 (IDLE): Controller is idle
 *
 * The function is called after completing the current transfer.
 *
 * Returns:
 * False when the controller is in the IDLE state.
 * True when the controller is in the ACTIVE state.
 */
static bool i2c_dw_is_controller_active(struct dw_i2c_dev *dev)
{
    U32 status;
    ///<  等待设备空闲，约40ms。linux此处是2ms
    int timeout = 2;

    dw_reg_read(dev, DW_IC_STATUS, &status);
    while (status & DW_IC_STATUS_MASTER_ACTIVITY) {
        if (timeout <= 0) {
            return -ETIMEDOUT;
        }
        timeout--;
        rtems_task_wake_after(2);
        dw_reg_read(dev, DW_IC_STATUS, &status);
    }
    return false;
}

static int i2c_dw_check_stopbit(struct dw_i2c_dev *dev)
{
	U32 val;
    int timeout = 2;

    dw_reg_read(dev, DW_IC_RAW_INTR_STAT, &val);
    while (!(val & DW_IC_INTR_STOP_DET)) {
        if (timeout <= 0) {
            LOGE("Timeout waiting for i2c-%u stop.\n", dev->i2c_channel_num);
            return -ETIMEDOUT;
        }
        timeout--;
        rtems_task_wake_after(2);
        dw_reg_read(dev, DW_IC_RAW_INTR_STAT, &val);
    }
    return 0;
}

static int i2c_dw_check_txready(struct dw_i2c_dev *dev)
{
	U32 val;
    int timeout = 2;
    dw_reg_read(dev, DW_IC_STATUS, &val);
    ///<  tx full 需要等
    while ((val & 0x2) == 0) {
        if (timeout <= 0) {
            LOGE("Timeout waiting for i2c-%u tx ready.\n", dev->i2c_channel_num);
            return -ETIMEDOUT;
        }
        timeout--;
        rtems_task_wake_after(2);
        dw_reg_read(dev, DW_IC_STATUS, &val);
    }
    return 0;
}

static int i2c_dw_check_rxready(struct dw_i2c_dev *dev)
{
	U32 val;
    int timeout = 2;
    dw_reg_read(dev, DW_IC_STATUS, &val);
    ///<  rx empty 需要等
    while ((val & 0x8) == 0) {
        if (timeout <= 0) {
            LOGE("Timeout waiting for i2c-%u tx ready.\n", dev->i2c_channel_num);
            return -ETIMEDOUT;
        }
        timeout--;
        rtems_task_wake_after(2);
        dw_reg_read(dev, DW_IC_STATUS, &val);
    }
    return 0;
}

static int i2c_dw_check_err(struct dw_i2c_dev *dev)
{
    U32 stat, errcode, dummy;
    dw_reg_read(dev, DW_IC_RAW_INTR_STAT, &stat);
    if (stat & DW_IC_INTR_TX_ABRT) {
        dw_reg_read(dev, DW_IC_TX_ABRT_SOURCE, &errcode);
        dw_reg_read(dev, DW_IC_CLR_TX_ABRT, &dummy);
        LOGE("Transfer abort(0x%08x).\n", errcode); 
        return -EIO;
    } else {
        return 0;
    }
}

static int i2c_dw_xfer_poll(struct dw_i2c_dev *dev, struct i2c_msg msgs[], int num)
{
    int msg_wrt_idx, msg_itr_lmt, buf_len;
    int cmd = 0, status;
    U8 *buf;
    U32 val;
    int j = 0;

    dev->msgs = msgs;
    dev->msgs_num = num;
    i2c_dw_xfer_init(dev);
    dw_reg_write(dev, DW_IC_INTR_MASK, 0);

    /* Initiate messages read/write transaction */
    for (msg_wrt_idx = 0; msg_wrt_idx < num; msg_wrt_idx++) {
        buf = msgs[msg_wrt_idx].buf;
        buf_len = msgs[msg_wrt_idx].len;

        for (msg_itr_lmt = buf_len; msg_itr_lmt > 0; msg_itr_lmt--) {
            if (msg_wrt_idx == num - 1 && msg_itr_lmt == 1) {
                cmd |= BIT(9);
            }
            if (msgs[msg_wrt_idx].flags & I2C_M_RD) {
                status = i2c_dw_check_txready(dev);
                if (status) {
                    LOGE("Check tx ready timeout(%d).\n", status);
                    return status;
                }
                dw_reg_write(dev, DW_IC_DATA_CMD, 0x100 | cmd);
                j = 0;
                while(j < CHECK_ERR_TIMEOUT_COUNT) {
                    status = i2c_dw_check_err(dev);
                    if(status == 0) { 
                        j++;
						break;
                    } else {
                        LOGE("Check err(%d).\n", status);
                        return status;
                    }
                }
                status = i2c_dw_check_rxready(dev);
                if (status) {
                    LOGE("Check rx ready timeout(%d).\n", status);
                    return status;
                }
                dw_reg_read(dev, DW_IC_DATA_CMD, &val);
                while(j < CHECK_ERR_TIMEOUT_COUNT) {
                    status = i2c_dw_check_err(dev);
                    if(status == 0) { 
                        j++;
                    } else {
                        LOGE("Check err(%d).\n", status);
                        return status;
                    }
                }
                *buf++ = val;
            } else {
                status = i2c_dw_check_txready(dev);
                if (status) {
                    LOGE("Check tx ready timeout(%d).\n", status);
                    return status;
                }
                dw_reg_write(dev, DW_IC_DATA_CMD, *buf++ | cmd);
                j = 0;
                while(j < CHECK_ERR_TIMEOUT_COUNT) {
                    status = i2c_dw_check_err(dev);
                    if(status == 0) { 
                        j++;
                        break;
                    } else {
                        LOGE("Check err(%d).\n", status);
                        return status;
                    }
                }
            }
        }
        status = i2c_dw_check_stopbit(dev);
        if (status) {
            return status;
        }
    }
    return 0;
}

/*
 * Prepare controller for a transaction and call i2c_dw_xfer_msg.
 */
int i2c_dw_xfer(struct dw_i2c_dev *dev, struct i2c_msg msgs[], int num)
{
    int ret = 0;

    LOGT("msg: %d.\n", num);

    ///<  轮询模式
    if (dev->work_mode == 1) {
        ret = i2c_dw_xfer_poll(dev, msgs, num);
        goto done;
    }

    while(RTEMS_SUCCESSFUL == rtems_semaphore_obtain(dev->semaphore_id, RTEMS_NO_WAIT, 0)) {
        ;
    }

    dev->msgs = msgs;
    dev->msgs_num = num;
    dev->cmd_err = 0;
    dev->msg_write_idx = 0;
    dev->msg_read_idx = 0;
    dev->msg_err = 0;
    dev->status = 0;
    dev->abort_source = 0;
    dev->rx_outstanding = 0;

    ret = i2c_dw_wait_bus_not_busy(dev);
    if (ret < 0) {
        goto done;
    }

    /* Start the transfers */
    i2c_dw_xfer_init(dev);

    /* Wait for tx to complete */
    if(rtems_semaphore_obtain(dev->semaphore_id, RTEMS_WAIT, 
                RTEMS_MILLISECONDS_TO_TICKS(dev->transfer_timeout)) != RTEMS_SUCCESSFUL) {
        LOGE("I2c controller timed out.\n");
        i2c_dw_init_master(dev);
        ret = -ETIMEDOUT;
        goto done;
    }

    /*
     * This happens rarely (~1:500) and is hard to reproduce. Debug trace
     * showed that IC_STATUS had value of 0x23 when STOP_DET occurred,
     * if disable IC_ENABLE.ENABLE immediately that can result in
     * IC_RAW_INTR_STAT.MASTER_ON_HOLD holding SCL low. Check if
     * controller is still ACTIVE before disabling I2C.
     */
    if (i2c_dw_is_controller_active(dev))
        LOGE("controller(%d) active\n", dev->i2c_channel_num);

    /*
     * We must disable the adapter before returning and signaling the end
     * of the current transfer. Otherwise the hardware might continue
     * generating interrupts which in turn causes a race condition with
     * the following transfer.  Needs some more investigation if the
     * additional interrupts are a hardware bug or this driver doesn't
     * handle them correctly yet.
     */
    __i2c_dw_disable_nowait(dev);

    if (dev->msg_err) {
        ret = dev->msg_err;
        goto done;
    }

    /* No error */
    if (!dev->cmd_err && !dev->status) {
        ret = num;
        goto done;
    }

    /* We have an error */
    if (dev->cmd_err == DW_IC_ERR_TX_ABRT) {
        ret = i2c_dw_handle_tx_abort(dev);
        goto done;
    }

    if (dev->status) {
        LOGE("transfer terminated early - interrupt latency too high?\n");
    }

    ret = -EFAULT;

done:
    return ret;
}

static U32 i2c_dw_read_clear_intrbits(struct dw_i2c_dev *dev)
{
    U32 stat, dummy;

    /*
     * The IC_INTR_STAT register just indicates "enabled" interrupts.
     * The unmasked raw version of interrupt status bits is available
     * in the IC_RAW_INTR_STAT register.
     *
     * That is,
     *   stat = readl(IC_INTR_STAT);
     * equals to,
     *   stat = readl(IC_RAW_INTR_STAT) & readl(IC_INTR_MASK);
     *
     * The raw version might be useful for debugging purposes.
     */
    dw_reg_read(dev, DW_IC_INTR_STAT, &stat);

    /*
     * Do not use the IC_CLR_INTR register to clear interrupts, or
     * you'll miss some interrupts, triggered during the period from
     * readl(IC_INTR_STAT) to readl(IC_CLR_INTR).
     *
     * Instead, use the separately-prepared IC_CLR_* registers.
     */
    if (stat & DW_IC_INTR_RX_UNDER)
        dw_reg_read(dev, DW_IC_CLR_RX_UNDER, &dummy);
    if (stat & DW_IC_INTR_RX_OVER)
        dw_reg_read(dev, DW_IC_CLR_RX_OVER, &dummy);
    if (stat & DW_IC_INTR_TX_OVER)
        dw_reg_read(dev, DW_IC_CLR_TX_OVER, &dummy);
    if (stat & DW_IC_INTR_RD_REQ)
        dw_reg_read(dev, DW_IC_CLR_RD_REQ, &dummy);
    if (stat & DW_IC_INTR_TX_ABRT) {
        /*
         * The IC_TX_ABRT_SOURCE register is cleared whenever
         * the IC_CLR_TX_ABRT is read.  Preserve it beforehand.
         */
        dw_reg_read(dev, DW_IC_TX_ABRT_SOURCE, &dev->abort_source);
        dw_reg_read(dev, DW_IC_CLR_TX_ABRT, &dummy);
    }
    if (stat & DW_IC_INTR_RX_DONE)
        dw_reg_read(dev, DW_IC_CLR_RX_DONE, &dummy);
    if (stat & DW_IC_INTR_ACTIVITY)
        dw_reg_read(dev, DW_IC_CLR_ACTIVITY, &dummy);
    if ((stat & DW_IC_INTR_STOP_DET) &&
            ((dev->rx_outstanding == 0) || (stat & DW_IC_INTR_RX_FULL)))
        dw_reg_read(dev, DW_IC_CLR_STOP_DET, &dummy);
    if (stat & DW_IC_INTR_START_DET)
        dw_reg_read(dev, DW_IC_CLR_START_DET, &dummy);
    if (stat & DW_IC_INTR_GEN_CALL)
        dw_reg_read(dev, DW_IC_CLR_GEN_CALL, &dummy);

    return stat;
}

__attribute__((optimize(0)))
static void dwI2cMstArbitrationHandle(struct dw_i2c_dev *dev)
{
    U32 arbState = 0;
    U32 intrState = 0;
    U32 mask = 0;
    U32 clr = 0;

    dw_reg_read(dev,DW_IC_INTR_STAT,&intrState);
    intrState = intrState & (IC_INTR_STAT_TX_ABRT);
    if (!intrState)
    {
        return;
    }

    if (dev->arbCb == NULL) {
        dw_reg_read(dev,DW_IC_CLR_TX_ABRT,&clr);
        clr = clr;
        return;
    }

    ///< 关仲裁失败中断
    dw_reg_read(dev,DW_IC_INTR_MASK,&mask);
    mask = mask & (~0x40);
    dw_reg_write(dev,DW_IC_INTR_MASK,mask);

    dw_reg_read(dev,DW_IC_TX_ABRT_SOURCE,&arbState);
    if (arbState & IC_TX_ABRT_SLV_ARBLOST)
    {
        dev->arbCb(dev->arbCbParam);
    }

    ///< 先清仲裁失败中断（读清）
    dw_reg_read(dev,DW_IC_CLR_TX_ABRT,&clr);
    clr = clr;

    ///< 开中断
    dw_reg_read(dev,DW_IC_INTR_MASK,&mask);
    mask = mask | 0x40;
    dw_reg_write(dev,DW_IC_INTR_MASK,mask);
}


/*
 * Interrupt service routine. This gets called whenever an I2C master interrupt
 * occurs.
 */
static void i2c_dw_isr(void *dev_id)
{
    struct dw_i2c_dev *dev = dev_id;
    U32 stat, enabled;

    dw_reg_read(dev, DW_IC_ENABLE, &enabled);
    dw_reg_read(dev, DW_IC_RAW_INTR_STAT, &stat);
    if (!enabled || !(stat & ~DW_IC_INTR_ACTIVITY) || (stat == GENMASK(31, 0))) {
        LOGE("enabled=%#x stat=%#x.\n", enabled, stat);
        return;
    }

    dwI2cMstArbitrationHandle(dev);     
    
    stat = i2c_dw_read_clear_intrbits(dev);

    if (!(dev->status & STATUS_ACTIVE)) {                            
        /*
         ** Unexpected interrupt in driver point of view. State       
         ** variables are either unset or stale so acknowledge and    
         ** disable interrupts for suppressing further interrupts if  
         ** interrupt really came from this HW (E.g. firmware has left
         ** the HW active).                                           
         **/                                                          
        dw_reg_write(dev, DW_IC_INTR_MASK, 0);                  
        return;                                          
    }                                                           

    if (stat & DW_IC_INTR_TX_ABRT) {
        dev->cmd_err |= DW_IC_ERR_TX_ABRT;
        dev->status &= ~STATUS_MASK;
        dev->rx_outstanding = 0;

        /*
         * Anytime TX_ABRT is set, the contents of the tx/rx
         * buffers are flushed. Make sure to skip them.
         */
        dw_reg_write(dev, DW_IC_INTR_MASK, 0);
        goto tx_aborted;
    }

    if (stat & DW_IC_INTR_RX_FULL) {
        i2c_dw_read(dev);
    }

    if (stat & DW_IC_INTR_TX_EMPTY) {
        i2c_dw_xfer_msg(dev);
    }

    /*
     * No need to modify or disable the interrupt mask here.
     * i2c_dw_xfer_msg() will take care of it according to
     * the current transmit status.
     */

tx_aborted:
    if (((stat & (DW_IC_INTR_TX_ABRT | DW_IC_INTR_STOP_DET)) || dev->msg_err) &&
            (dev->rx_outstanding == 0)) {
        rtems_semaphore_release(dev->semaphore_id);
    }

    return;
}

void i2c_dw_configure_master(struct dw_i2c_dev *dev)
{
    struct i2c_timings *t = &dev->timings;

    ///<  默认master为7bit模式，未配置DW_IC_CON_10BITADDR_MASTER	
    dev->master_cfg = DW_IC_CON_MASTER | DW_IC_CON_SLAVE_DISABLE |
        DW_IC_CON_RESTART_EN;
    if (dev->addr_mode == I2C_10BIT_ADDRESS) {
        dev->master_cfg |= DW_IC_CON_10BITADDR_MASTER;
    }

    dev->mode = DW_IC_MASTER;

    switch (t->bus_freq_hz) {
        case I2C_MAX_STANDARD_MODE_FREQ:
            dev->master_cfg |= DW_IC_CON_SPEED_STD;
            break;
        case I2C_MAX_HIGH_SPEED_MODE_FREQ:
            dev->master_cfg |= DW_IC_CON_SPEED_HIGH;
            break;
        default:
            dev->master_cfg |= DW_IC_CON_SPEED_FAST;
    }
}

int i2c_dw_probe_master(struct dw_i2c_dev *dev)
{
    int ret = 0;
    rtems_status_code status;
    U32 ic_con;

    ret = i2c_dw_calc_timings_master(dev);
    if (ret) {
        return ret;
    }
    ret = i2c_dw_calc_fifo_size(dev);
    if (ret) {
        return ret;
    }
    /*
     * On AMD platforms BIOS advertises the bus clear feature
     * and enables the SCL/SDA stuck low. SMU FW does the
     * bus recovery process. Driver should not ignore this BIOS
     * advertisement of bus clear feature.
     */
    dw_reg_read(dev, DW_IC_CON, &ic_con);
    if (ic_con & DW_IC_CON_BUS_CLEAR_CTRL)
        dev->master_cfg |= DW_IC_CON_BUS_CLEAR_CTRL;
    
    ret = i2c_dw_init_master(dev);
    if (ret) {
        return ret;
    }
    dw_reg_write(dev, DW_IC_INTR_MASK, 0);
    ///<  中断模式
    if (dev->work_mode == 0) {
        status = rtems_semaphore_create(rtems_build_name('I', '2', 'C', '0' + dev->i2c_channel_num), 0,
                RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_FIFO, 0, &(dev->semaphore_id));
        if (status != RTEMS_SUCCESSFUL) {
            LOGE("Create sem for i2c-%u faile(%u).\n", dev->i2c_channel_num, status);
            ret = -EPERM;
            return ret;
        }
        snprintf(dev->i2c_name, I2C_NAME_LEN, "I2%2u", dev->i2c_channel_num % 100);
        dev->i2c_name[I2C_NAME_LEN - 1] = 0;
        status = rtems_interrupt_handler_install(dev->irq, dev->i2c_name, 
                RTEMS_INTERRUPT_UNIQUE, i2c_dw_isr, dev);
        if (status != RTEMS_SUCCESSFUL) {
            rtems_semaphore_delete(dev->semaphore_id);
            LOGE("Failure requesting irq %u: %u\n", dev->irq, status);
            ret = -EPERM;
            return ret;
        }
    }
    return ret;
}

int i2c_dw_unprobe_master(struct dw_i2c_dev *dev)
{
    rtems_status_code status;
    i2c_dw_disable(dev);
    dev->master_cfg = 0;
    if (dev->work_mode == 0) {
        status = rtems_interrupt_handler_remove(dev->irq, i2c_dw_isr, dev);
        if (status != RTEMS_SUCCESSFUL) {
            LOGE("Remove irq %u: %u.\n", dev->irq, status);
        }
        status = rtems_semaphore_delete(dev->semaphore_id);
        if (status != RTEMS_SUCCESSFUL) {
            LOGE("Delate semaphore %u: %u.\n", dev->semaphore_id, status);
        }
    }
    return 0;
}

static uint8_t crc8(uint16_t data)
{
    int32_t i;

    for (i = 0; i < 8; i++) {
        if (data & 0x8000) {
            data = data ^ CRC8_POLY;
        }
        data = data << 1;
    }
    return (uint8_t)(data >> 8);
}

static uint8_t smbusPecCalc(uint8_t crc, uint8_t *p, uint32_t count)
{
    int32_t i;

    for (i = 0; i < count; i++) {
        crc = crc8((crc ^ p[i]) << 8);
    }
    return crc;
}

///<  only for 7bit
static uint8_t i2cAddrConvert(uint8_t addrIn, bool isWrite)
{
    return (addrIn << 1) | (isWrite ? 0 : 1);
}

static uint8_t smbusPecPktConstruct(uint8_t addr7bitIn, bool isWrite, uint8_t *pData, uint32_t count)
{
    uint8_t addr = i2cAddrConvert(addr7bitIn, isWrite);
    uint8_t addrWithPec = smbusPecCalc(0, &addr, 1);

    return smbusPecCalc(addrWithPec, pData, count);
}


static inline void dwI2cDisable(struct dw_i2c_dev *dev)
{
    U32 tmp;

    dw_reg_read(dev,DW_IC_ENABLE,&tmp);
    tmp &= ~(1);
    dw_reg_write(dev,DW_IC_ENABLE,tmp);
}

static int32_t smbusArpPrepare(struct dw_i2c_dev *dev)
{
    int32_t ret = EXIT_SUCCESS;
    uint8_t data_buf[1];
    uint8_t defaultAddr = SMBUS_ARP_DEFAULT_ADDR;
    uint32_t arpPrepareData = 0;
    uint32_t pecData = 0;
    uint32_t status;

    arpPrepareData = ((SMBUS_CMD_PREPARE_4_ARP)&0xff)|(0<<8);
    if (i2c_dw_check_txready(dev) != 0) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    dw_reg_write(dev,DW_IC_DATA_CMD,arpPrepareData);
    
    ///< < Step 2: Fill PEC 
    data_buf[0] = SMBUS_CMD_PREPARE_4_ARP;
    pecData = smbusPecPktConstruct(defaultAddr, true, data_buf, 1);
    pecData = (pecData&0xff)|(0<<8)|(1<<9);
    if (i2c_dw_check_txready(dev) != 0) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    dw_reg_write(dev,DW_IC_DATA_CMD,pecData);

    ///< < Wait for Slave ACK or timeout ?
    udelay(10000);
    dw_reg_read(dev,DW_IC_TX_ABRT_SOURCE,&status);
    if ((status&0x9) != 0) {
        ///< < noack 
        ret = EXIT_FAILURE;
    }
exit:
    return ret;
}

static int32_t smbusArpResetDevice(struct dw_i2c_dev *dev)
{
    int32_t ret = EXIT_SUCCESS;
    uint8_t data_buf[1];
    uint8_t defaultAddr = SMBUS_ARP_DEFAULT_ADDR;
    uint32_t arpResetData = 0;
    uint32_t pecData = 0;
    uint32_t status;

    ///< < Step 1: Send reset device command
    arpResetData = ((SMBUS_CMD_GENERAL_RESET_DEVICE)&0xff)|(0<<8);
    if (i2c_dw_check_txready(dev) != 0) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    dw_reg_write(dev,DW_IC_DATA_CMD,arpResetData);

    ///< <  Step 2: Fill PEC 
    data_buf[0] = SMBUS_CMD_GENERAL_RESET_DEVICE;
    pecData = smbusPecPktConstruct(defaultAddr, true, data_buf, 1);
    pecData = (pecData&0xff)|(0<<8)|(1<<9);
    if (i2c_dw_check_txready(dev) != 0) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    dw_reg_write(dev,DW_IC_DATA_CMD,pecData);

    ///< <  Wait for Slave ACK or timeout
    udelay(1000);
    dw_reg_read(dev,DW_IC_TX_ABRT_SOURCE,&status);
    if ((status&0x9) != 0) {
        ///< < noack 
        ret = EXIT_FAILURE;
    }
exit:
    return ret;
}

static int32_t smbusUdidGet(struct dw_i2c_dev *dev, uint32_t *udid_word0, uint32_t timeout)
{
    int32_t ret = EXIT_SUCCESS;
    uint32_t i;
    uint8_t buffer[19];
    uint8_t data_buf[20];
    uint32_t cmd_cnt = 0;
    uint8_t pec = 0;
    uint32_t data_index = 0;
    uint32_t size = 19;
    uint8_t default_addr = SMBUS_ARP_DEFAULT_ADDR;
    uint32_t udidGetCmd = 0;
    U32 data;

    /* pec caculate */
    memset(buffer, 0, sizeof(buffer));
    memset(data_buf, 0, sizeof(data_buf));
    /* may to modify */
    /* Step 1: Send Get UDID command */
    udidGetCmd = ((SMBUS_CMD_GENERAL_GET_UDID)&0xff)|(0<<8);
    if (i2c_dw_check_txready(dev) != 0) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    dw_reg_write(dev,DW_IC_DATA_CMD,udidGetCmd);

    /* update pec */
    data_buf[cmd_cnt] = udidGetCmd;
    pec = smbusPecPktConstruct(default_addr, true, &data_buf[cmd_cnt], 1);
    pec = smbusPecCalc(pec, &pec, 1);
    cmd_cnt++;
    /* Step 2: Read 18 times for GUID */
    for (i = 0; i < 18; i++) {
        udidGetCmd = (1<<8)|(0<<9);
        if (i2c_dw_check_txready(dev) != 0) {
            ret = EXIT_FAILURE;
            goto exit;
        }
        dw_reg_write(dev,DW_IC_DATA_CMD,udidGetCmd);

        /* update pec */
        data_buf[cmd_cnt] = 0;
        pec = smbusPecPktConstruct(default_addr, true, &data_buf[cmd_cnt], 1);
        pec = smbusPecCalc(pec, &pec, 1);
        cmd_cnt++;
    }
    /* Step 3: Write PEC */
    if (i2c_dw_check_txready(dev) != 0) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    dw_reg_write(dev,DW_IC_DATA_CMD,(pec&0xff)|(1<<8)|(1<<9));

    udelay(10000);
    /* Step 4: read back all data */
    while (data_index < size) {
        /* Read out the resulting received data as they come in. */
        ///<  if (EXIT_SUCCESS != stars_i2c_rx_fifo_level_get(smbusMasterId, &level)) {
        ///<      ret = EXIT_FAILURE;
        ///<      break;
        ///<  }
        ///<  if (0 == level) {
        ///<      if (timeout-- == 0) {
        ///<          ret = EXIT_FAILURE;
        ///<          break;
        ///<      }
        ///<      continue;
        ///<  }
        if (i2c_dw_check_rxready(dev) != 0) {
            ret = EXIT_FAILURE;
            goto exit;
        }
        dw_reg_read(dev,DW_IC_DATA_CMD,&data);
        buffer[data_index++] = data;
        ///<  if (dwI2cGetRxflr(i2cCtrPtr->dwI2cRegs) != 0)
        ///<      buffer[data_index++] = i2cCtrPtr->dwI2cRegs->IC_DATA_CMD;
    }

    memcpy(udid_word0, &buffer[1], sizeof(uint32_t) * 4);
exit:
    return ret;
}

static int32_t smbusArpAddressAssign(struct dw_i2c_dev *dev,uint32_t *udid_word0,uint8_t assign_addr)
{
    int32_t ret = EXIT_SUCCESS;
    uint32_t i;
    uint8_t *buffer = (uint8_t *)udid_word0;
    uint8_t data_buf[19];
    uint8_t pec = 0;
    uint8_t default_addr = SMBUS_ARP_DEFAULT_ADDR;
    uint32_t index = 0;
    uint32_t assignAddrCmd = 0;

    /* Step 1: Send Assign Address command */
    assignAddrCmd = ((SMBUS_CMD_GENERAL_ASSIGN_ADDR)&0xff)|(0<<8);
    if (i2c_dw_check_txready(dev) != 0) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    dw_reg_write(dev,DW_IC_DATA_CMD,assignAddrCmd);

    data_buf[index++] = (SMBUS_CMD_GENERAL_ASSIGN_ADDR);

    /* Step 2: Write Byte Count = 17 */
    assignAddrCmd = ((17)&0xff)|(0<<8);
    if (i2c_dw_check_txready(dev) != 0) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    dw_reg_write(dev,DW_IC_DATA_CMD,assignAddrCmd);
    data_buf[index++] = 17;

    /* Step 3: Write UDID Byte 15 to 0 */
    for (i = 0; i < 16; i++) {
        if (i2c_dw_check_txready(dev) != 0) {
            ret = EXIT_FAILURE;
            goto exit;
        }
        dw_reg_write(dev,DW_IC_DATA_CMD,(buffer[i]&0xff)|(0<<8));
        data_buf[index++] = buffer[i];
    }

    /* Step 4: Write Assign Adress */
    assignAddrCmd = (((assign_addr << 1)|0x01)&0xff)|(0<<8);
    if (i2c_dw_check_txready(dev) != 0) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    dw_reg_write(dev,DW_IC_DATA_CMD,assignAddrCmd);
    data_buf[index++] = (((assign_addr << 1)|0x01)&0xff);

    /* pec caculate */
    pec = smbusPecPktConstruct(default_addr, true, data_buf, index);
    /* Step 5: Write PEC */
    assignAddrCmd = pec;
    if (i2c_dw_check_txready(dev) != 0) {
        ret = EXIT_FAILURE;
        goto exit;
    }
    dw_reg_write(dev,DW_IC_DATA_CMD,assignAddrCmd|(0<<8)|(1<<9));

exit:
    return ret;
}

int32_t smbusDevAddrAssign(struct dw_i2c_dev *dev, uint8_t assign_addr)
{
    int32_t ret = EXIT_SUCCESS;
    uint32_t default_addr = SMBUS_ARP_DEFAULT_ADDR;
    uint32_t udid_word[4] = { 0 };
    uint32_t timeout;
    U32 ic_con;

    /* Step 1: Set IC_TAR as default  */
    dwI2cDisable(dev);
    
    dw_reg_read(dev,DW_IC_CON,&ic_con);
    ic_con &= ~DW_IC_CON_10BITADDR_MASTER;  ///<  ???????????
    dw_reg_write(dev, DW_IC_CON, ic_con);
    default_addr = default_addr&(0x7F);
    dw_reg_write(dev,DW_IC_TAR,default_addr);
    __i2c_dw_enable(dev);
	///<  耦合了太多业务代码，规避业务代码引入的逻辑问题
    dw_reg_write(dev, DW_IC_INTR_MASK, 0);

    ///<  dw_reg_write(dev, DW_IC_TAR, default_addr);


    /* Step 2: Prepare for ARP */
    if (EXIT_SUCCESS != smbusArpPrepare(dev)) {
        /* not get ack or get nack, arp exit */
        ret = EXIT_FAILURE;
        ///<  LOGT("smbusArpPrepare failed! \r\n");
        goto exit;
    }
    ///<  LOGT("arp prepare done \r\n");
    /* Optional: Reset device */
    if (EXIT_SUCCESS != smbusArpResetDevice(dev)) {
        /* not get ack or get nack, arp exit */
        ret = EXIT_FAILURE;
        ///<  LOGT("smbusArpResetDevice failed! \r\n");
        goto exit;
    }

    /* Step 3: Now get ack from slave, Get UDID */
    timeout = 19000; /* size * 1000, and size = 19,  */
    if (EXIT_SUCCESS != smbusUdidGet(dev, udid_word, timeout)) {
        ///<  LOGT("%s(): stars_smbus_udid_get() failed, timeout = %d\r\n",__func__);
        ret = EXIT_FAILURE;
        goto exit;
    }

    ///<  LOGT("%x %x %x %x \r\n",udid_word[0],udid_word[1],udid_word[2],udid_word[3]);
    /* Step 4: Assign Address */
    if (EXIT_SUCCESS != smbusArpAddressAssign(dev, udid_word, assign_addr)) {
        ///<  LOGT("%s(): stars_smbus_address_assign() failed\r\n",__func__);
        ret = EXIT_FAILURE;
        goto exit;
    }

exit:
    return ret;
}

///<  DEV改配为master or slave后必须重新配置设备，例如中断
S32 smbusMasterSlaveModeSwitch(struct dw_i2c_dev *dev, DwI2cMode_t i2cMode)
{
    U32 speed_val;
    U32 ti2c_poll;
    U32 enable_sts;
    U32 max_t_poll_count;
    U32 i;
    U32 val;
    rtems_interval start_ticks = rtems_clock_get_ticks_since_boot();
    rtems_interval timeout_ticks = rtems_clock_get_ticks_per_second(); //1s
    rtems_interval current_ticks;

    ///<  wait for i2c idle
    do {
        dw_reg_read(dev,DW_IC_STATUS,&val);
        if ((val & 1) == 0) {
            break;
        }

        current_ticks = rtems_clock_get_ticks_since_boot();
        if (current_ticks - start_ticks > timeout_ticks) {
            LOGT("i2cSwitchMode timeout.\n");
            return EXIT_FAILURE;
        }

        rtems_task_wake_after(1);
    } while (1);

    ///<  根据当前的smbus速率 来计算需要delay的次数 以及每次delay的时长
    dw_reg_read(dev,DW_IC_CON,&speed_val);
    speed_val = speed_val & (IC_CON_SPEED_MASK);
    switch (speed_val)
    {
        case IC_CON_SPEED_STANDARD:
            ti2c_poll = IC_DELAY_100US;
            max_t_poll_count = IC_DELAY_100US_COUNT;
            break;
        case IC_CON_SPEED_FAST:
            ti2c_poll = IC_DELAY_25US;
            max_t_poll_count = IC_DELAY_25US_COUNT;
            break;
        default:
            ///<  不支持其他速度
            LOGT("i2cSwitchMode,speed_val:%d\n", speed_val);
            return -1;
    }
    ///<  disable i2c
    ///<  dw_reg_read(dev,DW_IC_ENABLE,&val);
    ///<  val = val & (~(DW_I2C_ENABLE));
    ///<  dw_reg_write(dev,DW_IC_ENABLE,val);
    i2c_dw_disable(dev);
    ///<  若1s内 dw_apb_i2c 还未被disable 则返回失败
    for (i = 0; i < max_t_poll_count; i++)
    {
        dw_reg_read(dev,DW_IC_ENABLE_STATUS,&enable_sts);
        enable_sts = enable_sts & (IC_ENABLE_STATUS_IC_EN);
        if (!enable_sts)
        {
            break;
        }
        ///<  延时转换成us
        rtems_counter_delay_nanoseconds(ti2c_poll * IC_1US_TO_1000NANO);
    }

    /*  若i为max_t_poll_count 
        则表示经过 max_t_poll_count 次数后dw_apb_i2c仍然是enable的状态
        则返回失败
    */
    if (i == max_t_poll_count)
    {
        ///<  恢复enable状态
        __i2c_dw_enable(dev);
        return -1;
    }

    /* master模式只需要把 bit0 bit6 置1，其他配置保留
       slave模式只需要把 bit0 bit6 置0，其他配置保留 
    */
    dw_reg_read(dev,DW_IC_CON,&val);
    if (i2cMode)
    {
        ///<  DEV改配为master or slave后必须重新配置设备，例如中断
        val |= IC_CON_ENA_MASTER_MODE;
        ///<  i2cCtrlPtr->i2cMode = DEV_MASTER_MODE;
        ///<  i2cCtrlPtr->supportMode = i2cCtrlPtr->mstSupportMode;
        ///<  i2cRegPtr->IC_INTR_MASK = i2cCtrlPtr->mstIntMask;
        ///<  i2cRegPtr->IC_SMBUS_INTR_MASK = i2cCtrlPtr->mstSmbusIntMask;
    }
    else
    {
        ///<  DEV改配为master or slave后必须重新配置设备，例如中断
        val &= ~IC_CON_ENA_MASTER_MODE;
        ///<  i2cCtrlPtr->i2cMode = DEV_SLAVE_MODE;
        ///<  i2cCtrlPtr->supportMode = i2cCtrlPtr->slvSupportMode;
        ///<  i2cRegPtr->IC_INTR_MASK = i2cCtrlPtr->slvIntMask;
        ///<  i2cRegPtr->IC_SMBUS_INTR_MASK = i2cCtrlPtr->slvSmbusIntMask;
    }

    dw_reg_write(dev,DW_IC_CON,val);
    
    ///<  enable i2c
    __i2c_dw_enable(dev);
    
    return 0;
}

