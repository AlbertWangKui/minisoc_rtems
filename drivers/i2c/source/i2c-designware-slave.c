/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file i2c-designware-slave.c
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "common_defines.h"
#include "bsp_device.h"
#include "log_msg.h"
#include "drv_i2c_api.h"
#include "../include/i2c-designware-core.h"

ISR_lock_Control g_i2c_slave_lock = ISR_LOCK_INITIALIZER( "i2c_slave_lock" );

void i2c_dw_lock_slave(ISR_lock_Context *lock_contex)
{
    _ISR_lock_ISR_disable_and_acquire(&g_i2c_slave_lock , lock_contex);
    return;
}

void i2c_dw_unlock_slave(ISR_lock_Context *lock_contex)
{
    _ISR_lock_Release_and_ISR_enable( &g_i2c_slave_lock, lock_contex);
    return;
}

///< 和业务对齐，暂时没有响应广播地址需求，关闭广播地址响应
static inline void i2c_dw_slave_disable_ack_gen_call(struct dw_i2c_dev *dev)
{
    dw_reg_write(dev, DW_IC_ACK_GENERAL_CALL, 0);
}

///< 和OOM业务岳远志沟通，OOM定义的无效协议是返回4byte的0xFF
static int i2c_dw_slave_tx_buf_invalid(struct dw_i2c_dev *dev)
{
    if (SLAVE_BUF_LEN < 4) {
        return -EINVAL;
    } else {
        memset(dev->slave_tx_buf, 0xFF, 4);
        dev->slave_valid_usr_tx_len = 4;
        dev->slave_cur_tx_index = 0;
        return 0;
    }
}

///< 参考linux eeprom
///< 策略：
///< master写策略：master往slave buf每次都从头开始写，写完成后通知业务拷贝。业务保证master在其取之前不会再次写，否则会覆盖。
///< master读策略: 每次将业务复制到slave tx buf的数据反馈给master，发送完成后将数据清理。
static inline int i2c_slave_event(struct dw_i2c_dev *dev, enum i2c_slave_event event, U8 *val)
{
    static enum i2c_slave_event lastevt;
    int ret = 0;

    LOGT("i2c-%u process evt(%u)-value(%d), last evt is (%u).\n", 
            dev->i2c_channel_num, event, *val, lastevt);
    switch (event) {
        case I2C_SLAVE_WRITE_RECEIVED:
            lastevt = I2C_SLAVE_WRITE_RECEIVED;
            ISR_lock_Context lock_contex;
            i2c_dw_lock_slave(&lock_contex);
            if (dev->slave_valid_rx_len < SLAVE_BUF_LEN) {
                ///< 将master写入fifo的数据存放rx buf
                dev->slave_rx_buf[dev->slave_valid_rx_len++] = *val;
            } else {
                i2c_dw_unlock_slave(&lock_contex);
                ///< OOM业务设计上保证buf不会越界(1K)
               LOGT("I2C-%u rx buf over.\n", dev->i2c_channel_num); 
               ret = -ENOBUFS;
               goto exit;
            }
            i2c_dw_unlock_slave(&lock_contex);
            break;
        case I2C_SLAVE_READ_PROCESSED:
            dev->slave_cur_tx_index++;
            ///< OOM业务设计上保证buf不会越界(1K)
            if (dev->slave_cur_tx_index  >= dev->slave_valid_usr_tx_len) {
               LOGT("I2C-%u tx buf posi(%u) over valid len(%u), send invalid data.\n", 
                       dev->i2c_channel_num, dev->slave_cur_tx_index, dev->slave_valid_usr_tx_len); 
               *val = 0xFF;
               goto exit;
            }
            ///< 正常情况下fallthrough
        case I2C_SLAVE_READ_REQUESTED:
            lastevt = I2C_SLAVE_READ_REQUESTED;
            ///< OOM业务设计上TX buf不会为空，要么为4个0xFF，要么为业务有效数据
            if (dev->slave_valid_usr_tx_len == 0) {
               LOGT("I2C-%u tx buf empty.\n", dev->i2c_channel_num); 
               *val = 0xFF;
               goto exit;
            }
            ///< 此处处理的场景是上一次master read超时未产生stop事件，slave_cur_tx_index未清零
            ///< 这种场景下，master重新读，此时返回0xFF无效数据
            if (dev->slave_cur_tx_index  >= dev->slave_valid_usr_tx_len) {
               LOGT("I2C-%u tx buf posi(%u) over valid len(%u), send invalid data.\n", 
                       dev->i2c_channel_num, dev->slave_cur_tx_index, dev->slave_valid_usr_tx_len); 
               *val = 0xFF;
               goto exit;
            }
            *val = dev->slave_tx_buf[dev->slave_cur_tx_index];
            break;
        case I2C_SLAVE_STOP:
            ///< master读结束，通知上层写buf
            if (lastevt == I2C_SLAVE_READ_REQUESTED) {
                if (dev->callback_write_func != NULL) {
                    dev->callback_write_func(dev->callback_write_param);
                }
            } else {
            ///< master写结束，通知上层读buf
                if (dev->callback_read_func != NULL) {
                    dev->callback_read_func(dev->callback_read_param);
                }
            }
            lastevt = I2C_SLAVE_STOP;
            break;
        case I2C_SLAVE_WRITE_REQUESTED:
            lastevt = I2C_SLAVE_WRITE_REQUESTED;
            ///< master发起写操作，先将rx len置零
            ///< 正常情况下，是STOP事件触发业务回调read接口清零。这里处理的是master端超时未产生正常的stop事件
            dev->slave_valid_rx_len = 0; 
            break;
        default:
            break;
    }
exit:
    return ret;
}

static void i2c_dw_configure_fifo_slave(struct dw_i2c_dev *dev)
{
    /* Configure Tx/Rx FIFO threshold levels. */
    dw_reg_write(dev, DW_IC_TX_TL, 0);
    dw_reg_write(dev, DW_IC_RX_TL, 0);

    /* Configure the I2C slave. */
    dw_reg_write(dev, DW_IC_CON, dev->slave_cfg);
    dw_reg_write(dev, DW_IC_INTR_MASK, DW_IC_INTR_SLAVE_MASK);
}

/**
 * i2c_dw_init_slave() - Initialize the designware i2c slave hardware
 * @dev: device private data
 *
 * This function configures and enables the I2C in slave mode.
 * This function is called during I2C init function, and in case of timeout at
 * run time.
 */
static int i2c_dw_init_slave(struct dw_i2c_dev *dev)
{
    /* Disable the adapter. */
    __i2c_dw_disable(dev);

    /* Write SDA hold time if supported */
    if (dev->sda_hold_time) {
        dw_reg_write(dev, DW_IC_SDA_HOLD, dev->sda_hold_time);
    }

    i2c_dw_configure_fifo_slave(dev);

    ///< 关闭广播地址响应
    i2c_dw_slave_disable_ack_gen_call(dev);

    return 0;
}

int i2c_dw_set_slave_addr(struct dw_i2c_dev *dev)
{
    U32 stat;
    U32 addr;
    /*
     * Set slave address in the IC_SAR register,
     * the address to which the DW_apb_i2c responds.
     */
    dw_reg_read(dev, DW_IC_ENABLE, &stat);
    
    __i2c_dw_disable(dev);
    dw_reg_write(dev, DW_IC_SAR, dev->addr);

    if (stat & DW_IC_ENABLE_ENABLE) {
        __i2c_dw_enable(dev);
    }

    dw_reg_read(dev, DW_IC_SAR, &addr);
    if (addr != dev->addr) {
        LOGE("I2C-%u set slave addr[0x%x] err[0x%x].\n", dev->i2c_channel_num,
                dev->addr, addr); 
        ///< 设置失败后，恢复正确地址
        dev->addr = addr;
        return -EINVAL;
    }

    return 0;
}

static int i2c_dw_reg_slave(struct dw_i2c_dev *dev)
{
    U32 tmp;

    /*
     * Set slave address in the IC_SAR register,
     * the address to which the DW_apb_i2c responds.
     */
    __i2c_dw_disable_nowait(dev);
    dw_reg_write(dev, DW_IC_SAR, dev->addr);
    dw_reg_read(dev,DW_IC_SMBUS_INTR_MASK,&tmp);
    tmp |= (IC_SMBUS_ARP_ASSGIN_ADDR_CMD_BIT);
    tmp &= (~(IC_SMBUS_ARP_GET_UDID_CMD_BIT));
    tmp &= (~(IC_SMBUS_ARP_RESET_CMD_BIT));
    tmp &= (~(IC_SMBUS_ARP_PREPARE_CMD_BIT));
    dw_reg_write(dev,DW_IC_SMBUS_INTR_MASK, tmp); 
    __i2c_dw_enable(dev);

    dev->status = 0;

    return 0;
}

static inline int i2c_dw_unreg_slave(struct dw_i2c_dev *dev)
{
    dw_reg_write(dev, DW_IC_INTR_MASK, 0);
    i2c_dw_disable(dev);
    dev->addr = 0;

    return 0;
}

static U32 i2c_dw_read_clear_intrbits_slave(struct dw_i2c_dev *dev)
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
    if (stat & DW_IC_INTR_TX_ABRT)
        dw_reg_read(dev, DW_IC_CLR_TX_ABRT, &dummy);
    if (stat & DW_IC_INTR_RX_UNDER)
        dw_reg_read(dev, DW_IC_CLR_RX_UNDER, &dummy);
    if (stat & DW_IC_INTR_RX_OVER)
        dw_reg_read(dev, DW_IC_CLR_RX_OVER, &dummy);
    if (stat & DW_IC_INTR_TX_OVER)
        dw_reg_read(dev, DW_IC_CLR_TX_OVER, &dummy);
    if (stat & DW_IC_INTR_RX_DONE)
        dw_reg_read(dev, DW_IC_CLR_RX_DONE, &dummy);
    if (stat & DW_IC_INTR_ACTIVITY)
        dw_reg_read(dev, DW_IC_CLR_ACTIVITY, &dummy);
    if (stat & DW_IC_INTR_STOP_DET)
        dw_reg_read(dev, DW_IC_CLR_STOP_DET, &dummy);
    if (stat & DW_IC_INTR_START_DET)
        dw_reg_read(dev, DW_IC_CLR_START_DET, &dummy);
    if (stat & DW_IC_INTR_GEN_CALL)
        dw_reg_read(dev, DW_IC_CLR_GEN_CALL, &dummy);
    if (stat & DW_IC_INTR_RESTART_DET)
        dw_reg_read(dev, DW_IC_CLR_RESTART_DET, &dummy);

    return stat;
}

__attribute__((optimize(0)))
void dwI2cSlvArpAssignAddrFinishHandle(struct dw_i2c_dev *dev)
{
    U32 mask = 0;
    U32 ic_status = 0;
    U32 addrId = 0;
    U32 tmp;
    U32 icSar = 0;

    dw_reg_read(dev,DW_IC_SMBUS_INTR_STAT,&tmp);

    if ((tmp&(IC_SMBUS_ARP_GET_UDID_CMD_BIT)))
    {
        ///< LOGT("%s IC_SMBUS_ARP_GET_UDID_CMD_BIT ...0x%x \r\n",__func__,tmp);
        dw_reg_write(dev,DW_IC_CLR_SMBUS_INTR,IC_SMBUS_ARP_GET_UDID_CMD_BIT);
    }

    if ((tmp&(IC_SMBUS_ARP_RESET_CMD_BIT)))
    {
        ///< LOGT("%s IC_SMBUS_ARP_RESET_CMD_BIT ...0x%x \r\n",__func__,tmp);
        dw_reg_write(dev,DW_IC_CLR_SMBUS_INTR,IC_SMBUS_ARP_RESET_CMD_BIT);
    }

    if ((tmp&(IC_SMBUS_ARP_PREPARE_CMD_BIT)))
    {
        ///< LOGT("%s IC_SMBUS_ARP_PREPARE_CMD_BIT ...0x%x \r\n",__func__,tmp);
        dw_reg_write(dev,DW_IC_CLR_SMBUS_INTR,IC_SMBUS_ARP_PREPARE_CMD_BIT);
    }


    ///<  LOGT("stat: %x \r\n",tmp);
    if ((tmp&(IC_SMBUS_INTR_STAT_ARP_ASSGN_ADDR_CMD_DET)) == 0)
    {
        return;
    }

    if (dev->arpAssignedCB.arpAddrAssignedCB == NULL)
    {
        ///< clear ARP ASSIGN
        ///< LOGT("%s 2 ... \r\n",__func__);
        dw_reg_write(dev,DW_IC_CLR_SMBUS_INTR,IC_SMBUS_ARP_ASSGIN_ADDR_CMD_BIT);
        return;
    }

    ///< 关ARP_ASSGN_ADDR_CMD中断
    dw_reg_read(dev,DW_IC_SMBUS_INTR_MASK,&mask);
    mask = mask & (~(IC_SMBUS_ARP_ASSGIN_ADDR_CMD_BIT));
    dw_reg_write(dev,DW_IC_SMBUS_INTR_MASK,mask);
    rtems_counter_delay_nanoseconds(2000 * 1000 * 10);


    
    dw_reg_read(dev,DW_IC_STATUS,&ic_status);
    if ((ic_status & (IC_STATUS_SMBUS_SLAVE_ADDR_VALID))
        && (ic_status & (IC_STATUS_SMBUS_SLAVE_ADDR_RESOLVED)))
    {
        addrId = 1;
        ///< LOGT("%s 3 ... \r\n",__func__);
        dw_reg_read(dev,DW_IC_SAR,&icSar);
        dev->arpAssignedCB.arpAddrAssignedCB(addrId,icSar,dev->arpAssignedCB.param);
        ///< i2cCtrlPtr->i2cCbs.arpAssignCb(addrId, dwI2cRegs->IC_SAR, i2cCtrlPtr->i2cCbs.arpAssign_param);
    }

    LOGT("%s 4 0x%x ... \r\n",__func__,ic_status);
    if ((ic_status & (IC_STATUS_SMBUS_SLAVE_ADDR2_VALID))
        && (ic_status & (IC_STATUS_SMBUS_SLAVE_ADDR2_RESOLVED)))
    {
        addrId = 2;
        ///< LOGT("%s 5 ... \r\n",__func__);
        dw_reg_read(dev,DW_IC_SAR2,&icSar);
        dev->arpAssignedCB.arpAddrAssignedCB(addrId,icSar,dev->arpAssignedCB.param);
    }

    ///< 先清ARP_ASSGN_ADDR_CMD中断（写清），在开中断
    dw_reg_write(dev,DW_IC_CLR_SMBUS_INTR,IC_SMBUS_ARP_ASSGIN_ADDR_CMD_BIT);
    dw_reg_read(dev,DW_IC_SMBUS_INTR_MASK,&tmp);
    tmp |= (IC_SMBUS_ARP_ASSGIN_ADDR_CMD_BIT);
    dw_reg_write(dev,DW_IC_SMBUS_INTR_MASK,tmp);
}

/*
 * Interrupt service routine. This gets called whenever an I2C slave interrupt
 * occurs.
 */
static void i2c_dw_isr_slave(void *dev_id)
{
    struct dw_i2c_dev *dev = dev_id;
    U32 raw_stat, stat, enabled, tmp;
    U8 val = 0, slave_activity;
    unsigned int rx_valid;

    dw_reg_read(dev, DW_IC_ENABLE, &enabled);
    dw_reg_read(dev, DW_IC_RAW_INTR_STAT, &raw_stat);
    dw_reg_read(dev, DW_IC_STATUS, &tmp);
    slave_activity = ((tmp & DW_IC_STATUS_SLAVE_ACTIVITY) >> 6);

    if (!enabled || !(raw_stat & ~DW_IC_INTR_ACTIVITY)) {
        return;
    }

    dwI2cSlvArpAssignAddrFinishHandle(dev);

    dw_reg_read(dev, DW_IC_RXFLR, &rx_valid);
    stat = i2c_dw_read_clear_intrbits_slave(dev);
    LOGT("%#x STATUS SLAVE_ACTIVITY=%#x : RAW_INTR_STAT=%#x : INTR_STAT=%#x, RX_VALID=%u.\n",
            enabled, slave_activity, raw_stat, stat, rx_valid);

    if (stat & DW_IC_INTR_RX_FULL) {
        if (!(dev->status & STATUS_WRITE_IN_PROGRESS)) {
            dev->status |= STATUS_WRITE_IN_PROGRESS;
            dev->status &= ~STATUS_READ_IN_PROGRESS;
            i2c_slave_event(dev, I2C_SLAVE_WRITE_REQUESTED, &val);
        }

        do {
            dw_reg_read(dev, DW_IC_DATA_CMD, &tmp);
            if (tmp & DW_IC_DATA_CMD_FIRST_DATA_BYTE) {
                i2c_slave_event(dev, I2C_SLAVE_WRITE_REQUESTED, &val);
            }
            val = tmp;
            i2c_slave_event(dev, I2C_SLAVE_WRITE_RECEIVED, &val);
            dw_reg_read(dev, DW_IC_STATUS, &tmp);
        } while (tmp & DW_IC_STATUS_RFNE);
    }

    ///< 当前slave无法支持restart。
    ///< stop处理提前避免上一次stop影响这一次read
    if ((stat & DW_IC_INTR_STOP_DET) || (stat & DW_IC_INTR_RESTART_DET)) {
        if (dev->status & STATUS_READ_IN_PROGRESS) {
            dev->status &= ~STATUS_READ_IN_PROGRESS;
        }
        if (dev->status & STATUS_WRITE_IN_PROGRESS) {
            dev->status &= ~STATUS_WRITE_IN_PROGRESS;
        }
        i2c_slave_event(dev, I2C_SLAVE_STOP, &val);
    }

    if (stat & DW_IC_INTR_RD_REQ) {
        if (slave_activity) {
            dw_reg_read(dev, DW_IC_CLR_RD_REQ, &tmp);

            if (!(dev->status & STATUS_READ_IN_PROGRESS)) {
                i2c_slave_event(dev, I2C_SLAVE_READ_REQUESTED, &val);
                dev->status |= STATUS_READ_IN_PROGRESS;
                dev->status &= ~STATUS_WRITE_IN_PROGRESS;
            } else {
                i2c_slave_event(dev, I2C_SLAVE_READ_PROCESSED, &val);
            }
            dw_reg_write(dev, DW_IC_DATA_CMD, val);
        }
    }

    return;
}

void i2c_dw_configure_slave(struct dw_i2c_dev *dev)
{
    ///< rx fifo full hdl设置是为了在rxfifo满时，ip拉住总线
    ///< 默认slave为7bit模式，未配置DW_IC_CON_10BITADDR_SLAVE	
    dev->slave_cfg = DW_IC_CON_RX_FIFO_FULL_HLD_CTRL |
        DW_IC_CON_RESTART_EN | DW_IC_CON_STOP_DET_IFADDRESSED;
    if (dev->addr_mode == I2C_10BIT_ADDRESS) {
        dev->slave_cfg |= DW_IC_CON_10BITADDR_SLAVE;
    }

    dev->mode = DW_IC_SLAVE;
}

int i2c_dw_probe_slave(struct dw_i2c_dev *dev)
{
    int ret;
    rtems_status_code status;

    ///< sda hold参数改由用户配置
#if 0
    ret = i2c_dw_calc_sda_hold(dev);
    if (ret) {
        return ret;
    }
#endif

    ret = i2c_dw_calc_fifo_size(dev);
    if (ret) {
        return ret;
    }

    ret = i2c_dw_init_slave(dev);
    if (ret) {
        return ret;
    }

    ///< 初始化成功后分配slave buf，如果有反初始化要注意是否该资源
    dev->slave_rx_buf = (U8 *)malloc(SLAVE_BUF_LEN);
    if (dev->slave_rx_buf == NULL) {
        ret = -EPERM;
        LOGE("Slave i2c-%u failure to malloc rx buf(len:%d).\n", 
                dev->i2c_channel_num, SLAVE_BUF_LEN);
        return ret;
    }
    dev->slave_valid_rx_len = 0;
    dev->slave_tx_buf = (U8 *)malloc(SLAVE_BUF_LEN);
    if (dev->slave_tx_buf == NULL) {
        free(dev->slave_rx_buf);
        dev->slave_rx_buf = NULL;
        LOGE("Slave i2c-%u failure to malloc tx buf(len:%d).\n", 
                dev->i2c_channel_num, SLAVE_BUF_LEN);
        ret = -EPERM;
        return ret;
    }
    ///< 将tx buf数据置为OOM定义的无效值
    ret = i2c_dw_slave_tx_buf_invalid(dev);
    if (ret != 0) {
        free(dev->slave_rx_buf);
        free(dev->slave_tx_buf);
        dev->slave_rx_buf = NULL;
        dev->slave_tx_buf = NULL;
        LOGE("Slave i2c-%u invalid tx buf failed(%d).\n", dev->i2c_channel_num, ret);
        return ret;
    }
    snprintf(dev->i2c_name, I2C_NAME_LEN, "I2%2u", dev->i2c_channel_num % 100);
    dev->i2c_name[I2C_NAME_LEN - 1] = 0;
    status = rtems_interrupt_handler_install(dev->irq, dev->i2c_name, 
            RTEMS_INTERRUPT_UNIQUE, i2c_dw_isr_slave, dev);
    if (status != RTEMS_SUCCESSFUL) {
        free(dev->slave_rx_buf);
        free(dev->slave_tx_buf);
        dev->slave_rx_buf = NULL;
        dev->slave_tx_buf = NULL;
        ret = -EPERM;
        LOGE("Slave i2c-%u failure requesting irq %u: %u\n", 
                dev->i2c_channel_num, dev->irq, status);
        return ret;
    }

    ///< linux在save设备注册是会调用接口打开slave(i2c_slave_register->i2c_dw_reg_slave)。我们的接口和linux不一样，
    ///< 需要在slave初始化完成后就打开，参考i2c_dw_reg_slave函数
    (void)i2c_dw_reg_slave(dev);

    return ret;
}

int i2c_dw_unprobe_slave(struct dw_i2c_dev *dev)
{
    rtems_status_code status;
    i2c_dw_disable(dev);
    dev->slave_cfg = 0;
    status = rtems_interrupt_handler_remove(dev->irq, i2c_dw_isr_slave, dev);
    if (status != RTEMS_SUCCESSFUL) {
        LOGE("Remove irq %u: %u.\n", dev->irq, status);
    }
    if (dev->slave_rx_buf != NULL) {
        free(dev->slave_rx_buf);
        dev->slave_rx_buf = NULL;
    }
    if (dev->slave_tx_buf != NULL){
        free(dev->slave_tx_buf);
        dev->slave_tx_buf = NULL;
    }
    return 0;
}

static inline void dwI2cDisable(struct dw_i2c_dev *dev)
{
    U32 tmp;

    dw_reg_read(dev,DW_IC_ENABLE,&tmp);
    tmp &= ~(1);
    dw_reg_write(dev,DW_IC_ENABLE,tmp);
}

int smbusSarEnable(struct dw_i2c_dev *dev, int sarNum)
{
    U32 tmp = 0;

    if (sarNum > IC_SAR_NUM_COUNT)
    {
        return 0;
    }


    dwI2cDisable(dev);
    dw_reg_read(dev,DW_IC_ENABLE,&tmp);
    tmp |= (IC_SAR_ENABLE << (sarNum - 1));
    dw_reg_write(dev,DW_IC_ENABLE,tmp);
    __i2c_dw_enable(dev);

    return 0;
}

int smbusSarDisable(struct dw_i2c_dev *dev, int sarNum)
{
    U32 tmp = 0;

    if (sarNum > IC_SAR_NUM_COUNT)
    {
        return 0;
    }


    dwI2cDisable(dev);
    dw_reg_read(dev,DW_IC_ENABLE,&tmp);
    tmp &= ~(IC_SAR_ENABLE << (sarNum - 1));
    dw_reg_write(dev,DW_IC_ENABLE,tmp);
    __i2c_dw_enable(dev);

    return 0;
}

int smbusArpEnable(struct dw_i2c_dev *dev, int sarNum)
{
    U32 tmp = 0;

    if (sarNum > IC_SAR_NUM_COUNT)
    {
        return EXIT_FAILURE;
    }

    dwI2cDisable(dev);

    if (sarNum == IC_SAR_NUM)
    {
        dw_reg_read(dev,DW_IC_CON,&tmp);
        tmp |= (IC_CON_SMBUS_ARP_EN);
        dw_reg_write(dev,DW_IC_CON,tmp);
    }

    if (sarNum == IC_SAR2_NUM)
    {
        dw_reg_read(dev,DW_IC_CON,&tmp);
        tmp |= (IC_CON_SAR2_SMBUS_ARP_EN);
        dw_reg_write(dev,DW_IC_CON,tmp);
    }
    
    ///< enable i2c 
    __i2c_dw_enable(dev);

    return 0;
}

int smbusArpDisable(struct dw_i2c_dev *dev, int sarNum)
{
    U32 tmp = 0;

    if (sarNum > IC_SAR_NUM_COUNT)
    {
        return EXIT_FAILURE;
    }

    dwI2cDisable(dev);

    if (sarNum == IC_SAR_NUM)
    {
        dw_reg_read(dev,DW_IC_CON,&tmp);
        tmp &= ~(IC_CON_SMBUS_ARP_EN);
        dw_reg_write(dev,DW_IC_CON,tmp);
    }

    if (sarNum == IC_SAR2_NUM)
    {
        dw_reg_read(dev,DW_IC_CON,&tmp);
        tmp &= ~(IC_CON_SAR2_SMBUS_ARP_EN);
        dw_reg_write(dev,DW_IC_CON,tmp);
    }
    
    ///< enable i2c 
    __i2c_dw_enable(dev);

    return 0;
}

S32 smbusArpUdidSet(struct dw_i2c_dev *dev, U32 idx, DwSmbusArpUdid_t *udidInfo)
{
    U32 regAddr;
    U32 base = 0;

    base = (U32)(dev->base);

    dwI2cDisable(dev);
    regAddr = (idx == IC_SAR_NUM) ? (base+(DW_IC_SMBUS_UDID_WORD0)) : 
            (base+(DW_IC_SMBUS2_UDID_WORD0)) + ((idx - IC_SAR2_NUM) * 4*sizeof(U32));
    *(volatile U32*)regAddr = udidInfo->sar_udid0;
    *(volatile U32*)(regAddr+4) = udidInfo->sar_udid1;
    *(volatile U32*)(regAddr+8) = udidInfo->sar_udid2;
    *(volatile U32*)(regAddr+12) = udidInfo->sar_udid3;
    __i2c_dw_enable(dev);

    return 0;
}

S32 smbusArpUdidGet(struct dw_i2c_dev *dev, U32 idx, DwSmbusArpUdid_t *udidInfo)
{
    U32 regAddr;
    U32 base = 0;

    base = (U32)(dev->base);

    dwI2cDisable(dev);
    regAddr = (idx == IC_SAR_NUM) ? (base+(DW_IC_SMBUS_UDID_WORD0)) : 
            (base+(DW_IC_SMBUS2_UDID_WORD0)) + ((idx - IC_SAR2_NUM) * 4*sizeof(U32));
    udidInfo->sar_udid0 = *(volatile U32*)regAddr;
    udidInfo->sar_udid1 = *(volatile U32*)(regAddr+4);
    udidInfo->sar_udid2 = *(volatile U32*)(regAddr+8);
    udidInfo->sar_udid3 = *(volatile U32*)(regAddr+12);
    __i2c_dw_enable(dev);

    return 0;
}

S32 smbusArpAddrResolvedGetStatus(struct dw_i2c_dev *dev, U32 sarNum)
{
    U32 ic_status;


    dw_reg_read(dev,DW_IC_STATUS,&ic_status);

    switch (sarNum)
    {
        case IC_SAR_NUM:
            return (ic_status & IC_STATUS_SMBUS_SLAVE_ADDR_RESOLVED);

        case IC_SAR2_NUM:
            return (ic_status & IC_STATUS_SMBUS_SLAVE_ADDR2_RESOLVED);
        default:
            break;
    }

    return -EXIT_FAILURE;
}

S32 smbusArpAddrValidGetStatus(struct dw_i2c_dev *dev, U32 sarNum)
{
    U32 ic_status;

    dw_reg_read(dev,DW_IC_STATUS,&ic_status);

    switch (sarNum)
    {
        case IC_SAR_NUM:
            return (ic_status & IC_STATUS_SMBUS_SLAVE_ADDR_VALID);
    
        case IC_SAR2_NUM:
            return (ic_status & IC_STATUS_SMBUS_SLAVE_ADDR2_VALID);
        default:
            break;
    }

    return -EXIT_FAILURE;
}

