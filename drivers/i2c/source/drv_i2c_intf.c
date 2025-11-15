/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file drv_i2c_intf.c
 * @author
 * @date 2025/06/06
 * @brief none
 * @version v1.0
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <rtems/counter.h>
#include <errno.h>
#include <bsp.h>

#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_device.h"
#include "bsp_api.h"
#include "udelay.h"
#include "log_msg.h"
#include "drv_i2c_api.h"
#include "../include/i2c-designware-core.h"
#include "bsp_sbr.h"
#include "sbr_api.h"
#include "bsp_drv_id.h"

static S32 i2cDevCfgGet(DevList_e devId, SbrI2cSmbusCfg_s *i2cSbrCfg)
{
    S32 ret = EXIT_SUCCESS;

    if (i2cSbrCfg == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

    if (devSbrRead(devId, i2cSbrCfg, 0, sizeof(SbrI2cSmbusCfg_s)) != sizeof(SbrI2cSmbusCfg_s)) {
        ret = -EXIT_FAILURE;
        goto out;
    }

#ifdef CONFIG_DUMP_SBR
    LOGI("i2c: SBR dump - regAddr:%p, irqNo:%u, irqPrio:%u, masterMode:%u, interruptMode:%u, speed:%u, addrMode:%u\r\n",
         i2cSbrCfg->regAddr, i2cSbrCfg->irqNo, i2cSbrCfg->irqPrio, i2cSbrCfg->masterMode,
         i2cSbrCfg->interruptMode, i2cSbrCfg->speed, i2cSbrCfg->addrMode);
    LOGI("i2c: SBR dump - slaveAddrHigh:%u, slaveAddrLow:%u, enSmbus:%u, reserved:%u\r\n",
         i2cSbrCfg->slaveAddrHigh, i2cSbrCfg->slaveAddrLow, i2cSbrCfg->enSmbus, i2cSbrCfg->reserved);
#endif

    if (i2cSbrCfg->regAddr == NULL) {
        ret = -EXIT_FAILURE;
        goto out;
    }

out:
    return ret;
}

S32 i2cTransfer(int i2cDevId, struct i2c_msg msgs[], int num)
{
    int ret = EXIT_SUCCESS;
    int i;
    struct dw_i2c_dev *dev = NULL;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (dev == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if ((msgs == NULL) || (num == 0)) {
        ret = -EINVAL;
        LOGE("I2c-%d transfer parameter err(msg buf:%p, num:%u).\n", dev->i2c_channel_num, msgs, num);
        goto exit;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (dev->mode == DW_IC_MASTER) {
        ///< 业务可不关注从设备地址长度flag
        if (dev->master_cfg & DW_IC_CON_10BITADDR_MASTER) {
            for (i=0; i<num; i++) {
                msgs[i].flags |= I2C_M_TEN;
            }
        }
        ret = i2c_dw_xfer(dev, msgs, num);
        if (ret < 0) {
            LOGT("Master(i2c-%d) transfer failed(%d).\n", dev->i2c_channel_num, ret);
        } else {
            ret = 0;
        }
        goto unlock;
    } else {
        ret = -EINVAL;
        LOGE("I2c-%d is mod[%d], not support transfer.\n", dev->i2c_channel_num, dev->mode);
        goto unlock;
    }

unlock:
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

S32 i2cRead(DevList_e i2cDevId, int addr, void *data, U32 len)
{
    int ret;
    struct dw_i2c_dev *dev = NULL;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (dev == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if ((data == NULL) || (len == 0)) {
        ret = -EINVAL;
        LOGE("I2c-%d read parameter err(data buf:%p, len:%u).\n", dev->i2c_channel_num, data, len);
        goto exit;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    if (dev->mode == DW_IC_MASTER) {
        struct i2c_msg msg_xfer;
        int num;
        msg_xfer.addr = addr;
        ///< linux master初始化时不限制模式，会根据slave的flags动态配置master模式
        ///< 由于秦少青、刘畅前期接口限制，此处做适配，实际上限制了master的灵活性
        if (dev->master_cfg & DW_IC_CON_10BITADDR_MASTER) {
            msg_xfer.flags = I2C_M_TEN | I2C_M_RD;
        } else {
            msg_xfer.flags = I2C_M_RD;
        }
        msg_xfer.buf = data;
        msg_xfer.len = len;
        num = 1;
        ret = i2c_dw_xfer(dev, &msg_xfer, num);
        if (ret < 0) {
            LOGT("Master(i2c-%d) read failed(%d).\n", dev->i2c_channel_num, ret);
        } else {
            ret = 0;
        }
        goto unlock;
    } else {
        ISR_lock_Context lock_contex;
        i2c_dw_lock_slave(&lock_contex);
        if (dev->work_mode == 0) {
            ///<  scan场景会发送数据长度为0的通信
            if (dev->slave_valid_rx_len == 0) {
                i2c_dw_unlock_slave(&lock_contex);
                ret = 0; ///< 需要和业务对齐扫描的具体意义
                goto unlock;
            } else {
                ///< 与业务对齐，拷贝min(userlen, valid_rx_len)，并且返回实际收到的长度和业务buffer长度的较小值
                ret = min(len, dev->slave_valid_rx_len);
                memcpy((U8 *)data, dev->slave_rx_buf, min(len, dev->slave_valid_rx_len));
                memset(dev->slave_rx_buf, 0, SLAVE_BUF_LEN);
                dev->slave_valid_rx_len = 0;
                i2c_dw_unlock_slave(&lock_contex);
                goto unlock;
            }
        } else {
            ret = -EPERM;
            i2c_dw_unlock_slave(&lock_contex);
            LOGE("Slave(i2c-%d) polling mode not supported.\n", dev->i2c_channel_num);
            goto unlock;
        }
    }

unlock:
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

S32 i2cWrite(DevList_e i2cDevId, int addr, void *data, U32 len)
{
    int ret;
    struct dw_i2c_dev *dev = NULL;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (dev == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if ((data == NULL) || (len == 0)) {
        ret = -EINVAL;
        LOGE("I2c-%d write parameter err(data buf:%p, len:%u.\n", dev->i2c_channel_num, data, len);
        goto exit;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (dev->mode == DW_IC_MASTER) {
        struct i2c_msg msg_xfer;
        int num;
        msg_xfer.addr = addr;
        ///< linux master初始化时不限制模式，会用户传入的flags动态配置master模式
        ///< 由于秦少青、刘畅前期接口限制，此处做适配，实际上限制了master的灵活性
        if (dev->master_cfg & DW_IC_CON_10BITADDR_MASTER) {
            msg_xfer.flags = I2C_M_TEN;
        } else {
            msg_xfer.flags = 0;
        }
        msg_xfer.buf = data;
        msg_xfer.len = len;
        num = 1;
        ret = i2c_dw_xfer(dev, &msg_xfer, num);
        if (ret < 0) {
            LOGT("Master(i2c-%d) write failed(%d).\n", dev->i2c_channel_num, ret);
        } else {
            ret = 0;
        }
        goto unlock;
    } else {
        if (dev->work_mode == 0) {
            if (len > SLAVE_BUF_LEN) {
                ret = -ENOBUFS;
                LOGE("Slave(i2c-%d) usr int write len(%u) exceed slave buf len(%u).\n",
                        dev->i2c_channel_num, len, SLAVE_BUF_LEN);
                goto unlock;
            } else {
                if (len > (SLAVE_BUF_LEN)) {
                    ret = -EPERM;
                    goto unlock;
                }

                memset(dev->slave_tx_buf, 0, SLAVE_BUF_LEN);
                memcpy(dev->slave_tx_buf, (U8 *)data, len);
                dev->slave_valid_usr_tx_len = len;
                dev->slave_cur_tx_index = 0;
                ret = 0; ///< slave写直接返回0，但是这不代表操作真正成功，需要在中断判断
                goto unlock;
            }
        }
        else {
            ret = -EPERM;
            LOGE("Slave(i2c-%d) polling mode not supported.\n", dev->i2c_channel_num);
            goto unlock;
        }
    }

unlock:
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

S32 i2cSclStuckCheck(DevList_e i2cDevId)
{
    U32 enable_old;
    U32 con;
    U32 val;
    S32 ret = 0;
    struct dw_i2c_dev *dev = NULL;
    U32 origin_scl_low_timeout;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (dev == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    enable_old = dev->status & STATUS_ACTIVE;

    __i2c_dw_disable(dev);
    dw_reg_read(dev, DW_IC_SCL_STUCK_AT_LOW_TIMEOUT, &origin_scl_low_timeout);
    dw_reg_write(dev, DW_IC_SCL_STUCK_AT_LOW_TIMEOUT, 2);
    dw_reg_read(dev, DW_IC_CON, &con);
    con |= DW_IC_CON_BUS_CLEAR_CTRL;
    dw_reg_write(dev, DW_IC_CON, con);
    dw_reg_write(dev, DW_IC_ENABLE, DW_IC_ENABLE_ENABLE);
    ///< 10us, timeout
    udelay(10);

    dw_reg_read(dev, DW_IC_RAW_INTR_STAT, &val);
    if (DW_IC_INTR_SCL_STUCK_AT_LOW & val) {
        LOGE("i2c-%d scl stuck, can't recover.\n", dev->i2c_channel_num);
        ret = 1;
    }

    __i2c_dw_disable(dev);
    dw_reg_write(dev, DW_IC_SCL_STUCK_AT_LOW_TIMEOUT, origin_scl_low_timeout);
    dw_reg_write(dev, DW_IC_ENABLE, DW_IC_ENABLE_ENABLE);

    if (!enable_old)
        __i2c_dw_disable(dev);

    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

static void i2c_dw_sda_stuck_recover(struct dw_i2c_dev *dev)
{
    U32 enable_old;
    U32 enable;
    U32 con;
    U32 val;
    int timeout;
    U32 origin_sda_low_timeout;

    enable_old = dev->status & STATUS_ACTIVE;

    ///< 需要调用i2c_dw_disable，清除已存在错误状态
    i2c_dw_disable(dev);
    dw_reg_read(dev, DW_IC_SDA_STUCK_AT_LOW_TIMEOUT, &origin_sda_low_timeout);
    dw_reg_write(dev, DW_IC_SDA_STUCK_AT_LOW_TIMEOUT, 2);
    dw_reg_read(dev, DW_IC_CON, &con);
    con |= DW_IC_CON_BUS_CLEAR_CTRL;
    dw_reg_write(dev, DW_IC_CON, con);
    __i2c_dw_enable(dev);

    ///< 10us, timeout
    rtems_counter_delay_nanoseconds(10000);
    dw_reg_read(dev, DW_IC_TX_ABRT_SOURCE, &val);
    if (val & DW_IC_TX_ABRT_SDA_STUCK_AT_LOW) {
        enable = DW_IC_ENABLE_ENABLE | DW_IC_ENABLE_SDA_STUCK_RECOVERY;
        dw_reg_write(dev, DW_IC_ENABLE, enable);
        timeout = 15;
        while (--timeout) {
            ///< timeout: 10us * 15
            rtems_counter_delay_nanoseconds(10000);
            dw_reg_read(dev, DW_IC_ENABLE, &enable);
            if (!(enable & DW_IC_ENABLE_SDA_STUCK_RECOVERY)) {
                break;
            }
        }
        if (!timeout) {
            LOGE("i2c-%d timeout while trying to recover sda stuck.\n",
                        dev->i2c_channel_num);
        } else {
            dw_reg_read(dev, DW_IC_STATUS, &val);
            if (val & DW_IC_STATUS_SDA_STUCK_NOT_RECOVERED)
                LOGE("i2c-%d sda stuck can't recover.\n", dev->i2c_channel_num);
        }
    }

    i2c_dw_disable(dev);
    dw_reg_write(dev, DW_IC_SDA_STUCK_AT_LOW_TIMEOUT, origin_sda_low_timeout);
    __i2c_dw_enable(dev);

    if (!enable_old)
        __i2c_dw_disable(dev);
}

///< linux并没有实现reset接口，该接口:先关闭设备释放资源，然后重新配置.兼容业务，没有返回值
void i2cReset(DevList_e i2cDevId)
{
    int ret = EXIT_SUCCESS;
    struct dw_i2c_dev *dev = NULL;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (dev == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (dev->mode == DW_IC_MASTER) {
        i2c_dw_sda_stuck_recover(dev);
        (void)i2c_dw_unprobe_master(dev);
    } else {
        (void)i2c_dw_unprobe_slave(dev);
    }

    ///< 复位通道
    if (peripsClockEnable(i2cDevId) != 0) {
        LOGE("enable i2c-%d clock failed! \n", dev->i2c_channel_num);
        ret = -EXIT_FAILURE;
        goto unlock;
    }

    if (peripsReset(i2cDevId) != 0) {
        LOGE("reset i2c-%d failed! \n", dev->i2c_channel_num);
        ret = -EXIT_FAILURE;
        goto unlock;
    }

    ///< 当前把pmbus/smbus当成普通i2c使用，复位需要清除默认中断mask配置
    if (dev->is_smbus) {
        dw_reg_write(dev, DW_IC_SMBUS_INTR_MASK, 0);
    }

    if (dev->mode == DW_IC_MASTER) {
        i2c_dw_configure_master(dev);
        ret = i2c_dw_probe_master(dev);
        if (ret != 0) {
            LOGE("Init i2c-%d master failed(%d).\n", dev->i2c_channel_num, ret);
            goto unlock;
        }
    } else {
        i2c_dw_configure_slave(dev);
        if (dev->work_mode == 0) {
            ret = i2c_dw_probe_slave(dev);
            if (ret != 0) {
                LOGE("Init i2c-%d slave failed(%d).\n", dev->i2c_channel_num, ret);
                goto unlock;
            }
        } else {
            ///< 暂时不支持轮询模式
            LOGE("Slave(i2c-%d) polling mode not supported.\n", dev->i2c_channel_num);
            goto exit;
        }
    }

unlock:
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return;
}

void i2cDeInit(DevList_e i2cDevId)
{
    struct dw_i2c_dev *i2cDev = NULL;

    if (isDrvInit(i2cDevId) == false) {
        goto exit;
    }

    if (getDevDriver(i2cDevId,(void**)&i2cDev) != 0) {
        goto exit;
    }

    if (i2cDev == NULL) {
        goto exit;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        goto exit;
    }

    i2cDev->transfer_timeout = I2C_TRANSFER_TIMEOUT_DEF_NUM;
    if (i2cDev->mode == DW_IC_MASTER) {
        (void)i2c_dw_unprobe_master(i2cDev);
    } else {
        (void)i2c_dw_unprobe_slave(i2cDev);
        i2cDev->addr = 0;
        i2cDev->callback_read_func = NULL;
        i2cDev->callback_read_param = NULL;
        i2cDev->callback_write_func = NULL;
        i2cDev->callback_write_param = NULL;
    }
    i2cDev->enabled = DEV_DISABLED;

    peripsReset(i2cDevId);
    drvUninstall(i2cDevId);
    devUnlockByDriver(i2cDevId);
exit:
    return;
}

static inline void dwI2cUnmaskInterrupt(struct dw_i2c_dev *dev, U32 mask)
{
    U32 tmp;

    dw_reg_read(dev,DW_IC_INTR_MASK,&tmp);
    tmp |= mask;
    dw_reg_write(dev,DW_IC_INTR_MASK,tmp);
}

static inline void dwI2cMaskInterrupt(struct dw_i2c_dev *dev, U32 mask)
{
    U32 tmp;

    dw_reg_read(dev,DW_IC_INTR_MASK,&tmp);
    tmp &= ~mask;
    dw_reg_write(dev,DW_IC_INTR_MASK,tmp);
}
///< 需要有约束，业务先调用control接口再调用init接口 TODO 待与业务对齐
S32 i2cControl(DevList_e i2cDevId, U32 ctrlCmd, void *param)
{
    int ret = EXIT_SUCCESS;
    unsigned int timeout;
    unsigned int sda_hold_time;
    U32 val32;
    struct dw_i2c_dev *dev = NULL;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (dev == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ///< 入参检查
    if (param == NULL) {
        ret = -EINVAL;
        LOGE("I2c-%d control parameter err(ctrlcmd:%u, param:%p).\n", dev->i2c_channel_num, ctrlCmd, param);
        goto exit;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    switch (ctrlCmd) {
        case I2C_CMD_SET_ADDR_MODE:
            dev->addr_mode = *(unsigned int *)param;
            break;
        case I2C_CMD_SLV_SET_SLV_ADDR:
            dev->addr = *(unsigned int *)param;
            ret = i2c_dw_set_slave_addr(dev);
            break;
        case I2C_CMD_SET_TRANSFER_TIMEOUT:
            timeout = *(unsigned int *)param;
            if (timeout < I2C_TRANSFER_TIMEOUT_MIN_NUM) {
                LOGE("I2c-%d set timeout %d err less %d.\n", dev->i2c_channel_num, timeout,
                        I2C_TRANSFER_TIMEOUT_MIN_NUM);
                ret = -EINVAL;
            } else {
                dev->transfer_timeout = timeout;
            }
            break;
        case I2C_CMD_SET_SDA_HOLD_TIME:
            sda_hold_time = *(unsigned int *)param;
            if (sda_hold_time) {
                dev->sda_hold_time = sda_hold_time;
            }
            break;
        case I2C_CMD_GET_CHANNEL_MODE:
            if (dev->master_cfg != 0) {
                *(DwI2cMode_t *)param = DEV_MASTER_MODE;
            } else if (dev->slave_cfg != 0) {
                *(DwI2cMode_t *)param = DEV_SLAVE_MODE;
            } else {
                *(DwI2cMode_t *)param = DEV_INVALID_MODE;
            }
            break;
        case I2C_CMD_GET_STATUS:
            *((U32 *)param) = dev->enabled;
            break;
        case I2C_CMD_ENA_DEV:
            dw_reg_read(dev,DW_IC_ENABLE,&val32);
            val32 |= (DW_I2C_ENABLE);
            dw_reg_write(dev,DW_IC_ENABLE,val32);
            break;
        case I2C_CMD_DIS_DEV:
            dw_reg_read(dev,DW_IC_ENABLE,&val32);
            val32 &= (~(DW_I2C_ENABLE));
            dw_reg_write(dev,DW_IC_ENABLE,val32);
            break;
        case I2C_CMD_RESET:
            /* i2cReset(i2cDevId); */
            /* 不实现这个命令，不然会导致递归调用devLockByDriver(),虽然这个函数可以递归调用 */
            LOGI("i2cReset(i2cDevId) not implemented, please use i2cFree() instead.\n");
            break;
        case I2C_CMD_FLUSH_TX:
        case I2C_CMD_FLUSH_RX:
            dw_reg_read(dev,DW_IC_CLR_INTR,&val32);
            val32 = val32;
            break;
        case I2C_CMD_SET_TXINT:
            val32 = *((U32 *)param);
            if (dev->master_cfg != 0) {
                if (val32 == 0)
                {
                    dwI2cMaskInterrupt(dev, IC_INT_MST_TX_ENABLE);
                }
                else
                {
                    ///< enable intr
                    dwI2cUnmaskInterrupt(dev, IC_INT_MST_TX_ENABLE);
                }
            }
            else if (dev->slave_cfg != 0) {
                if (val32 == 0)
                {
                    ///< disable intr
                    dwI2cMaskInterrupt(dev, IC_INT_SLV_TX_ENABLE);
                }
                else
                {
                    ///< enable intr
                    dwI2cUnmaskInterrupt(dev, IC_INT_SLV_TX_ENABLE);
                }
            }
            else {
                ret = -EINVAL; ;
            }

            break;
        case I2C_CMD_SET_RXINT:
            val32 = *((U32 *)param);
            if (dev->master_cfg != 0) {
                if (val32 == 0)
                {
                    ///< disable intr,
                    dwI2cMaskInterrupt(dev, IC_INT_MST_RX_ENABLE);
                }
                else
                {
                    ///< enable intr
                    dwI2cUnmaskInterrupt(dev, IC_INT_MST_RX_ENABLE);
                }
            }
            else if (dev->slave_cfg != 0) {
                if (val32 == 0)
                {
                    ///< disable intr
                    dwI2cMaskInterrupt(dev, IC_INT_SLV_RX_ENABLE);
                }
                else
                {
                    ///< enable intr
                    dwI2cUnmaskInterrupt(dev, IC_INT_SLV_RX_ENABLE);
                }
            }
            else {
                ret = -EINVAL;
            }

            break;
        default:
            ret = -EINVAL;
            LOGE("I2c-%d invliad control cmd(%u).\n", dev->i2c_channel_num, ctrlCmd);
            break;
    }

    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

///< 初始化时会申请信号量、注册中断、slave模式会申请内存，如果有反初始化接口注意资源释放
S32 i2cInit(DevList_e i2cDevId, DwI2cMode_t i2cMode, DevMethod_t method, I2cSpeedMode_t speed)
{
    int ret;
    struct dw_i2c_dev *dev = NULL;
    Device_s *pDevice = NULL;
    SbrI2cSmbusCfg_s i2cSbrCfg = {0};

    if (isDrvInit(i2cDevId)) {
        ret = -EBUSY;
        goto exit;
    }

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if(devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    pDevice = getDevice(i2cDevId);
    if (pDevice == NULL) {
        ret = -EXIT_FAILURE;
        goto unlock;
    }

    dev = (struct dw_i2c_dev*)malloc(sizeof(struct dw_i2c_dev));
    if (dev == NULL) {
        ret = -EXIT_FAILURE;
        goto unlock;
    }

    memset(dev,0,sizeof(struct dw_i2c_dev));

    ///< 从SBR读取I2C配置
    if (i2cDevCfgGet(i2cDevId, &i2cSbrCfg) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto freeMem;
    }

    ///< 配置设备参数
    dev->base = (U32*)i2cSbrCfg.regAddr;
    dev->irq = i2cSbrCfg.irqNo;
    dev->i2c_channel_num = i2cDevId;// -DEVICE_I2C0; ///< 根据设备ID计算通道号
    dev->is_smbus = i2cSbrCfg.enSmbus;
    if (i2cMode == DEV_INVALID_MODE) { ///< 如果入参为无效模式，使用SBR配置中的masterMode
        i2cMode = (DwI2cMode_t)i2cSbrCfg.masterMode;
    }

    ///< 设置地址模式（0=7bit，1=10bit）
    if (i2cSbrCfg.addrMode) {
        dev->addr_mode = I2C_10BIT_ADDRESS;
    } else {
        dev->addr_mode = I2C_7BIT_ADDRESS;
    }

    ///< 设置从机地址（如果当前初始化为从机模式）
    if (i2cMode == DEV_SLAVE_MODE) {
        dev->addr = (i2cSbrCfg.slaveAddrHigh << 8) | i2cSbrCfg.slaveAddrLow;
    }

    LOGT("i2c%d device informations: \r\n",dev->i2c_channel_num);
    LOGT("reg: %p, irq: %d, id: %d, is_smbus: %d \r\n",dev->base,dev->irq,dev->i2c_channel_num,dev->is_smbus);

    ///< 目前只有双子V200和射手初始化或者reset接口复位i2c通道, do not delete this comment
    if (peripsClockEnable(i2cDevId) != 0) {
        LOGE("enable i2c-%d clock failed! \n", dev->i2c_channel_num);
        ret = -EXIT_FAILURE;
        goto freeMem;
    }

    if (peripsReset(i2cDevId) != 0) {
        LOGE("reset i2c-%d failed! \n", dev->i2c_channel_num);
        ret = -EXIT_FAILURE;
        goto freeMem;
    }

    ///< 当前把pmbus/smbus当成普通i2c使用，复位需要清除默认中断mask配置
    if (dev->is_smbus) {
        dw_reg_write(dev, DW_IC_SMBUS_INTR_MASK, 0);
    }

    ///< 目前只有双子V200初始化时配置驱动能力, do not delete this comment

    ///< 根据SBR配置或入参设置工作模式（优先使用SBR配置）
    if (method == DEV_POLL_METHOD) {
        dev->work_mode = 1;
    } else if (method == DEV_INTERRUPT_METHOD) {
        dev->work_mode = 0;
    } else {
        dev->work_mode = (i2cSbrCfg.interruptMode == 1) ? DEV_INTERRUPT_METHOD : DEV_POLL_METHOD;
    }

    dev->transfer_timeout = I2C_TRANSFER_TIMEOUT_DEF_NUM;

    ///< 根据SBR配置或入参设置速度（优先使用SBR配置）
    I2cSpeedMode_t finalSpeed = speed;
    if (speed == I2C_SPEED_INVALID) { ///< 如果入参为无效值，使用SBR配置
        finalSpeed = (I2cSpeedMode_t)i2cSbrCfg.speed;
    }

    switch(finalSpeed) {
        case I2C_SPEED_STANDARD:
            dev->timings.bus_freq_hz = I2C_MAX_STANDARD_MODE_FREQ;
            break;
        case I2C_SPEED_FAST:
            dev->timings.bus_freq_hz = I2C_MAX_FAST_MODE_FREQ;
            break;
        case I2C_SPEED_FASTPLUS:
            dev->timings.bus_freq_hz = I2C_MAX_FAST_MODE_PLUS_FREQ;
            break;
        case I2C_SPEED_HIGH:
            dev->timings.bus_freq_hz = I2C_MAX_HIGH_SPEED_MODE_FREQ;
            break;
        case I2C_SPEED_ULTRA:
            dev->timings.bus_freq_hz = I2C_MAX_ULTRA_FAST_MODE_FREQ;
            break;
        default:
            ret = -EINVAL;
            goto freeMem;
    }

    dev->arpAssignedCB.arpAddrAssignedCB = NULL;
    dev->arpAssignedCB.param = NULL;

    if (i2cMode == DEV_MASTER_MODE) {
        i2c_dw_configure_master(dev);
        ret = i2c_dw_probe_master(dev);
        if (ret != 0) {
            LOGE("Init i2c-%d master failed(%d).\n", dev->i2c_channel_num, ret);
            goto freeMem;
        }
        dev->enabled = DEV_ENABLED;
    } else {
        i2c_dw_configure_slave(dev);
        if (dev->work_mode == 0) {
            ret = i2c_dw_probe_slave(dev);
            if (ret != 0) {
                LOGE("Init i2c-%d slave failed(%d).\n", dev->i2c_channel_num, ret);
                goto freeMem;
            }
            dev->enabled = DEV_ENABLED;
        } else {
            ///< 暂时不支持轮询模式
            ret = -EINVAL;
            LOGE("Slave(i2c-%d) polling mode not supported.\n", dev->i2c_channel_num);
            goto freeMem;
        }
    }

    if (drvInstall(i2cDevId,dev) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto freeMem;
    }

    ret = EXIT_SUCCESS;

freeMem:
    free(dev);

unlock:
    devUnlockByDriver(i2cDevId);

exit:
    return ret;
}

S32 i2cRegArpAssignAddrFinishCb(DevList_e i2cDevId,ArpAssignAddrFinishCb arpAddrFinishCb, void* paramArp)
{
    S32 ret = EXIT_SUCCESS;
    struct dw_i2c_dev *dev = NULL;

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (dev == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    dev->arpAssignedCB.arpAddrAssignedCB = arpAddrFinishCb;
    dev->arpAssignedCB.param = paramArp;

    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

void i2cIntrReg(DevList_e i2cDevId, DevCallBack callBackRd, void *paramRd,
        DevCallBack callBackWr, void *paramWr)
{
    struct dw_i2c_dev *dev = NULL;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        goto exit;
    }

    if (dev == NULL) {
        goto exit;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        goto exit;
    }

    dev->callback_read_func = callBackRd;
    dev->callback_read_param = paramRd;
    dev->callback_write_func = callBackWr;
    dev->callback_write_param = paramWr;

    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        goto exit;
    }

exit:
    return;
}

#if 0
S32 i2cSmbusDevAddrAssign(U32 i2cDevId,uint8_t assign_addr)
{
    struct dw_i2c_dev *dev = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        return -EXIT_FAILURE;
    }

    if (dev == NULL) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = smbusDevAddrAssign(dev,assign_addr);
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

S32 i2cSmbusSarEnable(U32 i2cDevId,uint8_t sarNum)
{
    struct dw_i2c_dev *dev = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        return -EXIT_FAILURE;
    }

    if (dev == NULL) {
        return -EXIT_FAILURE;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = smbusSarEnable(dev,sarNum);
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

S32 i2cSmbusSarDisable(U32 i2cDevId,uint8_t sarNum)
{
    struct dw_i2c_dev *dev = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        return -EXIT_FAILURE;
    }

    if (dev == NULL) {
        return -EXIT_FAILURE;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = smbusSarDisable(dev,sarNum);
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

S32 i2cSmbusArpEnable(U32 i2cDevId,uint8_t sarNum)
{
    struct dw_i2c_dev *dev = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        return -EXIT_FAILURE;
    }

    if (dev == NULL) {
        return -EXIT_FAILURE;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = smbusArpEnable(dev,sarNum);
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

S32 i2cSmbusArpDisable(U32 i2cDevId,uint8_t sarNum)
{
    struct dw_i2c_dev *dev = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        return -EXIT_FAILURE;
    }

    if (dev == NULL) {
        return -EXIT_FAILURE;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = smbusArpDisable(dev,sarNum);
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

S32 i2cSmbusArpUdidSet(U32 i2cDevId,uint8_t sarNum,DwSmbusArpUdid_t *udid)
{
    struct dw_i2c_dev *dev = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        return -EXIT_FAILURE;
    }

    if (dev == NULL) {
        return -EXIT_FAILURE;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = smbusArpUdidSet(dev,sarNum,udid);
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

S32 i2cSmbusArpUdidGet(U32 i2cDevId,uint8_t sarNum,DwSmbusArpUdid_t *udid)
{
    struct dw_i2c_dev *dev = NULL;
    S32 ret = EXIT_SUCCESS;

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        return -EXIT_FAILURE;
    }

    if (dev == NULL) {
        return -EXIT_FAILURE;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = smbusArpUdidGet(dev,sarNum,udid);
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:

    return ret;
}
#endif
#if 0
S32 i2cSmbusArpAddrResolvedGetStatus(U32 i2cDevId, U32 sarNum)
{
    struct dw_i2c_dev *dev = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        return -EXIT_FAILURE;
    }

    if (dev == NULL) {
        return -EXIT_FAILURE;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = smbusArpAddrResolvedGetStatus(dev,sarNum);
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:

    return ret;
}


S32 i2cSmbusArpAddrValidGetStatus(U32 i2cDevId, U32 sarNum)
{
    struct dw_i2c_dev *dev = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        return -EXIT_FAILURE;
    }

    if (dev == NULL) {
        return -EXIT_FAILURE;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    ret = smbusArpAddrValidGetStatus(dev,sarNum);
     if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}

S32 i2cSmbusMasterArpNotifyCmd(U32 i2cDevId)
{
    U8 cmd[] = {IC_SMBUS_MST_NOTIFY_DEVID, IC_SMBUS_MST_NOTIFY_DATA0, IC_SMBUS_MST_NOTIFY_DATA1};
    S32 ret;
    S32 len;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EBUSY;
        goto exit;
    }

    len = sizeof(cmd)/sizeof(cmd[0]);
    ret = i2cWrite(i2cDevId, IC_SMBUS_MST_NOTIFY_ADDR, cmd, len);
    if (ret != 0)
    {
        ret = -EXIT_FAILURE;
        goto unlock;
    }

unlock:
    devUnlockByDriver(i2cDevId);

exit:
    return ret;
}
#endif
S32 i2cSwitchMode(U32 i2cDevId, DwI2cMode_t i2cMode)
{
    struct dw_i2c_dev *dev = NULL;
    S32 ret = EXIT_SUCCESS;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return -EINVAL;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        return -EXIT_FAILURE;
    }

    if (dev == NULL) {
        return -EXIT_FAILURE;
    }

    if (devLockByDriver(i2cDevId, 1000) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
        goto exit;
    }

    //ret = smbusMasterSlaveModeSwitch(dev,i2cMode);
    if (devUnlockByDriver(i2cDevId) != EXIT_SUCCESS) {
        ret = -EXIT_FAILURE;
    }

exit:
    return ret;
}


/*******************************************************************************
@ Brief: regArbitrationFailedCb
@ Param [i2cNum]: i2c number, callback handler and callback parameter
@ Return: none
@ Note: none
--------------------------------------------------------------------------------
@ Author: hursh@starsmicrosystem.com
@ Data: 2023/06/19
@ Modify instruction: New create
*******************************************************************************/
void i2cRegMstArbitrationFailedCb(DevList_e i2cDevId,DevCallBack callBackArb, void* paramArb)
{
    struct dw_i2c_dev *dev = NULL;

    /* Check driver match */
    if (!isDrvMatch(i2cDevId, DRV_ID_DW_I2C)) {
        return;
    }

    if (getDevDriver(i2cDevId,(void**)&dev) != 0) {
        return;
    }

    if (dev == NULL) {
        return;
    }

    dev->arbCb = callBackArb;
    dev->arbCbParam = paramArb;
}
