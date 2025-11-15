/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file i2c-designware-common.c
 * @author 
 * @date 2025/06/06
 * @brief none
 * @version v1.0
 */

#include <rtems.h>
#include <rtems/counter.h>
#include <rtems/bspIo.h>
#include <errno.h>
#include <bsp.h>
#include "common_defines.h"
#include "bsp_device.h"
#include "udelay.h"
#include "log_msg.h"
#include "drv_i2c_api.h"
#include "../include/i2c-designware-core.h"

///< rtems没有位数组，改成成一维数组
struct abort_sources_s {
    uint32_t abortbit;
    char *source;
};
static struct abort_sources_s g_abort_sources[] = {
    {ABRT_7B_ADDR_NOACK, "slave address not acknowledged (7bit mode)"},
    {ABRT_10ADDR1_NOACK, "first address byte not acknowledged (10bit mode)"},
    {ABRT_10ADDR2_NOACK, "second address byte not acknowledged (10bit mode)"},
    {ABRT_TXDATA_NOACK, "data not acknowledged"},
    {ABRT_GCALL_NOACK, "no acknowledgement for a general call"},
    {ABRT_GCALL_READ, "read after general call"},
    {ABRT_SBYTE_ACKDET, "start byte acknowledged"},
    {ABRT_SBYTE_NORSTRT, "trying to send start byte when restart is disabled"},
    {ABRT_10B_RD_NORSTRT, "trying to read when restart is disabled (10bit mode)"},
    {ABRT_MASTER_DIS, "trying to use disabled adapter"},
    {ARB_LOST, "lost arbitration"},
    {ABRT_SLAVE_FLUSH_TXFIFO, "read command so flush old data in the TX FIFO"},
    {ABRT_SLAVE_ARBLOST, "slave lost the bus while transmitting data to a remote master"},
    {ABRT_SLAVE_RD_INTX, "incorrect slave-transmitter mode configuration"},
};

static const U32 supported_speeds[] = {
    I2C_MAX_HIGH_SPEED_MODE_FREQ,
    I2C_MAX_FAST_MODE_PLUS_FREQ,
    I2C_MAX_FAST_MODE_FREQ,
    I2C_MAX_STANDARD_MODE_FREQ,
};

static inline int i2c_dw_validate_speed(struct dw_i2c_dev *dev)
{
    struct i2c_timings *t = &dev->timings;
    unsigned int i;

    /*
     * Only standard mode at 100kHz, fast mode at 400kHz,
     * fast mode plus at 1MHz and high speed mode at 3.4MHz are supported.
     */
    for (i = 0; i < ARRAY_SIZE(supported_speeds); i++) {
        if (t->bus_freq_hz == supported_speeds[i])
            return 0;
    }

    LOGE("%d Hz is unsupported, only 100kHz, 400kHz, 1MHz and 3.4MHz are supported\n",
            t->bus_freq_hz);

    return -EINVAL;
}

U32 i2c_dw_scl_hcnt(U32 ic_clk, U32 tSYMBOL, U32 tf, int cond, int offset)
{
    /*
     * DesignWare I2C core doesn't seem to have solid strategy to meet
     * the tHD;STA timing spec.  Configuring _HCNT based on tHIGH spec
     * will result in violation of the tHD;STA spec.
     */
    if (cond)
        /*
         * Conditional expression:
         *
         *   IC_[FS]S_SCL_HCNT + (1+4+3) >= IC_CLK * tHIGH
         *
         * This is based on the DW manuals, and represents an ideal
         * configuration.  The resulting I2C bus speed will be
         * faster than any of the others.
         *
         * If your hardware is free from tHD;STA issue, try this one.
         */
        return DIV_ROUND_CLOSEST(ic_clk * tSYMBOL, 1000000UL) - 8 + offset;
    else
        /*
         * Conditional expression:
         *
         *   IC_[FS]S_SCL_HCNT + 3 >= IC_CLK * (tHD;STA + tf)
         *
         * This is just experimental rule; the tHD;STA period turned
         * out to be proportinal to (_HCNT + 3).  With this setting,
         * we could meet both tHIGH and tHD;STA timing specs.
         *
         * If unsure, you'd better to take this alternative.
         *
         * The reason why we need to take into account "tf" here,
         * is the same as described in i2c_dw_scl_lcnt().
         */
        return DIV_ROUND_CLOSEST(ic_clk * (tSYMBOL + tf), 1000000UL) - 3 + offset;
}

U32 i2c_dw_scl_lcnt(U32 ic_clk, U32 tLOW, U32 tf, int offset)
{
    /*
     * Conditional expression:
     *
     *   IC_[FS]S_SCL_LCNT + 1 >= IC_CLK * (tLOW + tf)
     *
     * DW I2C core starts counting the SCL CNTs for the LOW period
     * of the SCL clock (tLOW) as soon as it pulls the SCL line.
     * In order to meet the tLOW timing spec, we need to take into
     * account the fall time of SCL signal (tf).  Default tf value
     * should be 0.3 us, for safety.
     */
    return DIV_ROUND_CLOSEST(ic_clk * (tLOW + tf), 1000000UL) - 1 + offset;
}

unsigned long i2c_dw_clk_rate(struct dw_i2c_dev *dev)
{
    /*
     ** Clock is not necessary if we got LCNT/HCNT values directly from
     ** the platform code.
     **/
    return DW_I2C_CLK / 1000;
}

void __i2c_dw_disable(struct dw_i2c_dev *dev)
{
    int timeout;
    U32 status;
    U32 raw_intr_stats, ic_status;
    U32 enable;
    int abort_needed;

    dw_reg_read(dev, DW_IC_RAW_INTR_STAT, &raw_intr_stats);
    dw_reg_read(dev, DW_IC_STATUS, &ic_status);
    dw_reg_read(dev, DW_IC_ENABLE, &enable);

    abort_needed = (raw_intr_stats & DW_IC_INTR_MST_ON_HOLD) || 
        (ic_status & DW_IC_STATUS_MASTER_HOLD_TX_FIFO_EMPTY);
    if (abort_needed) {
        if(!(enable & DW_IC_ENABLE_ENABLE)) {
            dw_reg_write(dev, DW_IC_ENABLE, DW_IC_ENABLE_ENABLE);
            /*
             * Wait 10 times the signaling period of the highest I2C
             * transfer supported by the driver (for 400KHz this is
             * 25us) to ensure the I2C ENABLE bit is already set
             * as described in the DesignWare I2C databook.
             */
            ///< 这里以100K的10倍计算，即100us
            rtems_counter_delay_nanoseconds(100000);
            /* Set ENABLE bit before setting ABORT */
            enable |= DW_IC_ENABLE_ENABLE;
        }
        dw_reg_write(dev, DW_IC_ENABLE, enable | DW_IC_ENABLE_ABORT);
        timeout = 10;
        while (--timeout) {
            dw_reg_read(dev, DW_IC_ENABLE, &enable);
            if (!(enable & DW_IC_ENABLE_ABORT)) {
                break;
            }
            ///< timeout: 10us * 10
            rtems_counter_delay_nanoseconds(10000);
        }
        if (!timeout)
            LOGE("timeout while trying to abort current transfer\n");
    }

    timeout = 100;
    do {
        __i2c_dw_disable_nowait(dev);
        /*
         * The enable status register may be unimplemented, but
         * in that case this test reads zero and exits the loop.
         */
        dw_reg_read(dev, DW_IC_ENABLE_STATUS, &status);
        if ((status & 1) == 0) {
            return;
        }
        /*
         * Wait 10 times the signaling period of the highest I2C
         * transfer supported by the driver (for 400KHz this is
         * 25us) as described in the DesignWare I2C databook.
         */
        rtems_counter_delay_nanoseconds(25000);
    } while (timeout--);
    LOGE("Timeout in disabling adapter(%u).\n", dev->i2c_channel_num);
}

/*
 * Waiting for bus not busy
 */
int i2c_dw_wait_bus_not_busy(struct dw_i2c_dev *dev)
{
    ///< 等待设备空闲，约40ms。linux此处是2ms
    int timeout = 2;
    U32 ic_status;

    dw_reg_read(dev, DW_IC_STATUS, &ic_status);
    while (ic_status & DW_IC_STATUS_ACTIVITY) {
        if (timeout <= 0) {
            LOGE("Timeout waiting for i2c-%u ready.\n", dev->i2c_channel_num);
            return -ETIMEDOUT;
        }
        timeout--;
        rtems_task_wake_after(2);
        dw_reg_read(dev, DW_IC_STATUS, &ic_status);
    }

    return 0;
}

int i2c_dw_handle_tx_abort(struct dw_i2c_dev *dev)
{
    unsigned long abort_source = dev->abort_source;
    int i;

    ///< 这样写的目的是不同错误返回不同的错误码
    if (abort_source & DW_IC_TX_ABRT_NOACK) {
        for (i = 0; i < ARRAY_SIZE(g_abort_sources); i++) {
            if (abort_source & (1 << g_abort_sources[i].abortbit)) {
                LOGT("%s.\n", g_abort_sources[i].source);
            }
        }
        return -EIO;
    }

    for (i = 0; i < ARRAY_SIZE(g_abort_sources); i++) {
        if (abort_source & (1 << g_abort_sources[i].abortbit)) {
            LOGE("%s.\n", g_abort_sources[i].source);
        }
    }

    if (abort_source & DW_IC_TX_ARB_LOST) {
        return -EAGAIN;
    } else if (abort_source & DW_IC_TX_ABRT_GCALL_READ) {
        return -EINVAL; /* wrong msgs[] data */
    } else {
        LOGE("tx abort other source:0x%x.", dev->abort_source);
        return -EFAULT;
    }
}

int i2c_dw_calc_fifo_size(struct dw_i2c_dev *dev)
{
    U32 param, tx_fifo_depth, rx_fifo_depth;
    int ret;

    /*
     * Try to detect the FIFO depth if not set by interface driver,
     * the depth could be from 2 to 256 from HW spec.
     */
    ret = dw_reg_read(dev, DW_IC_COMP_PARAM_1, &param);
    if (ret) {
        return ret;
    }
    tx_fifo_depth = ((param >> 16) & 0xff) + 1;
    rx_fifo_depth = ((param >> 8)  & 0xff) + 1;
    dev->tx_fifo_depth = tx_fifo_depth;
    dev->rx_fifo_depth = rx_fifo_depth;
    return 0;
}

void i2c_dw_disable(struct dw_i2c_dev *dev)
{
    U32 dummy;

    /* Disable controller */
    __i2c_dw_disable(dev);

    /* Disable all interrupts */
    dw_reg_write(dev, DW_IC_INTR_MASK, 0);
    dw_reg_read(dev, DW_IC_CLR_INTR, &dummy);
}
