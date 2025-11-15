/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file spi-cadence-quadspi.c
 * @brief
 * @author tianye
 * @version
 * @date 2024-05-18
 */

#include <rtems.h>
#include <rtems/sysinit.h>
#include <rtems/score/sysstate.h>
#include <rtems/counter.h>
#include <rtems/bspIo.h>
#include <rtems/irq-extension.h>
#include <bsp.h>
#include <bsp/irq.h>
#include <bsp/drv_spi_device.h>
#include <errno.h>
#include <string.h>
#include "spi-cadence-quadspi.h"
#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"

#define cqspi_reg_read(reg_addr)            (*(volatile unsigned int *)(reg_addr))
#define cqspi_reg_write(val, reg_addr)      (*(volatile unsigned int *)(reg_addr) = val)

extern struct cqspi_controller g_cqspi_dev;

static uint32_t g_cqspi_reg_info[CQSPI_REG_CNT] = {0};
static unsigned char g_cqspi_reg_offset[] = {
    0x00,0x04,0x08,0x0C,0x10,0x14,0x18,0x1C,
    0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,
    0x40,0x44,0x50,0x54,0x58,0x60,0x64,0x68,
    0x6C,0x70,0x74,0x78,0x7C,0x80,0x8C,0x90,
    0x94,0xA0,0xA4,0xA8,0xAC,0xB0,0xFC
};

static uint32_t qflashClkGet(void)
{
    uint32_t clk = 0;

    if (EXIT_SUCCESS != peripsClockFreqGet(DEVICE_QSPI0, &clk)) {
        errno = ETXTBSY;
    }
    return clk;
}

static int32_t qflashClkEnable(void)
{
    return peripsClockEnable(DEVICE_QSPI0);
}

static void qflashReset(void)
{
    peripsReset(DEVICE_QSPI0);
}

static int cqspi_wait_for_bit(volatile u32 *reg, const u32 mask, bool clr)
{
    u64 timeout_us = CQSPI_TIMEOUT_MS * USEC_PER_MSEC;
    u32 val;

    for(;;) {
        val = cqspi_reg_read(reg);
        if(((clr ? ~val : val) & mask) == mask) {
            break;
        }
        ///< 延时1us
        rtems_counter_delay_nanoseconds(1000);
        timeout_us--;
        if (timeout_us == 0) {
            val = cqspi_reg_read(reg);
            break;
        }
    }
    if(((clr ? ~val : val) & mask) == mask) {
        return 0;
    } else {
        spi_err("Wait for reg(%p-0x%08x) bit(0x%08x) clr(%u) timeout.\n", reg, val, mask, clr);
        return -ETIMEDOUT;
    }
}

static bool cqspi_is_idle(struct cqspi_controller *cqspi)
{
	u32 reg = cqspi_reg_read(cqspi->iobase + CQSPI_REG_CONFIG);

	return reg & (1UL << CQSPI_REG_CONFIG_IDLE_LSB);
}

static u32 cqspi_get_rd_sram_level(struct cqspi_controller *cqspi)
{
	u32 reg = cqspi_reg_read(cqspi->iobase + CQSPI_REG_SDRAMLEVEL);
	reg >>= CQSPI_REG_SDRAMLEVEL_RD_LSB;

	return reg & CQSPI_REG_SDRAMLEVEL_RD_MASK;
}

static void cqspi_irq_handler(void *dev)
{
	struct cqspi_controller *cqspi = dev;
	unsigned int irq_status;

	/* Read interrupt status */
	irq_status = cqspi_reg_read(cqspi->iobase + CQSPI_REG_IRQSTATUS);
	/* Clear interrupt */
	cqspi_reg_write(irq_status, cqspi->iobase + CQSPI_REG_IRQSTATUS);
	if (!cqspi->slow_sram)
		irq_status &= CQSPI_IRQ_MASK_RD | CQSPI_IRQ_MASK_WR;
	else
		irq_status &= CQSPI_REG_IRQ_WATERMARK | CQSPI_IRQ_MASK_WR;
	if (irq_status) {
        rtems_semaphore_release(cqspi->semaphore_id);
    }

	return;
}

static unsigned int cqspi_calc_rdreg(const struct spi_mem_op *op)
{
	u32 rdreg = 0;

	rdreg |= CQSPI_OP_WIDTH(op->cmd) << CQSPI_REG_RD_INSTR_TYPE_INSTR_LSB;
	rdreg |= CQSPI_OP_WIDTH(op->addr) << CQSPI_REG_RD_INSTR_TYPE_ADDR_LSB;
	rdreg |= CQSPI_OP_WIDTH(op->data) << CQSPI_REG_RD_INSTR_TYPE_DATA_LSB;

	return rdreg;
}

static unsigned int cqspi_calc_dummy(const struct spi_mem_op *op)
{
	unsigned int dummy_clk;

	if (!op->dummy.nbytes)
		return 0;

	dummy_clk = op->dummy.nbytes * (8 / op->dummy.buswidth);
	if (op->cmd.dtr)
		dummy_clk /= 2;

	return dummy_clk;
}

static int cqspi_wait_idle(struct cqspi_controller *cqspi)
{
    u64 timeout_us = CQSPI_TIMEOUT_MS * USEC_PER_MSEC;
    int ret = 0;

    ///< rtems不能用这种次数加超时时间，因为任务切出去后切回来的时间不可预期！！！！
    while (cqspi_is_idle(cqspi) != true) {
        rtems_counter_delay_nanoseconds(1000);
        timeout_us--;
        if (timeout_us == 0) {
            if (cqspi_is_idle(cqspi)) {
                ret = 0;
                goto exit;
            } else {
                ret = -1;
                spi_err("Qspi is still busy after %dms timeout.\n", CQSPI_TIMEOUT_MS);
                goto exit;
            }
        }

    }

exit:
    return ret;
}

static int cqspi_exec_flash_cmd(struct cqspi_controller *cqspi, unsigned int reg)
{
	void *reg_base = cqspi->iobase;
	int ret;

	/* Write the CMDCTRL without start execution. */
	cqspi_reg_write(reg, reg_base + CQSPI_REG_CMDCTRL);
	/* Start execute */
	reg |= CQSPI_REG_CMDCTRL_EXECUTE_MASK;
	cqspi_reg_write(reg, reg_base + CQSPI_REG_CMDCTRL);

	/* Polling for completion. */
	ret = cqspi_wait_for_bit(reg_base + CQSPI_REG_CMDCTRL,
				 CQSPI_REG_CMDCTRL_INPROGRESS_MASK, 1);
	if (ret) {
        spi_err("Flash command execute timed out.\n");
		return ret;
	}

	/* Polling QSPI idle status. */
	return cqspi_wait_idle(cqspi);
}

static int cqspi_setup_opcode_ext(struct cqspi_flash_pdata *f_pdata,
				  const struct spi_mem_op *op,
				  unsigned int shift)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;
	void *reg_base = cqspi->iobase;
	unsigned int reg;
	u8 ext;

	if (op->cmd.nbytes != 2)
		return -EINVAL;

	/* Opcode extension is the LSB. */
	ext = op->cmd.opcode & 0xff;

	reg = cqspi_reg_read(reg_base + CQSPI_REG_OP_EXT_LOWER);
	reg &= ~(0xff << shift);
	reg |= ext << shift;
	cqspi_reg_write(reg, reg_base + CQSPI_REG_OP_EXT_LOWER);

	return 0;
}

static int cqspi_enable_dtr(struct cqspi_flash_pdata *f_pdata,
			    const struct spi_mem_op *op, unsigned int shift)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;
	void *reg_base = cqspi->iobase;
	unsigned int reg;
	int ret;

	reg = cqspi_reg_read(reg_base + CQSPI_REG_CONFIG);

	/*
	 * We enable dual byte opcode here. The callers have to set up the
	 * extension opcode based on which type of operation it is.
	 */
	if (op->cmd.dtr) {
		reg |= CQSPI_REG_CONFIG_DTR_PROTO;
		reg |= CQSPI_REG_CONFIG_DUAL_OPCODE;

		/* Set up command opcode extension. */
		ret = cqspi_setup_opcode_ext(f_pdata, op, shift);
		if (ret)
			return ret;
    } else {
        unsigned int mask = CQSPI_REG_CONFIG_DTR_PROTO | CQSPI_REG_CONFIG_DUAL_OPCODE;
        /* Shortcut if DTR is already disabled. */
        if ((reg & mask) == 0)
            return 0;
        reg &= ~mask;
    }

	cqspi_reg_write(reg, reg_base + CQSPI_REG_CONFIG);

	return cqspi_wait_idle(cqspi);
}

static int cqspi_command_read(struct cqspi_flash_pdata *f_pdata,
			      const struct spi_mem_op *op)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;
	void *reg_base = cqspi->iobase;
	u8 *rxbuf = op->data.buf.in;
	u8 opcode;
	size_t n_rx = op->data.nbytes;
	unsigned int rdreg;
	unsigned int reg;
	unsigned int dummy_clk;
	size_t read_len;
	int status;

	status = cqspi_enable_dtr(f_pdata, op, CQSPI_REG_OP_EXT_STIG_LSB);
    if (status) {
        return status;
    }

	if (!n_rx || n_rx > CQSPI_STIG_DATA_LEN_MAX || !rxbuf) {
        spi_err("Invalid input argument, len %zu rxbuf 0x%p\n", n_rx, rxbuf);
		return -EINVAL;
	}

	if (op->cmd.dtr)
		opcode = op->cmd.opcode >> 8;
	else
		opcode = op->cmd.opcode;

	reg = opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;

	rdreg = cqspi_calc_rdreg(op);
	cqspi_reg_write(rdreg, reg_base + CQSPI_REG_RD_INSTR);

	dummy_clk = cqspi_calc_dummy(op);
	if (dummy_clk > CQSPI_DUMMY_CLKS_MAX)
		return -EOPNOTSUPP;

	if (dummy_clk)
		reg |= (dummy_clk & CQSPI_REG_CMDCTRL_DUMMY_MASK)
		     << CQSPI_REG_CMDCTRL_DUMMY_LSB;

	reg |= (0x1 << CQSPI_REG_CMDCTRL_RD_EN_LSB);

	/* 0 means 1 byte. */
	reg |= (((n_rx - 1) & CQSPI_REG_CMDCTRL_RD_BYTES_MASK)
		<< CQSPI_REG_CMDCTRL_RD_BYTES_LSB);

	/* setup ADDR BIT field */
	if (op->addr.nbytes) {
		reg |= (0x1 << CQSPI_REG_CMDCTRL_ADDR_EN_LSB);
		reg |= ((op->addr.nbytes - 1) &
			CQSPI_REG_CMDCTRL_ADD_BYTES_MASK)
			<< CQSPI_REG_CMDCTRL_ADD_BYTES_LSB;

		cqspi_reg_write(op->addr.val, reg_base + CQSPI_REG_CMDADDRESS);
	}

	status = cqspi_exec_flash_cmd(cqspi, reg);
	if (status)
		return status;

	reg = cqspi_reg_read(reg_base + CQSPI_REG_CMDREADDATALOWER);

	/* Put the read value into rx_buf */
	read_len = (n_rx > 4) ? 4 : n_rx;
	memcpy(rxbuf, &reg, read_len);
	rxbuf += read_len;

	if (n_rx > 4) {
		reg = cqspi_reg_read(reg_base + CQSPI_REG_CMDREADDATAUPPER);

		read_len = n_rx - read_len;
		memcpy(rxbuf, &reg, read_len);
	}

	/* Reset CMD_CTRL Reg once command read completes */
	cqspi_reg_write(0, reg_base + CQSPI_REG_CMDCTRL);

	return 0;
}

static int cqspi_command_write(struct cqspi_flash_pdata *f_pdata,
			       const struct spi_mem_op *op)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;
	void *reg_base = cqspi->iobase;
	u8 opcode;
	const u8 *txbuf = op->data.buf.out;
	size_t n_tx = op->data.nbytes;
	unsigned int reg;
	unsigned int data;
	size_t write_len;
	int ret;

	ret = cqspi_enable_dtr(f_pdata, op, CQSPI_REG_OP_EXT_STIG_LSB);
	if (ret)
		return ret;

	if (n_tx > CQSPI_STIG_DATA_LEN_MAX || (n_tx && !txbuf)) {
        spi_err("Invalid input argument, cmdlen %zu txbuf 0x%p\n",
                n_tx, txbuf);
		return -EINVAL;
	}

	reg = cqspi_calc_rdreg(op);
	cqspi_reg_write(reg, reg_base + CQSPI_REG_RD_INSTR);

	if (op->cmd.dtr)
		opcode = op->cmd.opcode >> 8;
	else
		opcode = op->cmd.opcode;

	reg = opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;

	if (op->addr.nbytes) {
		reg |= (0x1 << CQSPI_REG_CMDCTRL_ADDR_EN_LSB);
		reg |= ((op->addr.nbytes - 1) &
			CQSPI_REG_CMDCTRL_ADD_BYTES_MASK)
			<< CQSPI_REG_CMDCTRL_ADD_BYTES_LSB;

		cqspi_reg_write(op->addr.val, reg_base + CQSPI_REG_CMDADDRESS);
	}

	if (n_tx) {
		reg |= (0x1 << CQSPI_REG_CMDCTRL_WR_EN_LSB);
		reg |= ((n_tx - 1) & CQSPI_REG_CMDCTRL_WR_BYTES_MASK)
			<< CQSPI_REG_CMDCTRL_WR_BYTES_LSB;
		data = 0;
		write_len = (n_tx > 4) ? 4 : n_tx;
		memcpy(&data, txbuf, write_len);
		txbuf += write_len;
		cqspi_reg_write(data, reg_base + CQSPI_REG_CMDWRITEDATALOWER);

		if (n_tx > 4) {
			data = 0;
			write_len = n_tx - 4;
			memcpy(&data, txbuf, write_len);
			cqspi_reg_write(data, reg_base + CQSPI_REG_CMDWRITEDATAUPPER);
		}
	}

	ret = cqspi_exec_flash_cmd(cqspi, reg);

	/* Reset CMD_CTRL Reg once command write completes */
	cqspi_reg_write(0, reg_base + CQSPI_REG_CMDCTRL);

	return ret;
}

static int cqspi_read_setup(struct cqspi_flash_pdata *f_pdata,
			    const struct spi_mem_op *op)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;
	void *reg_base = cqspi->iobase;
	unsigned int dummy_clk = 0;
	unsigned int reg;
	int ret;
	u8 opcode;

	ret = cqspi_enable_dtr(f_pdata, op, CQSPI_REG_OP_EXT_READ_LSB);
	if (ret)
		return ret;

	if (op->cmd.dtr)
		opcode = op->cmd.opcode >> 8;
	else
		opcode = op->cmd.opcode;

	reg = opcode << CQSPI_REG_RD_INSTR_OPCODE_LSB;
	reg |= cqspi_calc_rdreg(op);

	/* Setup dummy clock cycles */
	dummy_clk = cqspi_calc_dummy(op);

	if (dummy_clk > CQSPI_DUMMY_CLKS_MAX)
		return -EOPNOTSUPP;

	if (dummy_clk)
		reg |= (dummy_clk & CQSPI_REG_RD_INSTR_DUMMY_MASK)
		       << CQSPI_REG_RD_INSTR_DUMMY_LSB;

	cqspi_reg_write(reg, reg_base + CQSPI_REG_RD_INSTR);

	/* Set address width */
	reg = cqspi_reg_read(reg_base + CQSPI_REG_SIZE);
	reg &= ~CQSPI_REG_SIZE_ADDRESS_MASK;
	reg |= (op->addr.nbytes - 1);
	cqspi_reg_write(reg, reg_base + CQSPI_REG_SIZE);
	return 0;
}

static int cqspi_indirect_read_execute(struct cqspi_flash_pdata *f_pdata,
				       u8 *rxbuf, u64 from_addr,
				       const size_t n_rx)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;
    bool use_irq = !(cqspi->ddata && cqspi->ddata->quirks & CQSPI_RD_NO_IRQ);
	void *reg_base = cqspi->iobase;
	void *ahb_base = cqspi->ahb_base;
	unsigned int remaining = n_rx;
	unsigned int mod_bytes = n_rx % 4;
	unsigned int bytes_to_read = 0;
	u8 *rxbuf_end = rxbuf + n_rx;
	int ret = 0;
    int i;

	cqspi_reg_write(from_addr, reg_base + CQSPI_REG_INDIRECTRDSTARTADDR);
	cqspi_reg_write(remaining, reg_base + CQSPI_REG_INDIRECTRDBYTES);

	/* Clear all interrupts. */
	cqspi_reg_write(CQSPI_IRQ_STATUS_MASK, reg_base + CQSPI_REG_IRQSTATUS);

	/*
	 * On SoCFPGA platform reading the SRAM is slow due to
	 * hardware limitation and causing read interrupt storm to CPU,
	 * so enabling only watermark interrupt to disable all read
	 * interrupts later as we want to run "bytes to read" loop with
	 * all the read interrupts disabled for max performance.
	 */

    if (use_irq && cqspi->slow_sram)
		cqspi_reg_write(CQSPI_REG_IRQ_WATERMARK, reg_base + CQSPI_REG_IRQMASK);
    else if (use_irq)
		cqspi_reg_write(CQSPI_IRQ_MASK_RD, reg_base + CQSPI_REG_IRQMASK);
	else
		cqspi_reg_write(0, reg_base + CQSPI_REG_IRQMASK);

    while(RTEMS_SUCCESSFUL == rtems_semaphore_obtain(cqspi->semaphore_id, RTEMS_NO_WAIT, 0)) {
        ;
    }
	cqspi_reg_write(CQSPI_REG_INDIRECTRD_START_MASK,
	       reg_base + CQSPI_REG_INDIRECTRD);

	while (remaining > 0) {
        if(use_irq &&
                (rtems_semaphore_obtain(cqspi->semaphore_id, RTEMS_WAIT,
                                        RTEMS_MILLISECONDS_TO_TICKS(CQSPI_READ_TIMEOUT_MS)) != RTEMS_SUCCESSFUL)) {
            spi_info("CQSPI controller timed out.\n");
            ret = -ETIMEDOUT;
        }

		/*
		 * Disable all read interrupts until
		 * we are out of "bytes to read"
		 */
		if (cqspi->slow_sram)
			cqspi_reg_write(0x0, reg_base + CQSPI_REG_IRQMASK);

		bytes_to_read = cqspi_get_rd_sram_level(cqspi);

		if (ret && bytes_to_read == 0) {
			spi_err("Indirect read timeout, no bytes\n");
			goto failrd;
		}

		while (bytes_to_read != 0) {
			unsigned int word_remain = round_down(remaining, 4);

			bytes_to_read *= cqspi->fifo_width;
			bytes_to_read = bytes_to_read > remaining ?
					remaining : bytes_to_read;
			bytes_to_read = round_down(bytes_to_read, 4);
			/* Read 4 byte word chunks then single bytes */
			if (bytes_to_read) {
                for (i = 0; i < (bytes_to_read / 4); ++i) {
                    *(u32 *)(rxbuf + i * 4) = *(volatile u32 *)ahb_base;
                }
			} else if (!word_remain && mod_bytes) {
				unsigned int temp = *(volatile u32 *)ahb_base;

				bytes_to_read = mod_bytes;
				memcpy(rxbuf, &temp, min((unsigned int)(rxbuf_end - rxbuf),bytes_to_read));
			}
			rxbuf += bytes_to_read;
			remaining -= bytes_to_read;
			bytes_to_read = cqspi_get_rd_sram_level(cqspi);
		}

		if (use_irq && (remaining > 0)) {
            while(RTEMS_SUCCESSFUL == rtems_semaphore_obtain(cqspi->semaphore_id, RTEMS_NO_WAIT, 0)) {
                ;
            }
			if (cqspi->slow_sram)
				cqspi_reg_write(CQSPI_REG_IRQ_WATERMARK, reg_base + CQSPI_REG_IRQMASK);
		}
	}

	/* Check indirect done status */
	ret = cqspi_wait_for_bit(reg_base + CQSPI_REG_INDIRECTRD,
				 CQSPI_REG_INDIRECTRD_DONE_MASK, 0);
	if (ret) {
		spi_err("Indirect read completion error (%i)\n", ret);
		goto failrd;
	}

	/* Disable interrupt */
	cqspi_reg_write(0, reg_base + CQSPI_REG_IRQMASK);

	/* Clear indirect completion status */
	cqspi_reg_write(CQSPI_REG_INDIRECTRD_DONE_MASK, reg_base + CQSPI_REG_INDIRECTRD);

	return 0;

failrd:
	/* Disable interrupt */
	cqspi_reg_write(0, reg_base + CQSPI_REG_IRQMASK);

	/* Cancel the indirect read */
	cqspi_reg_write(CQSPI_REG_INDIRECTRD_CANCEL_MASK,
	       reg_base + CQSPI_REG_INDIRECTRD);
	return ret;
}

static void cqspi_controller_enable(struct cqspi_controller *cqspi, bool enable)
{
	void *reg_base = cqspi->iobase;
	unsigned int reg;

	reg = cqspi_reg_read(reg_base + CQSPI_REG_CONFIG);

	if (enable)
		reg |= CQSPI_REG_CONFIG_ENABLE_MASK;
	else
		reg &= ~CQSPI_REG_CONFIG_ENABLE_MASK;

	cqspi_reg_write(reg, reg_base + CQSPI_REG_CONFIG);
}

static int cqspi_write_setup(struct cqspi_flash_pdata *f_pdata,
			     const struct spi_mem_op *op)
{
	unsigned int reg;
	int ret;
    struct cqspi_controller *cqspi = f_pdata->cqspi;
	void *reg_base = cqspi->iobase;
	u8 opcode;

	ret = cqspi_enable_dtr(f_pdata, op, CQSPI_REG_OP_EXT_WRITE_LSB);
	if (ret)
		return ret;

	if (op->cmd.dtr)
		opcode = op->cmd.opcode >> 8;
	else
		opcode = op->cmd.opcode;

	/* Set opcode. */
	reg = opcode << CQSPI_REG_WR_INSTR_OPCODE_LSB;
	reg |= CQSPI_OP_WIDTH(op->data) << CQSPI_REG_WR_INSTR_TYPE_DATA_LSB;
	reg |= CQSPI_OP_WIDTH(op->addr) << CQSPI_REG_WR_INSTR_TYPE_ADDR_LSB;
	cqspi_reg_write(reg, reg_base + CQSPI_REG_WR_INSTR);
	reg = cqspi_calc_rdreg(op);
	cqspi_reg_write(reg, reg_base + CQSPI_REG_RD_INSTR);

	/*
	 * SPI NAND flashes require the address of the status register to be
	 * passed in the Read SR command. Also, some SPI NOR flashes like the
	 * cypress Semper flash expect a 4-byte dummy address in the Read SR
	 * command in DTR mode.
	 *
	 * But this controller does not support address phase in the Read SR
	 * command when doing auto-HW polling. So, disable write completion
	 * polling on the controller's side. spinand and spi-nor will take
	 * care of polling the status register.
	 */
	if (cqspi->wr_completion) {
		reg = cqspi_reg_read(reg_base + CQSPI_REG_WR_COMPLETION_CTRL);
		reg |= CQSPI_REG_WR_DISABLE_AUTO_POLL;
		cqspi_reg_write(reg, reg_base + CQSPI_REG_WR_COMPLETION_CTRL);
		/*
		 * DAC mode require auto polling as flash needs to be polled
		 * for write completion in case of bubble in SPI transaction
		 * due to slow CPU/DMA master.
		 */
		cqspi->use_direct_mode_wr = false;
	}

	reg = cqspi_reg_read(reg_base + CQSPI_REG_SIZE);
	reg &= ~CQSPI_REG_SIZE_ADDRESS_MASK;
	reg |= (op->addr.nbytes - 1);
	cqspi_reg_write(reg, reg_base + CQSPI_REG_SIZE);
	return 0;
}

static int cqspi_indirect_write_execute(struct cqspi_flash_pdata *f_pdata,
        u64 to_addr, const u8 *txbuf,
        const size_t n_tx)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;
    bool use_irq = !(cqspi->ddata && cqspi->ddata->quirks & CQSPI_WR_NO_IRQ);
    void *reg_base = cqspi->iobase;
    unsigned int remaining = n_tx;
    unsigned int write_bytes;
    int ret;
    int i;

    cqspi_reg_write(to_addr, reg_base + CQSPI_REG_INDIRECTWRSTARTADDR);
    cqspi_reg_write(remaining, reg_base + CQSPI_REG_INDIRECTWRBYTES);

    /* Clear all interrupts. */
    cqspi_reg_write(CQSPI_IRQ_STATUS_MASK, reg_base + CQSPI_REG_IRQSTATUS);

    if (use_irq)
		cqspi_reg_write(CQSPI_IRQ_MASK_WR, reg_base + CQSPI_REG_IRQMASK);
	else
		cqspi_reg_write(0, reg_base + CQSPI_REG_IRQMASK);

    while(RTEMS_SUCCESSFUL == rtems_semaphore_obtain(cqspi->semaphore_id, RTEMS_NO_WAIT, 0)) {
        ;
    }
    cqspi_reg_write(CQSPI_REG_INDIRECTWR_START_MASK,
            reg_base + CQSPI_REG_INDIRECTWR);
    /*
     * As per 66AK2G02 TRM SPRUHY8F section 11.15.5.3 Indirect Access
     * Controller programming sequence, couple of cycles of
     * QSPI_REF_CLK delay is required for the above bit to
     * be internally synchronized by the QSPI module. Provide 5
     * cycles of delay.
     */
    if (cqspi->wr_delay) {
        rtems_counter_delay_nanoseconds(cqspi->wr_delay);
    }

    /*
     * If a hazard exists between the APB and AHB interfaces, perform a
     * dummy readback from the controller to ensure synchronization.
     */
    if (cqspi->apb_ahb_hazard) {
        cqspi_reg_read(reg_base + CQSPI_REG_INDIRECTWR);
    }

    while (remaining > 0) {
        size_t write_words, mod_bytes;

        write_bytes = remaining;
        write_words = write_bytes / 4;
        mod_bytes = write_bytes % 4;
        /* Write 4 bytes at a time then single bytes. */
        if (write_words) {
            for (i = 0; i < write_words; ++i) {
                *(volatile u32 *)(cqspi->ahb_base) = *(u32 *)(txbuf + i * 4);
            }
            txbuf += (write_words * 4);
        }
        if (mod_bytes) {
            unsigned int temp = 0xFFFFFFFF;

            memcpy(&temp, txbuf, mod_bytes);
            *(volatile u32 *)(cqspi->ahb_base) = temp;
            txbuf += mod_bytes;
        }

        if(use_irq &&
                (rtems_semaphore_obtain(cqspi->semaphore_id, RTEMS_WAIT,
                                        RTEMS_MILLISECONDS_TO_TICKS(CQSPI_TIMEOUT_MS)) != RTEMS_SUCCESSFUL)) {
            spi_err("CQSPI controller timed out.\n");
            ret = -ETIMEDOUT;
            goto failwr;
        }

        remaining -= write_bytes;

		if (use_irq && (remaining > 0))
            while(RTEMS_SUCCESSFUL == rtems_semaphore_obtain(cqspi->semaphore_id, RTEMS_NO_WAIT, 0)) {
                ;
            }
    }

    /* Check indirect done status */
    ret = cqspi_wait_for_bit(reg_base + CQSPI_REG_INDIRECTWR,
            CQSPI_REG_INDIRECTWR_DONE_MASK, 0);
    if (ret) {
        spi_err("Indirect write completion error (%i)\n", ret);
        goto failwr;
    }

    /* Disable interrupt. */
    cqspi_reg_write(0, reg_base + CQSPI_REG_IRQMASK);

    /* Clear indirect completion status */
    cqspi_reg_write(CQSPI_REG_INDIRECTWR_DONE_MASK, reg_base + CQSPI_REG_INDIRECTWR);

    cqspi_wait_idle(cqspi);

    return 0;

failwr:
    /* Disable interrupt. */
    cqspi_reg_write(0, reg_base + CQSPI_REG_IRQMASK);

    /* Cancel the indirect write */
    cqspi_reg_write(CQSPI_REG_INDIRECTWR_CANCEL_MASK,
            reg_base + CQSPI_REG_INDIRECTWR);
    return ret;
}

static void cqspi_chipselect(struct cqspi_flash_pdata *f_pdata)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;
	void *reg_base = cqspi->iobase;
	unsigned int chip_select = f_pdata->cs;
	unsigned int reg;

	reg = cqspi_reg_read(reg_base + CQSPI_REG_CONFIG);
	if (cqspi->is_decoded_cs) {
		reg |= CQSPI_REG_CONFIG_DECODE_MASK;
	} else {
		reg &= ~CQSPI_REG_CONFIG_DECODE_MASK;

		/* Convert CS if without decoder.
		 * CS0 to 4b'1110
		 * CS1 to 4b'1101
		 * CS2 to 4b'1011
		 * CS3 to 4b'0111
		 */
		chip_select = 0xF & ~(1 << chip_select);
	}

	reg &= ~(CQSPI_REG_CONFIG_CHIPSELECT_MASK
		 << CQSPI_REG_CONFIG_CHIPSELECT_LSB);
	reg |= (chip_select & CQSPI_REG_CONFIG_CHIPSELECT_MASK)
	    << CQSPI_REG_CONFIG_CHIPSELECT_LSB;
	cqspi_reg_write(reg, reg_base + CQSPI_REG_CONFIG);
}

static unsigned int calculate_ticks_for_ns(const unsigned int ref_clk_hz,
					   const unsigned int ns_val)
{
	unsigned int ticks;

	ticks = ref_clk_hz / 1000;	/* kHz */
	ticks = DIV_ROUND_UP(ticks * ns_val, 1000000);

	return ticks;
}

static void cqspi_delay(struct cqspi_flash_pdata *f_pdata)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;
	void *iobase = cqspi->iobase;
	const unsigned int ref_clk_hz = cqspi->master_ref_clk_hz;
	unsigned int tshsl, tchsh, tslch, tsd2d;
	unsigned int reg;
	unsigned int tsclk;

	/* calculate the number of ref ticks for one sclk tick */
	tsclk = DIV_ROUND_UP(ref_clk_hz, cqspi->sclk);

	tshsl = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tshsl_ns);
	/* this particular value must be at least one sclk */
	if (tshsl < tsclk)
		tshsl = tsclk;

	tchsh = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tchsh_ns);
	tslch = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tslch_ns);
	tsd2d = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tsd2d_ns);

	reg = (tshsl & CQSPI_REG_DELAY_TSHSL_MASK)
	       << CQSPI_REG_DELAY_TSHSL_LSB;
	reg |= (tchsh & CQSPI_REG_DELAY_TCHSH_MASK)
		<< CQSPI_REG_DELAY_TCHSH_LSB;
	reg |= (tslch & CQSPI_REG_DELAY_TSLCH_MASK)
		<< CQSPI_REG_DELAY_TSLCH_LSB;
	reg |= (tsd2d & CQSPI_REG_DELAY_TSD2D_MASK)
		<< CQSPI_REG_DELAY_TSD2D_LSB;
	cqspi_reg_write(reg, iobase + CQSPI_REG_DELAY);
}

static void cqspi_config_baudrate_div(struct cqspi_controller *cqspi)
{
	const unsigned int ref_clk_hz = cqspi->master_ref_clk_hz;
	void *reg_base = cqspi->iobase;
	u32 reg, div;

	/* Recalculate the baudrate divisor based on QSPI specification. */
	div = DIV_ROUND_UP(ref_clk_hz, 2 * cqspi->sclk) - 1;

	/* Maximum baud divisor */
	if (div > CQSPI_REG_CONFIG_BAUD_MASK) {
		div = CQSPI_REG_CONFIG_BAUD_MASK;
		spi_info("Unable to adjust clock <= %d hz. Reduced to %d hz\n",
			cqspi->sclk, ref_clk_hz/((div+1)*2));
	}

	reg = cqspi_reg_read(reg_base + CQSPI_REG_CONFIG);
	reg &= ~(CQSPI_REG_CONFIG_BAUD_MASK << CQSPI_REG_CONFIG_BAUD_LSB);
	reg |= (div & CQSPI_REG_CONFIG_BAUD_MASK) << CQSPI_REG_CONFIG_BAUD_LSB;
	cqspi_reg_write(reg, reg_base + CQSPI_REG_CONFIG);
}

static void cqspi_readdata_capture(struct cqspi_controller *cqspi,
				   const bool bypass,
				   const unsigned int delay)
{
	void *reg_base = cqspi->iobase;
	unsigned int reg;

	reg = cqspi_reg_read(reg_base + CQSPI_REG_READCAPTURE);

	if (bypass)
		reg |= (1 << CQSPI_REG_READCAPTURE_BYPASS_LSB);
	else
		reg &= ~(1 << CQSPI_REG_READCAPTURE_BYPASS_LSB);

	reg &= ~(CQSPI_REG_READCAPTURE_DELAY_MASK
		 << CQSPI_REG_READCAPTURE_DELAY_LSB);

	reg |= (delay & CQSPI_REG_READCAPTURE_DELAY_MASK)
		<< CQSPI_REG_READCAPTURE_DELAY_LSB;

	cqspi_reg_write(reg, reg_base + CQSPI_REG_READCAPTURE);
}

static void cqspi_configure(struct cqspi_flash_pdata *f_pdata,
			    unsigned long sclk)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;
	int switch_cs = (cqspi->current_cs != f_pdata->cs);
	int switch_ck = (cqspi->sclk != sclk);

	if (switch_cs || switch_ck)
		cqspi_controller_enable(cqspi, 0);

	/* Switch chip select. */
	if (switch_cs) {
		cqspi->current_cs = f_pdata->cs;
		cqspi_chipselect(f_pdata);
	}

	/* Setup baudrate divisor and delays */
	if (switch_ck) {
		cqspi->sclk = sclk;
		cqspi_config_baudrate_div(cqspi);
		cqspi_delay(f_pdata);
		cqspi_readdata_capture(cqspi, !cqspi->rclk_en,
				       f_pdata->read_delay);
	}

	if (switch_cs || switch_ck)
		cqspi_controller_enable(cqspi, 1);
}

static ssize_t cqspi_write(struct cqspi_flash_pdata *f_pdata,
			   const struct spi_mem_op *op)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;
	u64 to = op->addr.val;
	unsigned int len = op->data.nbytes;
	const u8 *buf = op->data.buf.out;
	int ret;

	ret = cqspi_write_setup(f_pdata, op);
	if (ret)
		return ret;

	/*
	 * Some flashes like the Cypress Semper flash expect a dummy 4-byte
	 * address (all 0s) with the read status register command in DTR mode.
	 * But this controller does not support sending dummy address bytes to
	 * the flash when it is polling the write completion register in DTR
	 * mode. So, we can not use direct mode when in DTR mode for writing
	 * data.
	 */
	if (!op->cmd.dtr && cqspi->use_direct_mode &&
	    cqspi->use_direct_mode_wr && ((to + len) <= cqspi->ahb_size)) {
		for (int i = 0; i < len; i++) {
			((u8 *)(cqspi->ahb_base + to))[i] = buf[i];
		}
		// memcpy(cqspi->ahb_base + to, buf, len);
		return cqspi_wait_idle(cqspi);
	}

	return cqspi_indirect_write_execute(f_pdata, to, buf, len);
}

///< 暂时不支持dma直接读
static int cqspi_direct_read_execute(struct cqspi_flash_pdata *f_pdata,
				     u8 *buf, u64 from, unsigned int len)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;

	if (buf != NULL) {
		for (int i = 0; i < len; i++) {
			buf[i] = ((u8 *)(cqspi->ahb_base + from))[i];
		}
		// memcpy(buf, cqspi->ahb_base + from, len);
		return 0;
	}

    return -EIO;
}

static ssize_t cqspi_read(struct cqspi_flash_pdata *f_pdata,
			  const struct spi_mem_op *op)
{
    struct cqspi_controller *cqspi = f_pdata->cqspi;
	u64 from = op->addr.val;
	unsigned int len = op->data.nbytes;
	u8 *buf = op->data.buf.in;
	int ret;

	ret = cqspi_read_setup(f_pdata, op);
	if (ret)
		return ret;

	if (cqspi->use_direct_mode && ((from + len) <= cqspi->ahb_size))
		return cqspi_direct_read_execute(f_pdata, buf, from, len);

	return cqspi_indirect_read_execute(f_pdata, buf, from, len);
}

static int cqspi_mem_process(struct spi_nor *nor, const struct spi_mem_op *op)
{
    struct cqspi_controller *cqspi = nor->cqspi;
	struct cqspi_flash_pdata *f_pdata;

	f_pdata = &cqspi->f_pdata[nor->channel_id];
	cqspi_configure(f_pdata, f_pdata->clk_rate);

	if (op->data.dir == SPI_MEM_DATA_IN && op->data.buf.in) {
		if (!op->addr.nbytes)
        {
			return cqspi_command_read(f_pdata, op);
        }

		return cqspi_read(f_pdata, op);
	}

	if (!op->addr.nbytes || !op->data.buf.out)
    {
		return cqspi_command_write(f_pdata, op);
    }

	return cqspi_write(f_pdata, op);
}

#define REG_INFO_LINE_CTRL  (8)
static void cqspi_reg_capture(struct cqspi_controller *cqspi)
{
    int32_t i = 0;
    for (i = 0; i < CQSPI_REG_CNT; i++) {
        g_cqspi_reg_info[i] = cqspi_reg_read(cqspi->iobase + g_cqspi_reg_offset[i]);
    }
    printk("[CQSPI controller registers:]\n");
    for (i = 0; i < CQSPI_REG_CNT; i++) {
        (void)printk("0x%08x", g_cqspi_reg_info[i]);
        if ((i + 1) % REG_INFO_LINE_CTRL == 0) {
            (void)printk("\n");
        } else {
            (void)printk(" ");
        }
    }
}

int qspi_exec_mem_op(struct spi_nor *nor, const struct spi_mem_op *op)
{
	int ret;
    struct cqspi_controller *cqspi = nor->cqspi;

    if (_System_all_cpu_state_Shutdown_Get() == false) {
        if (nor->trylock == true) {
            ret = _Mutex_Try_acquire(&(cqspi->bus_lock_mutex));
            if (ret != 0) {
                ret = -EAGAIN;
                goto exit;
            }
        } else {
            rtems_mutex_lock(&(cqspi->bus_lock_mutex));
        }
    }
	ret = cqspi_mem_process(nor, op);
	if (ret) {
        cqspi_reg_capture(cqspi);
		spi_err("operation failed with %d\n", ret);
    }
    if (_System_all_cpu_state_Shutdown_Get() == false) {
        rtems_mutex_unlock(&(cqspi->bus_lock_mutex));
    }

exit:
	return ret;
}

static void cqspi_controller_init(struct cqspi_controller *cqspi)
{
	u32 reg;

	cqspi_controller_enable(cqspi, 0);

	/* Configure the remap address register, no remap */
	cqspi_reg_write(0, cqspi->iobase + CQSPI_REG_REMAP);

	/* Disable all interrupts. */
	cqspi_reg_write(0, cqspi->iobase + CQSPI_REG_IRQMASK);

	/* Configure the SRAM split to 1:1 . */
	cqspi_reg_write(cqspi->fifo_depth / 2, cqspi->iobase + CQSPI_REG_SRAMPARTITION);

	/* Load indirect trigger address. */
	cqspi_reg_write(cqspi->trigger_address,
	       cqspi->iobase + CQSPI_REG_INDIRECTTRIGGER);

	/* Program read watermark -- 1/2 of the FIFO. */
	cqspi_reg_write(cqspi->fifo_depth * cqspi->fifo_width / 2,
	       cqspi->iobase + CQSPI_REG_INDIRECTRDWATERMARK);
	/* Program write watermark -- 1/8 of the FIFO. */
	cqspi_reg_write(cqspi->fifo_depth * cqspi->fifo_width / 8,
	       cqspi->iobase + CQSPI_REG_INDIRECTWRWATERMARK);

	if (cqspi->use_direct_mode) {
		reg = cqspi_reg_read(cqspi->iobase + CQSPI_REG_CONFIG);
		reg |= CQSPI_REG_CONFIG_ENB_DIR_ACC_CTRL;
		cqspi_reg_write(reg, cqspi->iobase + CQSPI_REG_CONFIG);
    } else {
		reg = cqspi_reg_read(cqspi->iobase + CQSPI_REG_CONFIG);
		reg &= ~CQSPI_REG_CONFIG_ENB_DIR_ACC_CTRL;
		cqspi_reg_write(reg, cqspi->iobase + CQSPI_REG_CONFIG);
    }

	if (cqspi->use_dma_read) {
		reg = cqspi_reg_read(cqspi->iobase + CQSPI_REG_CONFIG);
		reg |= CQSPI_REG_CONFIG_DMA_MASK;
		cqspi_reg_write(reg, cqspi->iobase + CQSPI_REG_CONFIG);
    } else {
		reg = cqspi_reg_read(cqspi->iobase + CQSPI_REG_CONFIG);
		reg &= ~CQSPI_REG_CONFIG_DMA_MASK;
		cqspi_reg_write(reg, cqspi->iobase + CQSPI_REG_CONFIG);
    }

	cqspi_controller_enable(cqspi, 1);
}

static void cqspi_controller_detect_fifo_depth(struct cqspi_controller *cqspi)
{
    u32 reg, fifo_depth;

    /*
     * Bits N-1:0 are writable while bits 31:N are read as zero, with 2^N
     * the FIFO depth.
     */
    cqspi_reg_write(U32_MAX, cqspi->iobase + CQSPI_REG_SRAMPARTITION);
    reg = cqspi_reg_read(cqspi->iobase + CQSPI_REG_SRAMPARTITION);
    fifo_depth = reg + 1;

    /* FIFO depth of zero means no value from devicetree was provided. */
    if (cqspi->fifo_depth == 0) {
        cqspi->fifo_depth = fifo_depth;
        spi_debug("using FIFO depth of %u\n", fifo_depth);
    } else if (fifo_depth != cqspi->fifo_depth) {
        spi_warn("detected FIFO depth (%u) different from config (%u)\n",
                fifo_depth, cqspi->fifo_depth);
    }
}

///< 老qspi驱动netchip和switch v200芯片给的方式是通过寄存器判断，这里为驱动特殊处理，后续也可以将该判断移动到业务
#if 0 ////< #if defined(PS3OS_NETCHIP) || defined(PS3OS_SWITCH_V200)
static uint32_t get_mode_sel(void)
{
    int reg;
    reg = (*(volatile unsigned int *)(0x49010000UL));
    return (reg >> 3) & 0x3;
}

static void qspi_flash_bsp_config_mode(void)
{
    uint32_t mod_sel;

    mod_sel = get_mode_sel();
    ///< 0x0 boorom,单线；0x1 sram，单线；0x2 xip 单线；0x3 bootrom 四线
    if (mod_sel == 3 ) {
        g_cqspi_dev.io_wire_cnt = 4;
    } else {
        g_cqspi_dev.io_wire_cnt = 1;
    }
}
#endif

/* 内部函数，配置flash的driver strength
 * len:
 * 0x00  100%
 * 0x01  75%
 * 0x10  50%
 * 0x11  25%
 */
#if 0 ///< #if defined(PS3OS_EXPANDER_V200)
static void qspi_config_driver_strength(void)
{
    ///< config gpio driver strength
    cqspi_reg_write( (cqspi_reg_read(0x58601300) & (~(0x7<<0))), 0x58601300 );
    cqspi_reg_write( (cqspi_reg_read(0x58601300) & (~(0x7<<15))), 0x58601300 );
    cqspi_reg_write( (cqspi_reg_read(0x58601300) & (~(0x7<<18))), 0x58601300 );
    cqspi_reg_write( (cqspi_reg_read(0x58601300) & (~(0x7<<21))), 0x58601300 );
    cqspi_reg_write( (cqspi_reg_read(0x58601300) & (~(0x7<<24))), 0x58601300 );
    cqspi_reg_write( (cqspi_reg_read(0x58601300) | (0x6<<0)), 0x58601300 );
    cqspi_reg_write( (cqspi_reg_read(0x58601300) | (0x6<<15)), 0x58601300 );
    cqspi_reg_write( (cqspi_reg_read(0x58601300) | (0x6<<18)), 0x58601300 );
    cqspi_reg_write( (cqspi_reg_read(0x58601300) | (0x6<<21)), 0x58601300 );
    cqspi_reg_write( (cqspi_reg_read(0x58601300) | (0x6<<24)), 0x58601300 );
    return;
}
#endif

int cqspi_probe(struct cqspi_controller *cqspi)
{
    rtems_status_code status;
    const struct cqspi_driver_platdata *ddata;
    int i;
    int ret;

    qflashClkEnable();
    qflashReset();
    cqspi->master_ref_clk_hz = qflashClkGet();

#if 0 ///< #if defined(PS3OS_NETCHIP) || defined(PS3OS_SWITCH_V200)
    ///< 天箭和天秤V200，芯片建议通过寄存器判断当前是单线还是四线，后续芯片建议通过spi接口由业务设置
    qspi_flash_bsp_config_mode( );
#endif

    if (_System_all_cpu_state_Shutdown_Get() == false) {
        rtems_mutex_init(&(cqspi->bus_lock_mutex), "cqspi_lock");
        status = rtems_semaphore_create(rtems_build_name('C', 'Q', 'P', 'I'), 0,
                RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_FIFO, 0, &(cqspi->semaphore_id));
        if (status != RTEMS_SUCCESSFUL) {
            spi_err("Create sem for qspi failed(%u).\n", status);
            rtems_mutex_destroy(&(cqspi->bus_lock_mutex));
            ret = -EPERM;
            return ret;
        }
        status = rtems_interrupt_handler_install(cqspi->irq, "QSPI", RTEMS_INTERRUPT_UNIQUE,
                cqspi_irq_handler, (void *)cqspi);
        if (status != RTEMS_SUCCESSFUL) {
            spi_err("Cqspi failure requesting irq %u: %u\n", cqspi->irq, status);
            rtems_mutex_destroy(&(cqspi->bus_lock_mutex));
            rtems_semaphore_delete(cqspi->semaphore_id);
            ret = -EPERM;
            return ret;
        }
    }

    for (i = 0; i < CQSPI_MAX_CHIPSELECT; ++i) {
       cqspi->f_pdata[i].cqspi = &g_cqspi_dev;
    }
	/* write completion is supported by default */
    cqspi->wr_completion = true;
    ddata = cqspi->ddata;
    if (ddata != NULL) {
        if (ddata->quirks & CQSPI_NEEDS_WR_DELAY) {
            cqspi->wr_delay = 50 * DIV_ROUND_UP(NSEC_PER_SEC, cqspi->master_ref_clk_hz);
        }
        if (!(ddata->quirks & CQSPI_DISABLE_DAC_MODE)) {
            cqspi->use_direct_mode = true;
            cqspi->use_direct_mode_wr = true;
        }
        if (ddata->quirks & CQSPI_SUPPORT_EXTERNAL_DMA)
            cqspi->use_dma_read = true;
        if (ddata->quirks & CQSPI_NO_SUPPORT_WR_COMPLETION)
            cqspi->wr_completion = false;
        if (ddata->quirks & CQSPI_SLOW_SRAM)
            cqspi->slow_sram = true;
        if (ddata->quirks & CQSPI_NEEDS_APB_AHB_HAZARD_WAR)
            cqspi->apb_ahb_hazard = true;
    }

    cqspi_wait_idle(cqspi);
    ///< 按照手册，idle之后做其他配置，需要至少4个参考时钟(500M), 1.8G主频下需要15个NOP，其他平台按照15个NOP配置也没问题
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    cqspi_controller_enable(cqspi, 0);
    cqspi_controller_detect_fifo_depth(cqspi);
    cqspi_controller_init(cqspi);
    cqspi_controller_enable(cqspi, 1);
	cqspi->current_cs = -1;
	cqspi->sclk = 0;

	return 0;
}

void cqspi_probe_inner(void);
void cqspi_probe_inner(void)
{
    (void)cqspi_probe(&g_cqspi_dev);
    return;
}

void cqspi_remove(struct cqspi_controller *cqspi)
{
    rtems_status_code status;

	cqspi_controller_enable(cqspi, 0);
    rtems_mutex_destroy(&(cqspi->bus_lock_mutex));
    status = rtems_interrupt_handler_remove(cqspi->irq, cqspi_irq_handler, cqspi);
    if (status != RTEMS_SUCCESSFUL) {
        spi_err("Remove irq %u: %u.\n", cqspi->irq, status);
    }
    status = rtems_semaphore_delete(cqspi->semaphore_id);
    if (status != RTEMS_SUCCESSFUL) {
        spi_err("Delate semaphore %u: %u.\n", cqspi->semaphore_id, status);
    }

    return;
}

int cqspi_controller_config(struct cqspi_ctr_usr_conf *usr_conf)
{
    u32 reg;

    if (usr_conf == NULL) {
        return -EINVAL;
    }

    if (usr_conf->io_wire_cnt != INVALID_IO_WIRE_CNT) {
        g_cqspi_dev.io_wire_cnt = usr_conf->io_wire_cnt;
    }
    g_cqspi_dev.is_decoded_cs = usr_conf->is_decoded_cs;
    g_cqspi_dev.use_direct_mode = usr_conf->use_direct_mode;
    if (usr_conf->rd_no_irq) {
        (g_cqspi_dev.ddata)->quirks |= CQSPI_RD_NO_IRQ;
    } else {
        (g_cqspi_dev.ddata)->quirks &= ~CQSPI_RD_NO_IRQ;
    }
    if (usr_conf->wr_no_irq) {
        (g_cqspi_dev.ddata)->quirks |= CQSPI_WR_NO_IRQ;
    } else {
        (g_cqspi_dev.ddata)->quirks &= ~CQSPI_WR_NO_IRQ;
    }

    if (usr_conf->user_dev_conf) {
        for (int i = 0; i < CQSPI_MAX_CHIPSELECT_NO_DECODED; ++i) {
            g_cqspi_dev.f_pdata[i].clk_rate     = usr_conf->pdata[i].clk_rate;
            g_cqspi_dev.f_pdata[i].read_delay   = usr_conf->pdata[i].read_delay;
        }
    }

    ///< 无竞争场景（1 业务在调用flash或者nvsram前调用；2、coredump在操作flash前调用）
	cqspi_controller_enable(&g_cqspi_dev, 0);
    cqspi_wait_idle(&g_cqspi_dev);
    ///< 按照手册，idle之后做其他配置，需要至少4个参考时钟(500M), 1.8G主频下需要15个NOP，其他平台按照15个NOP配置也没问题
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
    __asm__ volatile("NOP");
	if (g_cqspi_dev.use_direct_mode) {
		reg = cqspi_reg_read(g_cqspi_dev.iobase + CQSPI_REG_CONFIG);
		reg |= CQSPI_REG_CONFIG_ENB_DIR_ACC_CTRL;
		cqspi_reg_write(reg, g_cqspi_dev.iobase + CQSPI_REG_CONFIG);
    } else {
		reg = cqspi_reg_read(g_cqspi_dev.iobase + CQSPI_REG_CONFIG);
		reg &= ~CQSPI_REG_CONFIG_ENB_DIR_ACC_CTRL;
		cqspi_reg_write(reg, g_cqspi_dev.iobase + CQSPI_REG_CONFIG);
    }
	cqspi_controller_enable(&g_cqspi_dev, 1);

    return 0;
}

RTEMS_SYSINIT_ITEM(cqspi_probe_inner, RTEMS_SYSINIT_DEVICE_DRIVERS, RTEMS_SYSINIT_ORDER_FIRST);
