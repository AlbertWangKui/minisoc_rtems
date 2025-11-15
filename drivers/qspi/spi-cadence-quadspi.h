/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file spi-cadence-quadspi.h
 * @brief
 * @author tianye
 * @version
 * @date 2024-05-21
 */

#ifndef __SPI_CADENCE_QUADSPI_H__
#define __SPI_CADENCE_QUADSPI_H__

#include <strings.h>
#include <rtems.h>
#include <rtems/bspIo.h>
#include "drv_spi_device_inner.h"
#include "bsp_config.h" 

#define CQSPI_BASE              ((void *)(SYS_QSPI_BASE_ADRS))
#define CQSPI_TRIGGER_ADDRESS   (SYS_QSPI_MAP_FLASH1)           ///< 直接映射地址
#define CQSPI_AHB_BASE          ((void *)SYS_QSPI_MAP_FLASH1)   ///< 直接映射地址
#define CQSPI_AHB_SIZE          (128 * 1024 * 1024U)    ///< 直接映射地址范围
#define CQSPI_IRQ_NUM           (SYS_INT_NUM_QSPI)

/* Quirks */
#define CQSPI_NEEDS_WR_DELAY		BIT(0)
#define CQSPI_DISABLE_DAC_MODE		BIT(1)
#define CQSPI_SUPPORT_EXTERNAL_DMA	BIT(2)
#define CQSPI_NO_SUPPORT_WR_COMPLETION	BIT(3)
#define CQSPI_SLOW_SRAM		BIT(4)
#define CQSPI_NEEDS_APB_AHB_HAZARD_WAR	BIT(5)
#define CQSPI_RD_NO_IRQ                 BIT(6)
#define CQSPI_WR_NO_IRQ                 BIT(7)

/* Capabilities */
#define CQSPI_SUPPORTS_OCTAL		BIT(0)

static inline __attribute__((const))
int __ilog2_u32(u32 n)
{
    return fls(n) - 1;
}
static inline __attribute__((const))
int __ilog2_u64(u64 n)
{
    return flsll(n) - 1;
}
#define ilog2(n) \
    ( \
      __builtin_constant_p(n) ?   \
      ((n) < 2 ? 0 :          \
       63 - __builtin_clzll(n)) : \
       (sizeof(n) <= 4) ?      \
       __ilog2_u32(n) :        \
       __ilog2_u64(n)          \
       )
#define CQSPI_OP_WIDTH(part) ((part).nbytes ? ilog2((part).buswidth) : 0)

struct cqspi_driver_platdata {
	u32 hwcaps_mask;
	u8 quirks;
};

/* 正常来说，时序参数需要根据不同flash做不同配置，这就需要先以低频probe获取型号
 * 然后再选择合适参数。当前芯片所有项目在flash选型时都考虑频率和命令兼容。所以
 * 为了简化流程，我们直接配置固定值。
*/
struct cqspi_flash_pdata {
    struct cqspi_controller *cqspi;
	u32		clk_rate;
	u32		read_delay;
	u32		tshsl_ns;
	u32		tsd2d_ns;
	u32		tchsh_ns;
	u32		tslch_ns;
	u8		cs;
};

struct cqspi_controller {
    void *iobase;
    u32	trigger_address;
    void *ahb_base;
    u32 ahb_size;
	u32	mmap_phys_base;
    u32	irq;
    u32	fifo_depth;
    u32	fifo_width;
    u32	num_chipselect;
    u8 io_wire_cnt;
    bool is_decoded_cs;
	bool rclk_en;
	int	current_cs;
	u32 sclk;
    struct cqspi_driver_platdata *ddata;
	u32 master_ref_clk_hz;
    rtems_id semaphore_id;
	bool wr_completion;
	u32	wr_delay;
	bool use_direct_mode;
	bool use_direct_mode_wr;
	struct cqspi_flash_pdata f_pdata[CQSPI_MAX_CHIPSELECT];
	bool use_dma_read;
	u32	 pd_dev_id;
	bool slow_sram;
    bool apb_ahb_hazard;
    rtems_mutex bus_lock_mutex;
};

/* Operation timeout value */
#define CQSPI_TIMEOUT_MS			1000
#define CQSPI_READ_TIMEOUT_MS			20

/* Runtime_pm autosuspend delay */
#define CQSPI_AUTOSUSPEND_TIMEOUT		2000

#define CQSPI_DUMMY_CLKS_PER_BYTE		8
#define CQSPI_DUMMY_BYTES_MAX			4
#define CQSPI_DUMMY_CLKS_MAX			31

#define CQSPI_STIG_DATA_LEN_MAX         8

/* Register map */
#define CQSPI_REG_CONFIG			0x00
#define CQSPI_REG_CONFIG_ENABLE_MASK		BIT(0)
#define CQSPI_REG_CONFIG_ENB_DIR_ACC_CTRL	BIT(7)
#define CQSPI_REG_CONFIG_DECODE_MASK		BIT(9)
#define CQSPI_REG_CONFIG_CHIPSELECT_LSB		10
#define CQSPI_REG_CONFIG_DMA_MASK		BIT(15)
#define CQSPI_REG_CONFIG_BAUD_LSB		19
#define CQSPI_REG_CONFIG_DTR_PROTO		BIT(24)
#define CQSPI_REG_CONFIG_DUAL_OPCODE		BIT(30)
#define CQSPI_REG_CONFIG_IDLE_LSB		31
#define CQSPI_REG_CONFIG_CHIPSELECT_MASK	0xF
#define CQSPI_REG_CONFIG_BAUD_MASK		0xF

#define CQSPI_REG_RD_INSTR			0x04
#define CQSPI_REG_RD_INSTR_OPCODE_LSB		0
#define CQSPI_REG_RD_INSTR_TYPE_INSTR_LSB	8
#define CQSPI_REG_RD_INSTR_TYPE_ADDR_LSB	12
#define CQSPI_REG_RD_INSTR_TYPE_DATA_LSB	16
#define CQSPI_REG_RD_INSTR_MODE_EN_LSB		20
#define CQSPI_REG_RD_INSTR_DUMMY_LSB		24
#define CQSPI_REG_RD_INSTR_TYPE_INSTR_MASK	0x3
#define CQSPI_REG_RD_INSTR_TYPE_ADDR_MASK	0x3
#define CQSPI_REG_RD_INSTR_TYPE_DATA_MASK	0x3
#define CQSPI_REG_RD_INSTR_DUMMY_MASK		0x1F

#define CQSPI_REG_WR_INSTR			0x08
#define CQSPI_REG_WR_INSTR_OPCODE_LSB		0
#define CQSPI_REG_WR_INSTR_TYPE_ADDR_LSB	12
#define CQSPI_REG_WR_INSTR_TYPE_DATA_LSB	16

#define CQSPI_REG_DELAY				0x0C
#define CQSPI_REG_DELAY_TSLCH_LSB		0
#define CQSPI_REG_DELAY_TCHSH_LSB		8
#define CQSPI_REG_DELAY_TSD2D_LSB		16
#define CQSPI_REG_DELAY_TSHSL_LSB		24
#define CQSPI_REG_DELAY_TSLCH_MASK		0xFF
#define CQSPI_REG_DELAY_TCHSH_MASK		0xFF
#define CQSPI_REG_DELAY_TSD2D_MASK		0xFF
#define CQSPI_REG_DELAY_TSHSL_MASK		0xFF

#define CQSPI_REG_READCAPTURE			0x10
#define CQSPI_REG_READCAPTURE_BYPASS_LSB	0
#define CQSPI_REG_READCAPTURE_DELAY_LSB		1
#define CQSPI_REG_READCAPTURE_DELAY_MASK	0xF

#define CQSPI_REG_SIZE				0x14
#define CQSPI_REG_SIZE_ADDRESS_LSB		0
#define CQSPI_REG_SIZE_PAGE_LSB			4
#define CQSPI_REG_SIZE_BLOCK_LSB		16
#define CQSPI_REG_SIZE_ADDRESS_MASK		0xF
#define CQSPI_REG_SIZE_PAGE_MASK		0xFFF
#define CQSPI_REG_SIZE_BLOCK_MASK		0x3F

#define CQSPI_REG_SRAMPARTITION			0x18
#define CQSPI_REG_INDIRECTTRIGGER		0x1C

#define CQSPI_REG_DMA				0x20
#define CQSPI_REG_DMA_SINGLE_LSB		0
#define CQSPI_REG_DMA_BURST_LSB			8
#define CQSPI_REG_DMA_SINGLE_MASK		0xFF
#define CQSPI_REG_DMA_BURST_MASK		0xFF

#define CQSPI_REG_REMAP				0x24
#define CQSPI_REG_MODE_BIT			0x28

#define CQSPI_REG_SDRAMLEVEL			0x2C
#define CQSPI_REG_SDRAMLEVEL_RD_LSB		0
#define CQSPI_REG_SDRAMLEVEL_WR_LSB		16
#define CQSPI_REG_SDRAMLEVEL_RD_MASK		0xFFFF
#define CQSPI_REG_SDRAMLEVEL_WR_MASK		0xFFFF

#define CQSPI_REG_WR_COMPLETION_CTRL		0x38
#define CQSPI_REG_WR_DISABLE_AUTO_POLL		BIT(14)

#define CQSPI_REG_IRQSTATUS			0x40
#define CQSPI_REG_IRQMASK			0x44

#define CQSPI_REG_INDIRECTRD			0x60
#define CQSPI_REG_INDIRECTRD_START_MASK		BIT(0)
#define CQSPI_REG_INDIRECTRD_CANCEL_MASK	BIT(1)
#define CQSPI_REG_INDIRECTRD_DONE_MASK		BIT(5)

#define CQSPI_REG_INDIRECTRDWATERMARK		0x64
#define CQSPI_REG_INDIRECTRDSTARTADDR		0x68
#define CQSPI_REG_INDIRECTRDBYTES		0x6C

#define CQSPI_REG_CMDCTRL			0x90
#define CQSPI_REG_CMDCTRL_EXECUTE_MASK		BIT(0)
#define CQSPI_REG_CMDCTRL_INPROGRESS_MASK	BIT(1)
#define CQSPI_REG_CMDCTRL_DUMMY_LSB		7
#define CQSPI_REG_CMDCTRL_WR_BYTES_LSB		12
#define CQSPI_REG_CMDCTRL_WR_EN_LSB		15
#define CQSPI_REG_CMDCTRL_ADD_BYTES_LSB		16
#define CQSPI_REG_CMDCTRL_ADDR_EN_LSB		19
#define CQSPI_REG_CMDCTRL_RD_BYTES_LSB		20
#define CQSPI_REG_CMDCTRL_RD_EN_LSB		23
#define CQSPI_REG_CMDCTRL_OPCODE_LSB		24
#define CQSPI_REG_CMDCTRL_WR_BYTES_MASK		0x7
#define CQSPI_REG_CMDCTRL_ADD_BYTES_MASK	0x3
#define CQSPI_REG_CMDCTRL_RD_BYTES_MASK		0x7
#define CQSPI_REG_CMDCTRL_DUMMY_MASK		0x1F

#define CQSPI_REG_INDIRECTWR			0x70
#define CQSPI_REG_INDIRECTWR_START_MASK		BIT(0)
#define CQSPI_REG_INDIRECTWR_CANCEL_MASK	BIT(1)
#define CQSPI_REG_INDIRECTWR_DONE_MASK		BIT(5)

#define CQSPI_REG_INDIRECTWRWATERMARK		0x74
#define CQSPI_REG_INDIRECTWRSTARTADDR		0x78
#define CQSPI_REG_INDIRECTWRBYTES		0x7C

#define CQSPI_REG_INDTRIG_ADDRRANGE		0x80

#define CQSPI_REG_CMDADDRESS			0x94
#define CQSPI_REG_CMDREADDATALOWER		0xA0
#define CQSPI_REG_CMDREADDATAUPPER		0xA4
#define CQSPI_REG_CMDWRITEDATALOWER		0xA8
#define CQSPI_REG_CMDWRITEDATAUPPER		0xAC

#define CQSPI_REG_POLLING_STATUS		0xB0
#define CQSPI_REG_POLLING_STATUS_DUMMY_LSB	16

#define CQSPI_REG_OP_EXT_LOWER			0xE0
#define CQSPI_REG_OP_EXT_READ_LSB		24
#define CQSPI_REG_OP_EXT_WRITE_LSB		16
#define CQSPI_REG_OP_EXT_STIG_LSB		0

#define CQSPI_REG_VERSAL_DMA_SRC_ADDR		0x1000

#define CQSPI_REG_VERSAL_DMA_DST_ADDR		0x1800
#define CQSPI_REG_VERSAL_DMA_DST_SIZE		0x1804

#define CQSPI_REG_VERSAL_DMA_DST_CTRL		0x180C

#define CQSPI_REG_VERSAL_DMA_DST_I_STS		0x1814
#define CQSPI_REG_VERSAL_DMA_DST_I_EN		0x1818
#define CQSPI_REG_VERSAL_DMA_DST_I_DIS		0x181C
#define CQSPI_REG_VERSAL_DMA_DST_DONE_MASK	BIT(1)

#define CQSPI_REG_VERSAL_DMA_DST_ADDR_MSB	0x1828

#define CQSPI_REG_VERSAL_DMA_DST_CTRL_VAL	0xF43FFA00
#define CQSPI_REG_VERSAL_ADDRRANGE_WIDTH_VAL	0x6

/* Interrupt status bits */
#define CQSPI_REG_IRQ_MODE_ERR			BIT(0)
#define CQSPI_REG_IRQ_UNDERFLOW			BIT(1)
#define CQSPI_REG_IRQ_IND_COMP			BIT(2)
#define CQSPI_REG_IRQ_IND_RD_REJECT		BIT(3)
#define CQSPI_REG_IRQ_WR_PROTECTED_ERR		BIT(4)
#define CQSPI_REG_IRQ_ILLEGAL_AHB_ERR		BIT(5)
#define CQSPI_REG_IRQ_WATERMARK			BIT(6)
#define CQSPI_REG_IRQ_IND_SRAM_FULL		BIT(12)

#define CQSPI_IRQ_MASK_RD		(CQSPI_REG_IRQ_WATERMARK	| \
					 CQSPI_REG_IRQ_IND_SRAM_FULL	| \
					 CQSPI_REG_IRQ_IND_COMP)

#define CQSPI_IRQ_MASK_WR		(CQSPI_REG_IRQ_IND_COMP		| \
					 CQSPI_REG_IRQ_WATERMARK	| \
					 CQSPI_REG_IRQ_UNDERFLOW)

#define CQSPI_IRQ_STATUS_MASK		0x1FFFF
#define CQSPI_DMA_UNALIGN		0x3

#define CQSPI_REG_VERSAL_DMA_VAL		0x602

int cqspi_probe(struct cqspi_controller *cqspi);
void cqspi_remove(struct cqspi_controller *cqspi);
int qspi_exec_mem_op(struct spi_nor *nor, const struct spi_mem_op *op);
void cqspi_reinit(void);

#endif
