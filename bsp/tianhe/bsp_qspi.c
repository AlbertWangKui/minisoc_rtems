/**
 * copyright (C), 2022, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_qspi.h
 * @author taohb@starsmicrosystem.com
 * @date 2025/06/20
 * @brief qspi config for special bsp.
 */

#include "qspi/spi-cadence-quadspi.h"

static struct cqspi_driver_platdata g_star_qspi = {
    .quirks = CQSPI_DISABLE_DAC_MODE | CQSPI_NEEDS_APB_AHB_HAZARD_WAR | CQSPI_NO_SUPPORT_WR_COMPLETION,
};

struct cqspi_controller g_cqspi_dev = {
    .iobase = CQSPI_BASE,
    .trigger_address = CQSPI_TRIGGER_ADDRESS,
    .ahb_base = CQSPI_AHB_BASE,
    .ahb_size = CQSPI_AHB_SIZE,
    .irq = CQSPI_IRQ_NUM,
    .fifo_depth = 256,
    .fifo_width = 4,
    .num_chipselect = CQSPI_MAX_CHIPSELECT,
    .io_wire_cnt = 1,
    .is_decoded_cs = false,
    .use_direct_mode_wr = true,
    .rclk_en = 0,
    .current_cs = -1,
    .sclk = 0,
    .ddata = &g_star_qspi,
    ///< suport w25q512NW GD25LB512ME
    ///< cs_time寄存器配置成 0x16161616    capture配置成0x6
    .f_pdata = {
        [0] = {
            .cqspi = &g_cqspi_dev,
            .clk_rate = 25000000,
            .read_delay = 2,
            .tshsl_ns = 50,
            .tsd2d_ns = 50,
            .tchsh_ns = 37,
            .tslch_ns = 37,
            .cs = 0,
        },
    },
};