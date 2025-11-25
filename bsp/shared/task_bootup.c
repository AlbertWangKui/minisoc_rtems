/**
 * Copyright (C), 2020, WuXi Stars Micro System Technologies Co., Ltd.
 *
 * @file    osp_task_bootup.c
 * @author  lichenxiang
 * @date    2020.09.08
 * @brief   os 第一个任务启动实现
 * @note    NA
 */

#include <rtems.h>
#include <task_bootup.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <osp_version.h>
#include <osp_cpuopts.h>
#include <osp_timer.h>
#include <rtems/inttypes.h>
#include <rtems/bspIo.h>
#include <bsp.h>
#include <bspopts.h>
#include "version.h"
#include "board_init.h"
#include "drv_spi_api.h"
#include "log_msg.h"

OSP_LINKER_ROSET(_Appinit, osp_appinit_item); /*lint !e85 */

/**
 * @brief Configure terminal settings
 * @details This function configures the terminal for non-canonical mode
 *          with no echo and minimal input processing for shell interface.
 * @return None
 */
static void termiosConfig(void)
{
    struct termios term;

    if(tcgetattr(STDIN_FILENO, &term) < 0) {
        printf("Cannot get terminal attributes.\n");
    } else {
        /**
         * No echo, no canonical processing.
         */
        term.c_lflag &= ~(ECHO | ICANON | IEXTEN);

        /**
         * No sigint on BREAK, CR-to-NL off, input parity off,
         * don't strip 8th bit on input, output flow control off
         */
        term.c_iflag &= ~(INPCK | ISTRIP | IXON);
        term.c_cc[VMIN]  = 1;
        term.c_cc[VTIME] = 0;
        if(tcsetattr(STDIN_FILENO, TCSANOW, &term) < 0) {
            printf("Cannot set terminal attributes.\n");
        }
    }
    return;
}

/**
 * @brief Probe SPI device information
 * @details This function probes the SPI flash device to identify
 *          the device type, size, and ID information.
 * @return None
 */
static void spiDeviceProbe(void)
{
    int ret;

    /* init flash */
    struct channel_info chan_info = { .channel_id = 0 };
    struct spi_dev_info dev_info = { 0 };
    ret = spi_dev_probe(&chan_info, &dev_info);
    if(ret != 0) {
        LOGE("SPI Device Probe channel(%d) failed(%d).\n", chan_info.channel_id, ret);
    }
    else {
        LOGE("SPI Device Probe channel(%d) successfully.\n", chan_info.channel_id);
        LOGE("Device ID:");
        for(int i = 0; i < SPI_NOR_MAX_ID_LEN; i++) {
            LOGE("%02x", dev_info.id[i]);
        }
        LOGE(".\n");
        LOGE("Device size:0x%08x.\n", dev_info.size);
        LOGE("Device type:0x%08x.\n", dev_info.type);
    }
}

/**
 * @brief Main system bootup task
 * @details This function is the main entry point for RTEMS system
 *          initialization. It configures terminal, shows version info,
 *          probes SPI devices, initializes board, and calls all
 *          registered application initialization functions.
 * @param [in] argument Task argument (unused)
 * @return None (task calls rtems_task_exit)
 */
rtems_task ospTaskBootup(rtems_task_argument argument)
{
    const osp_appinit_item *item;
    struct timeval startus;

    (void)argument;

    ///< 配置terminal终端
    termiosConfig( );

    (void)ospClockGetUptimeTimeval(&startus);

    socShowVersionInfo();
    spiDeviceProbe();

    boardInit();

    RTEMS_LINKER_SET_FOREACH(_Appinit, item)
    {
        (*item->handler)( );
    }

    rtems_task_exit( );
}
