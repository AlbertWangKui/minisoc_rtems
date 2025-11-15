/**
 * Copyright (C), 2023, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file minisoc_init.c
 * @author taohb@starsmicrosystem.com
 * @date 2023.11.18
 * @brief bsp init, it is interface for minisoc library.
 */

#include "common_defines.h"
#include "bsp_api.h"

#include "log_msg.h"
#include "udelay.h"
#include "minisoc_init.h"
#include "drv_spi_api.h"

#include <rtems.h>
#include <rtems/score/todimpl.h>

static int32_t bsp_drives_init(void)
{
    int32_t ret = EXIT_SUCCESS;

    return ret;
}

/**
 * @brief Initialize System on Chip (SoC)
 * @details This function performs complete SoC initialization including
 *          BSP drivers and system components setup.
 * @return EXIT_SUCCESS if initialization completes successfully
 * @return -EXIT_FAILURE if any initialization step fails
 */
S32 socInit(void)
{
    int32_t ret = EXIT_SUCCESS;

    /* drivers init */
    if (EXIT_SUCCESS != bsp_drives_init()) {
        LOGE("%s(): bsp init failed\r\n", __func__);
        ret = -EXIT_FAILURE;
    }

    return ret;
}
