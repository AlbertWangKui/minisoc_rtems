/**
 * copyright (C), 2022, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file init.c
 * @author shaogl@starsmicrosystem.com
 * @date 2022/09/30
 * @brief init模块代码
 * @note None
 * @version v1.0
 */

#include "common_defines.h"
#include "bsp_config.h"
#include "bsp_api.h"
#include "minisoc_init.h"
#include "version.h"
#include "log_msg.h"
#include "drv_shell.h"
#include "osp_appinit.h"
#include <rtems/rtems/tasks.h>
#include "cli_core_api.h"

static void minisoc_task_init(void)
{
    if (EXIT_SUCCESS != socInit()) {
        LOGE("%s():init soc failed\r\n", __func__);
    }

#ifdef CONFIG_USING_DSHELL
    driver_shell_init(true);
#elif defined(CONFIG_USING_CLI_CORE)
    if (EXIT_SUCCESS != clishellInit()) {
        LOGE("%s():clishellInit soc failed\r\n", __func__);
    }
#endif
    rtems_task_exit();
}

OSP_APPINIT_ITEM(minisoc_task_init, OSP_APPINIT_MIDDLE, OSP_APPINIT_ORDER_SECOND);

