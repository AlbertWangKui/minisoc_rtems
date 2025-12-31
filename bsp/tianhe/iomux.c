/**
 * Copyright (C), 2024, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file iomux.c
 * @author yangzhl3 (yangzhl3@starsmicrosystem.com)
 * @date 2025/01/10
 * @brief
 *
 * @par ChangeLog:
 *
 * Date         Author          Description
 * 2025/01/10   yangzhl3        the first version
 *
 *
 */

#include "bsp_config.h"
#include "common_defines.h"
#include "bsp_api.h"
#include "bsp_device.h"
#include "stringto_wrapper.h"
#include "log_msg.h"
#include "iomux.h"
#include "kconfig.h"

S32 ioFuncConfig(IomuxIoNum_e ioNum, IomuxFunc_e funcSel)
{
    S32 ret = EXIT_SUCCESS;
    IomuxReg_s *ioMuxReg = (IomuxReg_s *)IOMUX_FUNC_BASE_REG;

    if (funcSel >= IOMUX_FUNC_INVALID) {
        LOGE("input iomux func Invalid\r\n");
        ret = -EXIT_FAILURE;
        goto exit;
    }

    if (ioNum >= IOMUX_TMUX_INVALID) {
        LOGE("input ioNum Invalid\r\n");
        ret = -EXIT_FAILURE;
        goto exit;        
    }

    ioMuxReg += ioNum;
    ioMuxReg->pinFunc.fields.mode = funcSel;

exit:
    return ret;
}