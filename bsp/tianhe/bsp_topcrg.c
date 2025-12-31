/**
 * copyright (C), 2025, WuXi Stars Micro System Technologies Co.,Ltd
 *
 * @file bsp_topcrg.c
 * @author huyl3@starsmicrosystem.com
 * @date 2025/09/26
 * @brief
 */

#include <errno.h>
#include "common_defines.h"
#include "log_msg.h"
#include "bsp_topcrg.h"
#include "osp_interrupt.h"

#define INFO_REG2_ADDR      0xB800CFF8
#define MINISOC_IDLE_ACK_REG (0xBE100014)
#define MINISOC_IDLE_IRQ (0x7B)

typedef enum
{
    HW_PLATFORM_ASIC,   /* 0 */
    HW_PLATFORM_EDA,
    HW_PLATFORM_S2C,
    HW_PLATFORM_HAPS100,
    HW_PLATFORM_HAPS200,
    HW_PLATFORM_UVHS_1_VP1902, /* 5 */
    HW_PLATFORM_UVHS_4_VP1902,
    HW_PLATFORM_EMU_PZ1,
    HW_PLATFORM_EMU_VELOCE,
    HW_PLATFORM_MAX,
} hwPlatformType;

typedef struct __minisocIdleCb {
    pMinisocIdleIrqCallBack callback;
    void *arg;
} minisocIdleCb_s;

minisocIdleCb_s minisocIdleData;

/**
 * @brief 注册minisoc idle中断回调
 * @details 注册minisoc idle的复位握手中断
 * @param [in] callback, 表示回调函数
 * @param [in] callbackArg, 表示回调函数的入参
 * @param [inout] none
 * @return EXIT_SUCCESS 或 -EINVAL
 */
S32 minisocIdleCallbackRegister(pMinisocIdleIrqCallBack callback, void *callbackArg)
{
    S32 ret = EXIT_SUCCESS;

    if (NULL == callback) {
        LOGE("%s-%d: callback is NULL\r\n", __func__, __LINE__);
        ret = -EINVAL;
        goto exit;
    }

    minisocIdleData.callback = callback;
    minisocIdleData.arg = callbackArg;

exit:
    return ret;
}

static void minisocIdleIsr(void *arg)
{
    minisocIdleCb_s *drvData = (minisocIdleCb_s *)arg;

    if (NULL == drvData) {
        return;
    }

    if (drvData->callback != NULL) {
        drvData->callback(drvData->arg);
    }

    reg32Write(MINISOC_IDLE_ACK_REG, 1);
    asm volatile  ("wfi");
    asm volatile ("DSB");
    asm volatile ("ISB");
}

/**
 * @brief minisoc idle初始化
 * @details minisoc idle初始化
 * @param [in] none
 * @param [inout] none
 * @return OSP_RESOURCE_IN_USE 或 OSP_SUCCESSFUL，失败返回-EXIT_FAILURE
 */
S32 minisocIdleInit(void)
{
    S32 ret;

    ret = ospInterruptHandlerInstall(MINISOC_IDLE_IRQ, "minisoc_idle", OSP_INTERRUPT_UNIQUE,
        (OspInterruptHandler)minisocIdleIsr, &minisocIdleData);
    if (ret == OSP_RESOURCE_IN_USE) {
        LOGW("%s: minisoc idle irq already installed\r\n", __func__);
    } else if (ret == OSP_SUCCESSFUL) {
        ospInterruptVectorEnable((MINISOC_IDLE_IRQ));
    } else {
        LOGE("%s: failed to install minisoc idle irq  \r\n", __func__);
        ret = -EXIT_FAILURE;
    }

    return ret;
}

// info_reg2[31:28] hwPlatformType
const U8 *getHwPlatformType(void)
{
    const char *hwName = NULL;
    U32 info = reg32Read(INFO_REG2_ADDR);

    info >>= 28;
    info &= 0x0F;

    switch(info)
    {
        case HW_PLATFORM_ASIC:
            hwName = "ASIC";
            break;
        case HW_PLATFORM_EDA:
            hwName = "EDA";
            break;
        case HW_PLATFORM_S2C:
            hwName = "S2C";
            break;
        case HW_PLATFORM_HAPS100:
            hwName = "HAPS100";
            break;
        case HW_PLATFORM_HAPS200:
            hwName = "HAPS200";
            break;
        case HW_PLATFORM_UVHS_1_VP1902:
            hwName = "UVHS_1*VP1902";
            break;
        case HW_PLATFORM_UVHS_4_VP1902:
            hwName = "UVHS_4*VP1902";
            break;
        case HW_PLATFORM_EMU_PZ1:
            hwName = "EMU(PZ1)";
            break;
        case HW_PLATFORM_EMU_VELOCE:
            hwName = "EMU(VELOCE)";
            break;
        default:
            LOGE("platform ID %d\r\n", info);
            hwName = "Unknown";
            break;
    }

    LOGI("hw platform %s\r\n", hwName);

    return (const U8 *)hwName;
}

// info_reg2[15:0] bcd code like 0723/0827
U32 getHwPlatformVersion(U8 *strBuffer, U32 strLen)
{
    U32 info = reg32Read(INFO_REG2_ADDR);
    U16 ver = (U16)(info & 0xFFFF);
    U8  verStr[5];

    if(strBuffer == NULL || strLen == 0)
    {
        return 0;
    }

    verStr[0] = ((ver >> 12) & 0x0F) + '0';
    verStr[1] = ((ver >> 8) & 0x0F) + '0';
    verStr[2] = ((ver >> 4) & 0x0F) + '0';
    verStr[3] = ((ver ) & 0x0F) + '0';
    verStr[4] = 0;

    LOGI("hw platform version %s\r\n", verStr);

    if(strLen > sizeof(verStr))
    {
        strLen = sizeof(verStr);
    }

    memcpy(strBuffer, verStr, strLen);

    return strLen;
}